//------------------------------------------------------------------------------
//  sokol_shape with JoltPhysics demo.
//------------------------------------------------------------------------------
#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#define SOKOL_IMPL
#define SOKOL_GLCORE
// #define SOKOL_D3D11
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_glue.h"
#include "util/sokol_shape.h"
#include "util/sokol_debugtext.h"
#include "HandmadeMath.h"
// #include "util/camera.h"
#include "util/camera2.h"
#include "shaders/shapes.glsl.h"

#include <iostream>
#include <cstdarg>
#include <thread>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

static void TraceImpl(const char *inFMT, ...)
{
    // Format the message
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);

    // Print to the TTY
    std::cout << buffer << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, JPH::uint inLine)
{
    // Print to the TTY
    std::cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << std::endl;

    // Breakpoint
    return true;
};

#endif // JPH_ENABLE_ASSERTS

namespace Layers
{
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
            case Layers::NON_MOVING:
                return inObject2 == Layers::MOVING; // Non moving only collides with moving
            case Layers::MOVING:
                return true; // Moving collides with everything
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

namespace BroadPhaseLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr JPH::uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
	BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual JPH::uint GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char * GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
	{
		switch ((JPH::BroadPhaseLayer::Type)inLayer)
		{
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// An example contact listener
class MyContactListener : public JPH::ContactListener
{
public:
	// See: ContactListener
	virtual JPH::ValidateResult	OnContactValidate(const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult) override
	{
		std::cout << "Contact validate callback" << std::endl;

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
		std::cout << "A contact was added" << std::endl;
	}

	virtual void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
		std::cout << "A contact was persisted" << std::endl;
	}

	virtual void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
	{
		std::cout << "A contact was removed" << std::endl;
	}
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
	virtual void OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
		std::cout << "A body got activated" << std::endl;
	}

	virtual void OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
		std::cout << "A body went to sleep" << std::endl;
	}
};

typedef struct {
    HMM_Vec3 pos;
    sshape_element_range_t draw;
} shape_t;

static struct {
    struct {
        sg_pass_action pass_action;
        sg_pipeline pip_fill;
        sg_pipeline pip_lines;
        sg_buffer vbuf;
        sg_buffer ibuf;
        shapes_vs_params_t vs_params;
        bool line_mode;
        float rx, ry;
        camera2_t camera;
    } graphics;
	struct {
		JPH::PhysicsSystem physics_system;
		JPH::BodyID floor_id;
		JPH::BodyID sphere_id;

        JPH::Vec3 floor_xyz = JPH::Vec3(10.0f, 0.2f, 10.0f);
        float sphere_radius = 0.5f;

		JPH::TempAllocatorImpl* temp_allocator = nullptr;
		JPH::JobSystemThreadPool* job_system = nullptr;

        BPLayerInterfaceImpl* broad_phase_layer_interface = nullptr;
        ObjectVsBroadPhaseLayerFilterImpl* object_vs_broadphase_layer_filter = nullptr;
        ObjectLayerPairFilterImpl* object_vs_object_layer_filter = nullptr;

        MyBodyActivationListener* body_activation_listener = nullptr;
        MyContactListener* contact_listener = nullptr;
	} physics;
} state;

HMM_Mat4 jolt_mat4_to_hmm_mat4(JPH::Mat44 in) {
    HMM_Mat4 result;

    for (int i = 0; i < 4; i++) {
        JPH::Vec4 v = in.GetColumn4(i);
        result.Columns[i] = HMM_V4(v.mF32[0], v.mF32[1], v.mF32[2], v.mF32[3]);
    }

    return result;
}

static void draw_jolt_box(JPH::BodyID box_id, JPH::Vec3 box_xyz) {
    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

    JPH::Mat44 world = body_interface.GetWorldTransform(box_id);

    HMM_Mat4 proj = state.graphics.camera.proj;
    HMM_Mat4 view = state.graphics.camera.view;
    HMM_Mat4 view_proj = HMM_MulM4(proj, view);

    HMM_Mat4 model = jolt_mat4_to_hmm_mat4(world);

    sshape_vertex_t vertices[6 * 1024];
    uint16_t indices[16 * 1024];

    sshape_buffer_t buf = {};
    buf.vertices.buffer = SSHAPE_RANGE(vertices);
    buf.indices.buffer  = SSHAPE_RANGE(indices);

    sshape_box_t _sshape_box_t{};
    _sshape_box_t.width  = box_xyz.mF32[0];
    _sshape_box_t.height = box_xyz.mF32[1];
    _sshape_box_t.depth  = box_xyz.mF32[2];
    _sshape_box_t.tiles  = 20;
    _sshape_box_t.random_colors = true;
    buf = sshape_build_box(&buf, &_sshape_box_t);
    sshape_element_range_t draw = sshape_element_range(&buf);

    const sg_buffer_desc vbuf_desc = sshape_vertex_buffer_desc(&buf);
    const sg_buffer_desc ibuf_desc = sshape_index_buffer_desc(&buf);
    state.graphics.vbuf = sg_make_buffer(&vbuf_desc);
    state.graphics.ibuf = sg_make_buffer(&ibuf_desc);

    sg_bindings _sg_bindings{};
    _sg_bindings.vertex_buffers[0] = state.graphics.vbuf;
    _sg_bindings.index_buffer = state.graphics.ibuf;
    sg_apply_bindings(_sg_bindings);
    state.graphics.vs_params.mvp = HMM_MulM4(view_proj, model);
    sg_apply_uniforms(UB_shapes_vs_params, SG_RANGE(state.graphics.vs_params));
    sg_draw(draw.base_element, draw.num_elements, 1);

    sg_destroy_buffer(state.graphics.vbuf);
    sg_destroy_buffer(state.graphics.ibuf);
}

static void draw_jolt_sphere(JPH::BodyID sphere_id, float radius) {
    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

    JPH::Mat44 world = body_interface.GetWorldTransform(sphere_id);

    HMM_Mat4 proj = state.graphics.camera.proj;
    HMM_Mat4 view = state.graphics.camera.view;
    HMM_Mat4 view_proj = HMM_MulM4(proj, view);

    HMM_Mat4 model = jolt_mat4_to_hmm_mat4(world);

    sshape_vertex_t vertices[6 * 1024];
    uint16_t indices[16 * 1024];

    sshape_buffer_t buf = {};
    buf.vertices.buffer = SSHAPE_RANGE(vertices);
    buf.indices.buffer  = SSHAPE_RANGE(indices);

    sshape_sphere_t _sshape_sphere_t{};
    _sshape_sphere_t.radius = radius;
    _sshape_sphere_t.slices = 36;
    _sshape_sphere_t.stacks = 20;
    _sshape_sphere_t.random_colors = true;
    buf = sshape_build_sphere(&buf, &_sshape_sphere_t);
    sshape_element_range_t draw = sshape_element_range(&buf);

    const sg_buffer_desc vbuf_desc = sshape_vertex_buffer_desc(&buf);
    const sg_buffer_desc ibuf_desc = sshape_index_buffer_desc(&buf);
    state.graphics.vbuf = sg_make_buffer(&vbuf_desc);
    state.graphics.ibuf = sg_make_buffer(&ibuf_desc);

    sg_bindings _sg_bindings{};
    _sg_bindings.vertex_buffers[0] = state.graphics.vbuf;
    _sg_bindings.index_buffer = state.graphics.ibuf;
    sg_apply_bindings(_sg_bindings);
    state.graphics.vs_params.mvp = HMM_MulM4(view_proj, model);
    sg_apply_uniforms(UB_shapes_vs_params, SG_RANGE(state.graphics.vs_params));
    sg_draw(draw.base_element, draw.num_elements, 1);

    sg_destroy_buffer(state.graphics.vbuf);
    sg_destroy_buffer(state.graphics.ibuf);
}

static void create_physics_scene() {
    using namespace JPH::literals;

    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

    JPH::BoxShapeSettings floor_shape_settings(state.physics.floor_xyz);
    floor_shape_settings.SetEmbedded();

    JPH::ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
    JPH::ShapeRefC floor_shape = floor_shape_result.Get();

    JPH::BodyCreationSettings floor_settings(floor_shape, JPH::RVec3(0.0_r, -1.0_r, 0.0_r), JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
    state.physics.floor_id = body_interface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);

    JPH::BodyCreationSettings sphere_settings(new JPH::SphereShape(state.physics.sphere_radius), JPH::RVec3(0.0_r, 5.0_r, 0.0_r), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    state.physics.sphere_id = body_interface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

    body_interface.SetLinearVelocity(state.physics.sphere_id, JPH::Vec3(0.0_r, -5.0_r, 0.0_r));
}

static void clear_physics_scene() {
    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

    body_interface.RemoveBody(state.physics.sphere_id);
    body_interface.DestroyBody(state.physics.sphere_id);

    body_interface.RemoveBody(state.physics.floor_id);
    body_interface.DestroyBody(state.physics.floor_id);
}

static void init(void) {
	// graphics init
	{
		sg_desc _sg_desc{};
	    _sg_desc.environment = sglue_environment();
	    _sg_desc.logger.func = slog_func;
	    sg_setup(&_sg_desc);

	    sdtx_desc_t _sdtx_desc{};
	    _sdtx_desc.fonts[0] = sdtx_font_oric();
	    _sdtx_desc.logger.func = slog_func;
	    sdtx_setup(&_sdtx_desc);

	    state.graphics.line_mode = false;

	    // clear to black
	    state.graphics.pass_action.colors[0] = { .load_action = SG_LOADACTION_CLEAR, .clear_value = { 0.0f, 0.0f, 0.0f, 1.0f } };

	    // initialize camera helper
	    camera2_desc_t camdesc = {0};
	    camdesc.nearz = 0.1f;
	    camdesc.farz = 100.0f;
	    camdesc.pos = HMM_V3(0.0f, 1.5f, 6.0f);
	    camdesc.up = HMM_V3(0.0f, 1.0f, 0.0f);
	    camdesc.yaw = -90.0f;
	    camdesc.pitch = 0.0f;
	    cam_init(&state.graphics.camera, &camdesc);

	    // shader and pipeline object
	    sg_pipeline_desc _fill_pipeline_desc = {};
	    _fill_pipeline_desc.shader = sg_make_shader(shapes_shader_desc(sg_query_backend()));
	    _fill_pipeline_desc.layout.buffers[0] = sshape_vertex_buffer_layout_state();
	    _fill_pipeline_desc.layout.attrs[0] = sshape_position_vertex_attr_state();
	    _fill_pipeline_desc.layout.attrs[1] = sshape_normal_vertex_attr_state();
	    _fill_pipeline_desc.layout.attrs[2] = sshape_texcoord_vertex_attr_state();
	    _fill_pipeline_desc.layout.attrs[3] = sshape_color_vertex_attr_state();
	    _fill_pipeline_desc.index_type = SG_INDEXTYPE_UINT16;
	    _fill_pipeline_desc.cull_mode = SG_CULLMODE_NONE;
	    _fill_pipeline_desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
	    _fill_pipeline_desc.depth.write_enabled = true;
	    _fill_pipeline_desc.label = "fill-pipeline";
	    state.graphics.pip_fill = sg_make_pipeline(&_fill_pipeline_desc);

	    sg_pipeline_desc _line_pipeline_desc = {};
	    _line_pipeline_desc.shader = sg_make_shader(shapes_shader_desc(sg_query_backend()));
	    _line_pipeline_desc.layout.buffers[0] = sshape_vertex_buffer_layout_state();
	    _line_pipeline_desc.layout.attrs[0] = sshape_position_vertex_attr_state();
	    _line_pipeline_desc.layout.attrs[1] = sshape_normal_vertex_attr_state();
	    _line_pipeline_desc.layout.attrs[2] = sshape_texcoord_vertex_attr_state();
	    _line_pipeline_desc.layout.attrs[3] = sshape_color_vertex_attr_state();
	    _line_pipeline_desc.index_type = SG_INDEXTYPE_UINT16;
	    _line_pipeline_desc.cull_mode = SG_CULLMODE_BACK;
	    _line_pipeline_desc.primitive_type = SG_PRIMITIVETYPE_LINES;
	    _line_pipeline_desc.depth.compare = SG_COMPAREFUNC_LESS_EQUAL;
	    _line_pipeline_desc.depth.write_enabled = true;
	    _line_pipeline_desc.label = "lines-pipeline";
	    state.graphics.pip_lines = sg_make_pipeline(&_line_pipeline_desc);
	}

	// physics init
	{
		JPH::RegisterDefaultAllocator();

		JPH::Trace = TraceImpl;
		JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl);

		JPH::Factory::sInstance = new JPH::Factory();

		JPH::RegisterTypes();

		state.physics.temp_allocator = new JPH::TempAllocatorImpl(10*1024*1024);
		state.physics.job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

		const JPH::uint cMaxBodies = 1024;
		const JPH::uint cNumBodyMutexes = 0;
		const JPH::uint cMaxBodyPairs = 1024;
		const JPH::uint cMaxContactConstraints = 1024;

        state.physics.broad_phase_layer_interface = new BPLayerInterfaceImpl();
        state.physics.object_vs_broadphase_layer_filter = new ObjectVsBroadPhaseLayerFilterImpl();
        state.physics.object_vs_object_layer_filter = new ObjectLayerPairFilterImpl();

		state.physics.physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, *state.physics.broad_phase_layer_interface, *state.physics.object_vs_broadphase_layer_filter, *state.physics.object_vs_object_layer_filter);

        state.physics.body_activation_listener = new MyBodyActivationListener();
		state.physics.physics_system.SetBodyActivationListener(state.physics.body_activation_listener);

        state.physics.contact_listener = new MyContactListener();
		state.physics.physics_system.SetContactListener(state.physics.contact_listener);
        state.physics.physics_system.OptimizeBroadPhase();

        create_physics_scene();
	}
}

static void cleanup(void) {
	// physics deinit
	{
        clear_physics_scene();

		delete state.physics.temp_allocator;
		delete state.physics.job_system;

        delete state.physics.broad_phase_layer_interface;
        delete state.physics.object_vs_broadphase_layer_filter;
        delete state.physics.object_vs_object_layer_filter;

        delete state.physics.body_activation_listener;
        delete state.physics.contact_listener;

		JPH::UnregisterTypes();

		delete JPH::Factory::sInstance;
		JPH::Factory::sInstance = nullptr;
	}

	// graphics deinit
	{
		sdtx_shutdown();
		sg_shutdown();
	}
}

static void frame(void) {
    const int cCollisionSteps = 2;
    state.physics.physics_system.Update((float)sapp_frame_duration(), cCollisionSteps, state.physics.temp_allocator, state.physics.job_system);

    const int fb_width = sapp_width();
    const int fb_height = sapp_height();
    cam_update(&state.graphics.camera, fb_width, fb_height);

    // help text
    sdtx_canvas(fb_width*0.5f, fb_height*0.5f);
    sdtx_pos(0.5f, 0.5f);
    sdtx_puts("press key to switch draw mode:\n\n"
              "  1: vertex normals\n"
              "  2: texture coords\n"
              "  3: vertex color\n"
              "  4: line/fill mode\n"
              "  R: reset physics\n");

    // view-projection matrix...
    HMM_Mat4 proj = state.graphics.camera.proj;
    HMM_Mat4 view = state.graphics.camera.view;
    HMM_Mat4 view_proj = HMM_MulM4(proj, view);

    // model-rotation matrix
    const float t = (float)(sapp_frame_duration() * 60.0);
    state.graphics.rx += 1.0f * t * HMM_DegToRad;
    state.graphics.ry += 2.0f * t * HMM_DegToRad;
    HMM_Mat4 rxm = HMM_Rotate_RH(state.graphics.rx, HMM_V3(1.0f, 0.0f, 0.0f));
    HMM_Mat4 rym = HMM_Rotate_RH(state.graphics.ry, HMM_V3(0.0f, 1.0f, 0.0f));
    HMM_Mat4 rm = HMM_MulM4(rxm, rym);

    // render shapes...
    sg_begin_pass({ .action = state.graphics.pass_action, .swapchain = sglue_swapchain() });
    if (state.graphics.line_mode) {
        sg_apply_pipeline(state.graphics.pip_lines);
    } else {
        sg_apply_pipeline(state.graphics.pip_fill);
    }

    draw_jolt_box(state.physics.floor_id, state.physics.floor_xyz);
    draw_jolt_sphere(state.physics.sphere_id, state.physics.sphere_radius);

    sdtx_draw();
    sg_end_pass();
    sg_commit();
}

static void input(const sapp_event* ev) {
    if (ev->type == SAPP_EVENTTYPE_KEY_DOWN) {
        switch (ev->key_code) {
            case SAPP_KEYCODE_1: state.graphics.vs_params.draw_mode = 0.0f; break;
            case SAPP_KEYCODE_2: state.graphics.vs_params.draw_mode = 1.0f; break;
            case SAPP_KEYCODE_3: state.graphics.vs_params.draw_mode = 2.0f; break;
            case SAPP_KEYCODE_4: state.graphics.line_mode = !state.graphics.line_mode; break;
            case SAPP_KEYCODE_R: clear_physics_scene(); create_physics_scene(); break;
            default: break;
        }
    }
    cam_handle_event(&state.graphics.camera, ev);
}

sapp_desc sokol_main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    sapp_desc _sapp_desc{};
    _sapp_desc.init_cb = init;
    _sapp_desc.frame_cb = frame;
    _sapp_desc.cleanup_cb = cleanup;
    _sapp_desc.event_cb = input;
    _sapp_desc.width = 800;
    _sapp_desc.height = 600;
    _sapp_desc.sample_count = 4;
    _sapp_desc.window_title = "sokol-shape with jolt";
    _sapp_desc.icon.sokol_default = true;
    _sapp_desc.logger.func = slog_func;
    return _sapp_desc;
}

