//------------------------------------------------------------------------------
//  sokol_shape with JoltPhysics demo.
//------------------------------------------------------------------------------
#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Math/Math.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
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
#include <random>

#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/SoftBody/SoftBodyShape.h"


#define DEMO_ENABLE_EVENT_LOGS 0

inline float random_float()
{
	// Returns a random real in [0,1).
	static std::uniform_real_distribution<float> distribution(0.0, 1.0);
	static std::mt19937 generator;
	return distribution(generator);
}

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
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "Contact validate callback" << std::endl;
#endif

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "A contact was added" << std::endl;
#endif
	}

	virtual void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "A contact was persisted" << std::endl;
#endif
	}

	virtual void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
	{
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "A contact was removed" << std::endl;
#endif
	}
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
	virtual void OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "A body got activated" << std::endl;
#endif
	}

	virtual void OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
#if DEMO_ENABLE_EVENT_LOGS
		std::cout << "A body went to sleep" << std::endl;
#endif
	}
};

class Drawable {
public:
	enum DrawAbleTypeEnum {
		BOX = 0,
		PLANE,
		SPHERE,
		CYLINDER,
		CAPSULE,
		TORUS,
		NUM_SHAPES
	};

	DrawAbleTypeEnum type = NUM_SHAPES;
	JPH::BodyID id;
	uint32_t color;
};

class BoxDrawable : public Drawable {
public:
	BoxDrawable(): extents(JPH::Vec3(1.0f,1.0f,1.0f)) { type = BOX; }
	BoxDrawable(JPH::Vec3 inExtents): extents(inExtents) { type = BOX; }
	JPH::Vec3 extents;
};

class PlaneDrawable : public Drawable {
public:
	PlaneDrawable(): width(0), depth(0) { type = PLANE; }
	PlaneDrawable(float inWidth, float inDepth): width(inWidth), depth(inDepth) { type = PLANE; }
	float width;
	float depth;
};

class SphereDrawable : public Drawable {
public:
	SphereDrawable(): radius(0) { type = SPHERE; }
	SphereDrawable(float inRadius): radius(inRadius) { type = SPHERE; }
	float radius;
};

class CylinderDrawable : public Drawable {
public:
	CylinderDrawable(): radius(0), height(0) { type = CYLINDER; }
	CylinderDrawable(float inRadius, float inHeight): radius(inRadius), height(inHeight) { type = CYLINDER; }
	float radius;
	float height;
};

class CapsuleDrawable : public Drawable {
public:
	CapsuleDrawable(): radius(0), height(0) { type = CAPSULE; }
	CapsuleDrawable(float inRadius, float inHeight): radius(inRadius), height(inHeight) { type = CYLINDER; }
	float radius;
	float height;
};

class TorusDrawable : public Drawable {
public:
	TorusDrawable() : radius(0), ring_radius(0) { type = TORUS; }
	TorusDrawable(float inRadidu, float inRingRadius) : radius(inRadidu), ring_radius(inRingRadius) { type = TORUS; }
	float radius;
	float ring_radius;
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

		BoxDrawable floor = BoxDrawable(JPH::Vec3(200.0f, 0.2f, 200.0f));
		SphereDrawable spheres[100];
		BoxDrawable boxes[100];

		struct {
			JPH::BodyID id;
			BoxDrawable clothBox;
		} cloth;

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

static void draw_jolt_object(const Drawable& drawable) {
	if (drawable.id.IsInvalid()) return;

	JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	JPH::Mat44 world = body_interface.GetWorldTransform(drawable.id);

	HMM_Mat4 proj = state.graphics.camera.proj;
	HMM_Mat4 view = state.graphics.camera.view;
	HMM_Mat4 view_proj = HMM_MulM4(proj, view);

	HMM_Mat4 model = jolt_mat4_to_hmm_mat4(world);

	sshape_vertex_t vertices[6 * 1024];
	uint16_t indices[16 * 1024];

	sshape_buffer_t buf = {};
	buf.vertices.buffer = SSHAPE_RANGE(vertices);
	buf.indices.buffer  = SSHAPE_RANGE(indices);

	sshape_element_range_t draw;

	switch (drawable.type) {
		case Drawable::BOX : {
			const BoxDrawable& box = static_cast<const BoxDrawable&>(drawable);
			sshape_box_t _sshape_box_t{};
			_sshape_box_t.width  = box.extents.mF32[0];
			_sshape_box_t.height = box.extents.mF32[1];
			_sshape_box_t.depth  = box.extents.mF32[2];
			_sshape_box_t.tiles  = 10;
			_sshape_box_t.color  = box.color;
			buf = sshape_build_box(&buf, &_sshape_box_t);
			draw = sshape_element_range(&buf);
			break;
		}
		case Drawable::PLANE: {
			const PlaneDrawable& plane = static_cast<const PlaneDrawable&>(drawable);
			sshape_plane_t _sshape_plane_t{};
			_sshape_plane_t.width  = plane.width;
			_sshape_plane_t.depth  = plane.depth;
			_sshape_plane_t.tiles  = 10;
			_sshape_plane_t.color  = plane.color;
			buf = sshape_build_plane(&buf, &_sshape_plane_t);
			draw = sshape_element_range(&buf);
			break;
		}
		case Drawable::SPHERE: {
			const SphereDrawable& sphere = static_cast<const SphereDrawable&>(drawable);
			sshape_sphere_t _sshape_sphere_t{};
			_sshape_sphere_t.radius = sphere.radius;
			_sshape_sphere_t.slices = 36;
			_sshape_sphere_t.stacks = 20;
			_sshape_sphere_t.color  = sphere.color;
			buf = sshape_build_sphere(&buf, &_sshape_sphere_t);
			draw = sshape_element_range(&buf);
			break;
		}
		case Drawable::CYLINDER: {
			const CylinderDrawable& cylinder = static_cast<const CylinderDrawable&>(drawable);
			sshape_cylinder_t _sshape_cylinder_t{};
			_sshape_cylinder_t.radius = cylinder.radius;
			_sshape_cylinder_t.height = cylinder.height;
			_sshape_cylinder_t.slices = 36;
			_sshape_cylinder_t.stacks = 20;
			_sshape_cylinder_t.color  = cylinder.color;
			buf = sshape_build_cylinder(&buf, &_sshape_cylinder_t);
			draw = sshape_element_range(&buf);
			break;
		}
		case Drawable::TORUS: {
			const TorusDrawable& torus = static_cast<const TorusDrawable&>(drawable);
			sshape_torus_t _sshape_torus_t{};
			_sshape_torus_t.radius = torus.radius;
			_sshape_torus_t.ring_radius = torus.ring_radius;
			_sshape_torus_t.rings = 36;
			_sshape_torus_t.sides = 18;
			_sshape_torus_t.color  = torus.color;
			buf = sshape_build_torus(&buf, &_sshape_torus_t);
			draw = sshape_element_range(&buf);
			break;
		}
		case Drawable::CAPSULE: {
			const CapsuleDrawable& capsule = static_cast<const CapsuleDrawable&>(drawable);
			// capsule = 1 cylinder + 2 spheres
			sshape_cylinder_t _sshape_cylinder_t{};
			_sshape_cylinder_t.radius = capsule.radius;
			_sshape_cylinder_t.height = capsule.height;
			_sshape_cylinder_t.slices = 36;
			_sshape_cylinder_t.stacks = 20;
			_sshape_cylinder_t.color  = capsule.color;
			_sshape_cylinder_t.merge  = false;
			buf = sshape_build_cylinder(&buf, &_sshape_cylinder_t);
			sshape_sphere_t _sshape_sphere_t{};
			_sshape_sphere_t.radius = capsule.radius;
			_sshape_sphere_t.slices = 36;
			_sshape_sphere_t.stacks = 20;
			_sshape_sphere_t.color  = capsule.color;
			_sshape_sphere_t.merge  = true;
			_sshape_sphere_t.transform = _sshape_mat4_identity();
			_sshape_sphere_t.transform.m[3][1] = capsule.height * 0.5f;
			buf = sshape_build_sphere(&buf, &_sshape_sphere_t);
			_sshape_sphere_t.transform.m[3][1] = -capsule.height * 0.5f;
			buf = sshape_build_sphere(&buf, &_sshape_sphere_t);
			draw = sshape_element_range(&buf);
			break;
		}
		default:
			assert(false, "drawable.type invalid");
			break;
	}

	const sg_buffer_desc vbuf_desc = sshape_vertex_buffer_desc(&buf);
	const sg_buffer_desc ibuf_desc = sshape_index_buffer_desc(&buf);
	state.graphics.vbuf = sg_make_buffer(&vbuf_desc);
	state.graphics.ibuf = sg_make_buffer(&ibuf_desc);

	sg_bindings _sg_bindings{};
	_sg_bindings.vertex_buffers[0] = state.graphics.vbuf;
	_sg_bindings.index_buffer = state.graphics.ibuf;
	sg_apply_bindings(_sg_bindings);
	state.graphics.vs_params.m = model;
	state.graphics.vs_params.vp = view_proj;
	sg_apply_uniforms(UB_shapes_vs_params, SG_RANGE(state.graphics.vs_params));
	sg_draw(draw.base_element, draw.num_elements, 1);

	sg_destroy_buffer(state.graphics.vbuf);
	sg_destroy_buffer(state.graphics.ibuf);
}

static void draw_jolt_cloth(const JPH::BodyID& inId) {
	if (inId.IsInvalid()) return;

	JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	JPH::Mat44 world = body_interface.GetWorldTransform(inId);

	JPH::BodyLockRead read_lock(state.physics.physics_system.GetBodyLockInterface(), inId);
	if (!read_lock.Succeeded()) return;

	const JPH::Body& body = read_lock.GetBody();

	if (!body.IsSoftBody()) return;

	const JPH::SoftBodyMotionProperties *soft_mp = static_cast<const JPH::SoftBodyMotionProperties *>(body.GetMotionProperties());
	const JPH::SoftBodyShape *soft_shape = static_cast<const JPH::SoftBodyShape *>(body.GetShape());


	HMM_Mat4 proj = state.graphics.camera.proj;
	HMM_Mat4 view = state.graphics.camera.view;
	HMM_Mat4 view_proj = HMM_MulM4(proj, view);

	HMM_Mat4 model = jolt_mat4_to_hmm_mat4(world);

	sshape_vertex_t vertices[6 * 1024];
	uint16_t indices[16 * 1024];

	sshape_buffer_t buf = {};
	buf.vertices.buffer = SSHAPE_RANGE(vertices);
	buf.indices.buffer  = SSHAPE_RANGE(indices);

	sshape_element_range_t draw;

	const auto& soft_vertices = soft_mp->GetVertices();
	for (int i = 0; i < soft_mp->GetVertices().size(); i++) {
		vertices[i].x = soft_vertices[i].mPosition.GetX();
		vertices[i].y = soft_vertices[i].mPosition.GetY();
		vertices[i].z = soft_vertices[i].mPosition.GetZ();
		vertices[i].normal = _sshape_pack_f4_byte4n(0.0f, 1.0f, 0.0f, 0.0f);
		vertices[i].color = _sshape_pack_f4_byte4n(0.1f, 0.6f, 0.3f, 1.0f);
	}
	buf.vertices.shape_offset = 0;
	buf.vertices.data_size = sizeof(sshape_vertex_t) * soft_vertices.size();

	const auto& soft_faces = soft_mp->GetFaces();
	for (int i = 0; i < soft_faces.size(); i++) {
		indices[i*3+0] = soft_faces[i].mVertex[0];
		indices[i*3+1] = soft_faces[i].mVertex[1];
		indices[i*3+2] = soft_faces[i].mVertex[2];
	}
	buf.indices.shape_offset = 0;
	buf.indices.data_size = sizeof(uint16_t) * soft_faces.size() * 3;

	buf.valid = true;

	draw = sshape_element_range(&buf);

	const sg_buffer_desc vbuf_desc = sshape_vertex_buffer_desc(&buf);
	const sg_buffer_desc ibuf_desc = sshape_index_buffer_desc(&buf);
	state.graphics.vbuf = sg_make_buffer(&vbuf_desc);
	state.graphics.ibuf = sg_make_buffer(&ibuf_desc);

	sg_bindings _sg_bindings{};
	_sg_bindings.vertex_buffers[0] = state.graphics.vbuf;
	_sg_bindings.index_buffer = state.graphics.ibuf;
	sg_apply_bindings(_sg_bindings);
	state.graphics.vs_params.m = model;
	state.graphics.vs_params.vp = view_proj;
	sg_apply_uniforms(UB_shapes_vs_params, SG_RANGE(state.graphics.vs_params));
	sg_draw(draw.base_element, draw.num_elements, 1);

	sg_destroy_buffer(state.graphics.vbuf);
	sg_destroy_buffer(state.graphics.ibuf);
}

static void create_physics_scene() {
    using namespace JPH::literals;

	state.physics.physics_system.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	// floor
    {
    	JPH::Vec3 half_extents = state.physics.floor.extents * 0.5f;
    	JPH::BoxShapeSettings floor_shape_settings(half_extents);
    	floor_shape_settings.SetEmbedded();

    	JPH::ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
    	JPH::ShapeRefC floor_shape = floor_shape_result.Get();

    	JPH::BodyCreationSettings floor_settings(floor_shape, JPH::RVec3(0.0_r, -5.0_r, 0.0_r), JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
    	state.physics.floor.id = body_interface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);
    }

	// spheres
    {
    	float tmp_radius = 0.5f;
    	JPH::Ref<JPH::SphereShape> sphere_shape = new JPH::SphereShape(tmp_radius);
	    for (int i = 0; i < 100; i++) {
	    	state.physics.spheres[i].radius = tmp_radius;
	    	float posx = random_float();
	    	float posy = random_float() - 12.0f;
	    	JPH::BodyCreationSettings sphere_settings(sphere_shape, JPH::RVec3(posx, (10.0 + 5.0*i), posy), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
	    	sphere_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
	    	sphere_settings.mMassPropertiesOverride.mMass = 5.0f;
	    	sphere_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
	    	state.physics.spheres[i].id = body_interface.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

	    	uint8_t random_r = static_cast<uint8_t>(random_float() * 255.0f);
	    	uint8_t random_g = static_cast<uint8_t>(random_float() * 255.0f);
	    	uint8_t random_b = static_cast<uint8_t>(random_float() * 255.0f);
	    	uint32_t random_vertex_color = random_r | random_g << 8 | random_b << 16;

	    	state.physics.spheres[i].color = random_vertex_color;
	    }
    }

	// boxes
	{
    	JPH::Vec3 extents = JPH::Vec3(1.0f, 1.0f, 1.0f);
    	JPH::Vec3 half_extents = extents * 0.5f;

    	JPH::BoxShapeSettings box_shape_settings(half_extents);
    	box_shape_settings.SetEmbedded();

    	JPH::ShapeSettings::ShapeResult box_shape_result = box_shape_settings.Create();
    	JPH::ShapeRefC box_shape = box_shape_result.Get();

    	for (int i = 0; i < 100; i++) {
    		state.physics.boxes[i].extents = extents;
    		float posx = random_float();
    		float posy = random_float() - 12.0f;
    		JPH::BodyCreationSettings box_settings(box_shape, JPH::RVec3(posx, (7.5 + 5.0*i), posy), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    		box_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    		box_settings.mMassPropertiesOverride.mMass = 5.0f;
    		box_settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    		state.physics.boxes[i].id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    		uint8_t random_r = static_cast<uint8_t>(random_float() * 255.0f);
    		uint8_t random_g = static_cast<uint8_t>(random_float() * 255.0f);
    		uint8_t random_b = static_cast<uint8_t>(random_float() * 255.0f);
    		uint32_t random_vertex_color = random_r | random_g << 8 | random_b << 16;

    		state.physics.boxes[i].color = random_vertex_color;
    	}
	}

	// soft bofy (cloth)
    {
    	JPH::Ref<JPH::SoftBodySharedSettings> settings  = new JPH::SoftBodySharedSettings;

    	const JPH::uint inGridSizeX = 30;
    	const JPH::uint inGridSizeZ = 30;
    	const float inGridSpacing = 0.75f;

    	// auto inVertexGetInvMass = [](JPH::uint, JPH::uint) { return 1.0f; };
		auto inVertexPerturbation = [](JPH::uint, JPH::uint) { return JPH::Vec3::sZero(); };

    	auto inVertexGetInvMass = [inGridSizeX, inGridSizeZ](JPH::uint inX, JPH::uint inZ) {
    		return (inX == 0 && inZ == 0)
				|| (inX == inGridSizeX - 1 && inZ == 0)
				|| (inX == 0 && inZ == inGridSizeZ - 1)
				|| (inX == inGridSizeX - 1 && inZ == inGridSizeZ - 1)? 0.0f : 1.0f;
    	};

    	const float cOffsetX = -0.5f * inGridSpacing * (inGridSizeX - 1);
    	const float cOffsetZ = -0.5f * inGridSpacing * (inGridSizeZ - 1);

    	// Create settings
    	for (JPH::uint z = 0; z < inGridSizeZ; ++z)
    		for (JPH::uint x = 0; x < inGridSizeX; ++x)
    		{
    			JPH::SoftBodySharedSettings::Vertex v;
    			JPH::Vec3 position = inVertexPerturbation(x, z) + JPH::Vec3(cOffsetX + x * inGridSpacing, 0.0f, cOffsetZ + z * inGridSpacing);
    			position.StoreFloat3(&v.mPosition);
    			v.mInvMass = inVertexGetInvMass(x, z);
    			settings->mVertices.push_back(v);
    		}

    	// Function to get the vertex index of a point on the cloth
    	auto vertex_index = [inGridSizeX](JPH::uint inX, JPH::uint inY) -> JPH::uint
    	{
    		return inX + inY * inGridSizeX;
    	};

    	// Create faces
    	for (JPH::uint z = 0; z < inGridSizeZ - 1; ++z)
    		for (JPH::uint x = 0; x < inGridSizeX - 1; ++x)
    		{
    			JPH::SoftBodySharedSettings::Face f;
    			f.mVertex[0] = vertex_index(x, z);
    			f.mVertex[1] = vertex_index(x, z + 1);
    			f.mVertex[2] = vertex_index(x + 1, z + 1);
    			settings->AddFace(f);

    			f.mVertex[1] = vertex_index(x + 1, z + 1);
    			f.mVertex[2] = vertex_index(x + 1, z);
    			settings->AddFace(f);
    		}

    	// Create constraints
    	const JPH::SoftBodySharedSettings::VertexAttributes &inVertexAttributes = { 1.0e-5f, 1.0e-5f, 1.0e-5f };
    	settings->CreateConstraints(&inVertexAttributes, 1, JPH::SoftBodySharedSettings::EBendType::None);

    	// Optimize the settings
    	settings->Optimize();

    	// Create Cloth
    	JPH::SoftBodyCreationSettings cloth(settings, JPH::RVec3(0, 5.0f, -12.0f), JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.25f * JPH::JPH_PI), Layers::MOVING);
    	cloth.mUpdatePosition = false; // Don't update the position of the cloth as it is fixed to the world
    	cloth.mMakeRotationIdentity = false; // Test explicitly checks if soft bodies with a rotation collide with shapes properly
    	state.physics.cloth.id = body_interface.CreateAndAddSoftBody(cloth, JPH::EActivation::Activate);

    	state.physics.cloth.clothBox.id = state.physics.cloth.id;
    	state.physics.cloth.clothBox.extents = JPH::RVec3(30, 0.5, 30);

    	uint8_t random_r = static_cast<uint8_t>(random_float() * 255.0f);
    	uint8_t random_g = static_cast<uint8_t>(random_float() * 255.0f);
    	uint8_t random_b = static_cast<uint8_t>(random_float() * 255.0f);
    	uint32_t random_vertex_color = random_r | random_g << 8 | random_b << 16;

    	state.physics.cloth.clothBox.color = random_vertex_color;
    }
}

static void clear_physics_scene() {
    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	body_interface.RemoveBody(state.physics.cloth.id);
	body_interface.DestroyBody(state.physics.cloth.id);

	for (int i = 0; i < 100; i++) {
		if (state.physics.boxes[i].id.IsInvalid()) continue;
		body_interface.RemoveBody(state.physics.boxes[i].id);
		body_interface.DestroyBody(state.physics.boxes[i].id);
	}
	for (int i = 0; i < 100; i++) {
		if (state.physics.spheres[i].id.IsInvalid()) continue;
		body_interface.RemoveBody(state.physics.spheres[i].id);
		body_interface.DestroyBody(state.physics.spheres[i].id);
	}
    body_interface.RemoveBody(state.physics.floor.id);
    body_interface.DestroyBody(state.physics.floor.id);
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
	    camdesc.farz = 1000.0f;
	    camdesc.pos = HMM_V3(0.0f, 5.5f, 6.0f);
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

		state.physics.temp_allocator = new JPH::TempAllocatorImpl(32*1024*1024);
		state.physics.job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

		const JPH::uint cMaxBodies = 10240;
		const JPH::uint cNumBodyMutexes = 0;
		const JPH::uint cMaxBodyPairs = 65536;
		const JPH::uint cMaxContactConstraints = 20480;

        state.physics.broad_phase_layer_interface = new BPLayerInterfaceImpl();
        state.physics.object_vs_broadphase_layer_filter = new ObjectVsBroadPhaseLayerFilterImpl();
        state.physics.object_vs_object_layer_filter = new ObjectLayerPairFilterImpl();

		JPH::PhysicsSettings physicsSettings;
		physicsSettings.mUseLargeIslandSplitter = true;
		state.physics.physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, *state.physics.broad_phase_layer_interface, *state.physics.object_vs_broadphase_layer_filter, *state.physics.object_vs_object_layer_filter);
		state.physics.physics_system.SetPhysicsSettings(physicsSettings);

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
	// physics update
    const int cCollisionSteps = 4;
    state.physics.physics_system.Update((float)sapp_frame_duration(), cCollisionSteps, state.physics.temp_allocator, state.physics.job_system);

	JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();
	for (int i = 0; i < 100; i++) {
		if (state.physics.spheres[i].id.IsInvalid()) continue;
		JPH::RVec3 position = body_interface.GetCenterOfMassPosition(state.physics.spheres[i].id);
		if (position.GetY() < -10.0f) {
			body_interface.RemoveBody(state.physics.spheres[i].id);
			body_interface.DestroyBody(state.physics.spheres[i].id);
			state.physics.spheres[i].id = JPH::BodyID();
		}
	}
	for (int i = 0; i < 100; i++) {
		if (state.physics.boxes[i].id.IsInvalid()) continue;
		JPH::RVec3 position = body_interface.GetCenterOfMassPosition(state.physics.boxes[i].id);
		if (position.GetY() < -10.0f) {
			body_interface.RemoveBody(state.physics.boxes[i].id);
			body_interface.DestroyBody(state.physics.boxes[i].id);
			state.physics.boxes[i].id = JPH::BodyID();
		}
	}


	// graphics update
    const int fb_width = sapp_width();
    const int fb_height = sapp_height();
    cam_update(&state.graphics.camera, fb_width, fb_height);

    // help text
    sdtx_canvas(fb_width*0.5f, fb_height*0.5f);
    sdtx_pos(0.5f, 0.5f);
    sdtx_puts("press key to switch draw mode:\n\n"
              "  1: basic lighting\n"
              "  2: vertex normals\n"
              "  3: texture coords\n"
              "  4: vertex color\n"
              "  5: line/fill mode\n"
              "  R: reset physics\n");

    // render shapes...
    sg_begin_pass({ .action = state.graphics.pass_action, .swapchain = sglue_swapchain() });
    if (state.graphics.line_mode) {
        sg_apply_pipeline(state.graphics.pip_lines);
    } else {
        sg_apply_pipeline(state.graphics.pip_fill);
    }

	draw_jolt_object(state.physics.floor);
	for (int i = 0; i < 100; i++) {
		draw_jolt_object(state.physics.spheres[i]);
		draw_jolt_object(state.physics.boxes[i]);
	}

	// draw cloth
    {
    	// draw_jolt_object(state.physics.cloth.clothBox);
    	draw_jolt_cloth(state.physics.cloth.id);
    }

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
            case SAPP_KEYCODE_4: state.graphics.vs_params.draw_mode = 3.0f; break;
            case SAPP_KEYCODE_5: state.graphics.line_mode = !state.graphics.line_mode; break;
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

