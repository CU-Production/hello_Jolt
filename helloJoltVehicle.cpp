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
// jolt vehicle headers
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

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
		struct {
			JPH::Ref<JPH::VehicleCollisionTester> mTesters[3];
			JPH::Body*                            mCarBody;
			JPH::Ref<JPH::VehicleConstraint>      mVehicleConstraint;
			JPH::Vec3                             mExtents;

			struct {
				float  mForward = 0.0f;
				float  mPreviousForward = 1.0f; ///< Keeps track of last car direction so we know when to brake and when to accelerate
				float  mRight = 0.0f;
				float  mBrake = 0.0f;
				float  mHandBrake = 0.0f;
			} control;
		} vehicle;
		BoxDrawable floor = BoxDrawable(JPH::Vec3(200.0f, 0.2f, 200.0f));
		SphereDrawable spheres[100];
		BoxDrawable boxes[100];

		JPH::PhysicsSystem physics_system;

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
			_sshape_sphere_t.color = sphere.color;
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
			_sshape_cylinder_t.color = cylinder.color;
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
			_sshape_torus_t.color = torus.color;
			buf = sshape_build_torus(&buf, &_sshape_torus_t);
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
	state.graphics.vs_params.mvp = HMM_MulM4(view_proj, model);
	sg_apply_uniforms(UB_shapes_vs_params, SG_RANGE(state.graphics.vs_params));
	sg_draw(draw.base_element, draw.num_elements, 1);

	sg_destroy_buffer(state.graphics.vbuf);
	sg_destroy_buffer(state.graphics.ibuf);
}

static void draw_jolt_vehicle() {
	if (!state.physics.vehicle.mCarBody) return;

	// draw CarBody
	{
		BoxDrawable main_car_body{};
		main_car_body.extents = state.physics.vehicle.mExtents;
		main_car_body.id = state.physics.vehicle.mCarBody->GetID();

		draw_jolt_object(main_car_body);
	}


	// draw Wheels
	if (!state.physics.vehicle.mVehicleConstraint) return;

	for (JPH::uint w = 0; w < 4; ++w)
	{
		const JPH::WheelSettings *settings = state.physics.vehicle.mVehicleConstraint->GetWheels()[w]->GetSettings();
		JPH::RMat44 wheel_transform = state.physics.vehicle.mVehicleConstraint->GetWheelWorldTransform(w, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX()); // The cylinder we draw is aligned with Y so we specify that as rotational axis

		HMM_Mat4 proj = state.graphics.camera.proj;
		HMM_Mat4 view = state.graphics.camera.view;
		HMM_Mat4 view_proj = HMM_MulM4(proj, view);

		HMM_Mat4 model = jolt_mat4_to_hmm_mat4(wheel_transform);

		sshape_vertex_t vertices[6 * 1024];
		uint16_t indices[16 * 1024];

		sshape_buffer_t buf = {};
		buf.vertices.buffer = SSHAPE_RANGE(vertices);
		buf.indices.buffer  = SSHAPE_RANGE(indices);

		sshape_cylinder_t _sshape_cylinder_t{};
		_sshape_cylinder_t.radius = settings->mRadius;
		_sshape_cylinder_t.height = settings->mWidth;
		_sshape_cylinder_t.slices = 10;
		_sshape_cylinder_t.stacks = 2;
		// _sshape_cylinder_t.color = (128<<8);
		_sshape_cylinder_t.random_colors = true;
		buf = sshape_build_cylinder(&buf, &_sshape_cylinder_t);
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
}

static void vehicle_handle_event(const sapp_event* ev) {
	if (!state.physics.vehicle.mCarBody) return;

	// Determine acceleration and brake
	// state.physics.vehicle.control.mForward = 0.0f;
	if (ev->type == SAPP_EVENTTYPE_KEY_DOWN) {
		switch (ev->key_code) {
			case SAPP_KEYCODE_UP: state.physics.vehicle.control.mForward = 1.0f; break;
			case SAPP_KEYCODE_DOWN: state.physics.vehicle.control.mForward = -1.0f; break;
			default: break;
		}
	} else if (ev->type == SAPP_EVENTTYPE_KEY_UP) {
		switch (ev->key_code) {
			case SAPP_KEYCODE_UP: state.physics.vehicle.control.mForward = 0.0f; break;
			case SAPP_KEYCODE_DOWN: state.physics.vehicle.control.mForward = 0.0f; break;
			default: break;
		}
	}

	// Check if we're reversing direction
	state.physics.vehicle.control.mBrake = 0.0f;
	if (state.physics.vehicle.control.mPreviousForward * state.physics.vehicle.control.mForward < 0.0f) {
		// Get vehicle velocity in local space to the body of the vehicle
		float velocity = (state.physics.vehicle.mCarBody->GetRotation().Conjugated() * state.physics.vehicle.mCarBody->GetLinearVelocity()).GetZ();
		if ((state.physics.vehicle.control.mForward > 0.0f && velocity < -0.1f) || (state.physics.vehicle.control.mForward < 0.0f && velocity > 0.1f)) {
			// Brake while we've not stopped yet
			state.physics.vehicle.control.mForward = 0.0f;
			state.physics.vehicle.control.mBrake = 1.0f;
		} else {
			// When we've come to a stop, accept the new direction
			state.physics.vehicle.control.mPreviousForward = state.physics.vehicle.control.mForward;
		}
	}

	// Hand brake will cancel gas pedal
	// state.physics.vehicle.control.mHandBrake = 0.0f;
	if (ev->type == SAPP_EVENTTYPE_KEY_DOWN && ev->key_code == SAPP_KEYCODE_Z) {
		state.physics.vehicle.control.mForward = 0.0f;
		state.physics.vehicle.control.mHandBrake = 1.0f;
	} else if (ev->type == SAPP_EVENTTYPE_KEY_UP && ev->key_code == SAPP_KEYCODE_Z) {
		state.physics.vehicle.control.mHandBrake = 0.0f;
	}

	// Steering
	// state.physics.vehicle.control.mRight = 0.0f;
	if (ev->type == SAPP_EVENTTYPE_KEY_DOWN) {
		switch (ev->key_code) {
			case SAPP_KEYCODE_LEFT: state.physics.vehicle.control.mRight = -1.0f; break;
			case SAPP_KEYCODE_RIGHT: state.physics.vehicle.control.mRight = 1.0f; break;
			default: break;
		}
	} else if (ev->type == SAPP_EVENTTYPE_KEY_UP) {
		switch (ev->key_code) {
			case SAPP_KEYCODE_LEFT: state.physics.vehicle.control.mRight = 0.0f; break;
			case SAPP_KEYCODE_RIGHT: state.physics.vehicle.control.mRight = 0.0f; break;
			default: break;
		}
	}
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

    	JPH::BodyCreationSettings floor_settings(floor_shape, JPH::RVec3(0.0_r, -1.0_r, 0.0_r), JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
    	state.physics.floor.id = body_interface.CreateAndAddBody(floor_settings, JPH::EActivation::DontActivate);
    }

	// spheres
    {
    	float tmp_radius = 0.5f;
    	JPH::Ref<JPH::SphereShape> sphere_shape = new JPH::SphereShape(tmp_radius);
	    for (int i = 0; i < 100; i++) {
	    	state.physics.spheres[i].radius = tmp_radius;
	    	float posx = random_float();
	    	float posy = random_float() + 12.0f;
	    	JPH::BodyCreationSettings sphere_settings(sphere_shape, JPH::RVec3(posx, (5.0 + 5.0*i), posy), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
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
    		float posy = random_float() + 12.0f;
    		JPH::BodyCreationSettings box_settings(box_shape, JPH::RVec3(posx, (2.5 + 5.0*i), posy), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    		state.physics.boxes[i].id = body_interface.CreateAndAddBody(box_settings, JPH::EActivation::Activate);

    		uint8_t random_r = static_cast<uint8_t>(random_float() * 255.0f);
    		uint8_t random_g = static_cast<uint8_t>(random_float() * 255.0f);
    		uint8_t random_b = static_cast<uint8_t>(random_float() * 255.0f);
    		uint32_t random_vertex_color = random_r | random_g << 8 | random_b << 16;

    		state.physics.boxes[i].color = random_vertex_color;
    	}
	}

	// vehicle
    {
    	static float    sInitialRollAngle = 0;
    	static float    sMaxRollAngle = (60.0f) * HMM_DegToRad;
    	static float    sMaxSteeringAngle = (30.0f) * HMM_DegToRad;
    	static int      sCollisionMode = 2;
    	static bool	    sFourWheelDrive = false;
    	static bool	    sAntiRollbar = true;
    	static bool	    sLimitedSlipDifferentials = true;
    	static bool	    sOverrideGravity = false;					///< If true, gravity is overridden to always oppose the ground normal
    	static float    sMaxEngineTorque = 500.0f;
    	static float    sClutchStrength = 10.0f;
    	static float    sFrontCasterAngle = 0.0f;
    	static float    sFrontKingPinAngle = 0.0f;
    	static float    sFrontCamber = 0.0f;
    	static float    sFrontToe = 0.0f;
    	static float    sFrontSuspensionForwardAngle = 0.0f;
    	static float    sFrontSuspensionSidewaysAngle = 0.0f;
    	static float    sFrontSuspensionMinLength = 0.3f;
    	static float    sFrontSuspensionMaxLength = 0.5f;
    	static float    sFrontSuspensionFrequency = 1.5f;
    	static float    sFrontSuspensionDamping = 0.5f;
    	static float    sRearSuspensionForwardAngle = 0.0f;
    	static float    sRearSuspensionSidewaysAngle = 0.0f;
    	static float    sRearCasterAngle = 0.0f;
    	static float    sRearKingPinAngle = 0.0f;
    	static float    sRearCamber = 0.0f;
    	static float    sRearToe = 0.0f;
    	static float    sRearSuspensionMinLength = 0.3f;
    	static float    sRearSuspensionMaxLength = 0.5f;
    	static float    sRearSuspensionFrequency = 1.5f;
    	static float    sRearSuspensionDamping = 0.5f;

    	const float wheel_radius = 0.3f;
    	const float wheel_width = 0.1f;
    	const float half_vehicle_length = 2.0f;
    	const float half_vehicle_width = 0.9f;
    	const float half_vehicle_height = 0.2f;

    	// init vehicle
    	state.physics.vehicle.mExtents = JPH::Vec3(half_vehicle_width, half_vehicle_height, half_vehicle_length);
    	state.physics.vehicle.mExtents *= 2.0f;

    	// Create collision testers
    	state.physics.vehicle.mTesters[0] = new JPH::VehicleCollisionTesterRay(Layers::MOVING);
    	state.physics.vehicle.mTesters[1] = new JPH::VehicleCollisionTesterCastSphere(Layers::MOVING, 0.5f * wheel_width);
    	state.physics.vehicle.mTesters[2] = new JPH::VehicleCollisionTesterCastCylinder(Layers::MOVING);

    	// Create vehicle body
    	JPH::RVec3 position(0, 2, 0);
    	JPH::RefConst<JPH::Shape> car_shape = JPH::OffsetCenterOfMassShapeSettings(JPH::Vec3(0, -half_vehicle_height, 0), new JPH::BoxShape(JPH::Vec3(half_vehicle_width, half_vehicle_height, half_vehicle_length))).Create().Get();
    	JPH::BodyCreationSettings car_body_settings(car_shape, position, JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), sInitialRollAngle), JPH::EMotionType::Dynamic, Layers::MOVING);
    	car_body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    	car_body_settings.mMassPropertiesOverride.mMass = 1500.0f;
    	state.physics.vehicle.mCarBody = body_interface.CreateBody(car_body_settings);
    	body_interface.AddBody(state.physics.vehicle.mCarBody->GetID(), JPH::EActivation::Activate);

    	// Create vehicle constraint
    	JPH::VehicleConstraintSettings vehicle_constraint_settings;
    	vehicle_constraint_settings.mDrawConstraintSize = 0.1f;
    	vehicle_constraint_settings.mMaxPitchRollAngle = sMaxRollAngle;

    	// Suspension direction
    	JPH::Vec3 front_suspension_dir = JPH::Vec3(JPH::Tan(sFrontSuspensionSidewaysAngle), -1, JPH::Tan(sFrontSuspensionForwardAngle)).Normalized();
    	JPH::Vec3 front_steering_axis = JPH::Vec3(-JPH::Tan(sFrontKingPinAngle), 1, -JPH::Tan(sFrontCasterAngle)).Normalized();
    	JPH::Vec3 front_wheel_up = JPH::Vec3(JPH::Sin(sFrontCamber), JPH::Cos(sFrontCamber), 0);
    	JPH::Vec3 front_wheel_forward = JPH::Vec3(-JPH::Sin(sFrontToe), 0, JPH::Cos(sFrontToe));
    	JPH::Vec3 rear_suspension_dir = JPH::Vec3(JPH::Tan(sRearSuspensionSidewaysAngle), -1, JPH::Tan(sRearSuspensionForwardAngle)).Normalized();
    	JPH::Vec3 rear_steering_axis = JPH::Vec3(-JPH::Tan(sRearKingPinAngle), 1, -JPH::Tan(sRearCasterAngle)).Normalized();
    	JPH::Vec3 rear_wheel_up = JPH::Vec3(JPH::Sin(sRearCamber), JPH::Cos(sRearCamber), 0);
    	JPH::Vec3 rear_wheel_forward = JPH::Vec3(-JPH::Sin(sRearToe), 0, JPH::Cos(sRearToe));
    	JPH::Vec3 flip_x(-1, 1, 1);

    	// Wheels, left front
		JPH::WheelSettingsWV *w1 = new JPH::WheelSettingsWV;
		w1->mPosition = JPH::Vec3(half_vehicle_width, -0.9f * half_vehicle_height, half_vehicle_length - 2.0f * wheel_radius);
		w1->mSuspensionDirection = front_suspension_dir;
		w1->mSteeringAxis = front_steering_axis;
		w1->mWheelUp = front_wheel_up;
		w1->mWheelForward = front_wheel_forward;
    	w1->mSuspensionMinLength = sFrontSuspensionMinLength;
		w1->mSuspensionMaxLength = sFrontSuspensionMaxLength;
		w1->mSuspensionSpring.mFrequency = sFrontSuspensionFrequency;
		w1->mSuspensionSpring.mDamping = sFrontSuspensionDamping;
		w1->mMaxSteerAngle = sMaxSteeringAngle;
		w1->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

		// Right front
		JPH::WheelSettingsWV *w2 = new JPH::WheelSettingsWV;
		w2->mPosition = JPH::Vec3(-half_vehicle_width, -0.9f * half_vehicle_height, half_vehicle_length - 2.0f * wheel_radius);
		w2->mSuspensionDirection = flip_x * front_suspension_dir;
		w2->mSteeringAxis = flip_x * front_steering_axis;
		w2->mWheelUp = flip_x * front_wheel_up;
		w2->mWheelForward = flip_x * front_wheel_forward;
		w2->mSuspensionMinLength = sFrontSuspensionMinLength;
		w2->mSuspensionMaxLength = sFrontSuspensionMaxLength;
		w2->mSuspensionSpring.mFrequency = sFrontSuspensionFrequency;
		w2->mSuspensionSpring.mDamping = sFrontSuspensionDamping;
		w2->mMaxSteerAngle = sMaxSteeringAngle;
		w2->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

		// Left rear
		JPH::WheelSettingsWV *w3 = new JPH::WheelSettingsWV;
		w3->mPosition = JPH::Vec3(half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 2.0f * wheel_radius);
		w3->mSuspensionDirection = rear_suspension_dir;
		w3->mSteeringAxis = rear_steering_axis;
		w3->mWheelUp = rear_wheel_up;
		w3->mWheelForward = rear_wheel_forward;
		w3->mSuspensionMinLength = sRearSuspensionMinLength;
		w3->mSuspensionMaxLength = sRearSuspensionMaxLength;
		w3->mSuspensionSpring.mFrequency = sRearSuspensionFrequency;
		w3->mSuspensionSpring.mDamping = sRearSuspensionDamping;
		w3->mMaxSteerAngle = 0.0f;

		// Right rear
		JPH::WheelSettingsWV *w4 = new JPH::WheelSettingsWV;
		w4->mPosition = JPH::Vec3(-half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 2.0f * wheel_radius);
		w4->mSuspensionDirection = flip_x * rear_suspension_dir;
		w4->mSteeringAxis = flip_x * rear_steering_axis;
		w4->mWheelUp = flip_x * rear_wheel_up;
		w4->mWheelForward = flip_x * rear_wheel_forward;
		w4->mSuspensionMinLength = sRearSuspensionMinLength;
		w4->mSuspensionMaxLength = sRearSuspensionMaxLength;
		w4->mSuspensionSpring.mFrequency = sRearSuspensionFrequency;
		w4->mSuspensionSpring.mDamping = sRearSuspensionDamping;
		w4->mMaxSteerAngle = 0.0f;

    	vehicle_constraint_settings.mWheels = { w1, w2, w3, w4 };

    	for (JPH::WheelSettings *w : vehicle_constraint_settings.mWheels) {
    		w->mRadius = wheel_radius;
    		w->mWidth = wheel_width;
    	}

    	JPH::WheeledVehicleControllerSettings *controller_settings = new JPH::WheeledVehicleControllerSettings;
    	vehicle_constraint_settings.mController = controller_settings;

    	// Differential
    	controller_settings->mDifferentials.resize(sFourWheelDrive? 2 : 1);
    	controller_settings->mDifferentials[0].mLeftWheel = 0;
    	controller_settings->mDifferentials[0].mRightWheel = 1;
    	if (sFourWheelDrive)
    	{
    		controller_settings->mDifferentials[1].mLeftWheel = 2;
    		controller_settings->mDifferentials[1].mRightWheel = 3;

    		// Split engine torque
    		controller_settings->mDifferentials[0].mEngineTorqueRatio = controller_settings->mDifferentials[1].mEngineTorqueRatio = 0.5f;
    	}

    	// Anti rollbars
    	if (sAntiRollbar)
    	{
    		vehicle_constraint_settings.mAntiRollBars.resize(2);
    		vehicle_constraint_settings.mAntiRollBars[0].mLeftWheel = 0;
    		vehicle_constraint_settings.mAntiRollBars[0].mRightWheel = 1;
    		vehicle_constraint_settings.mAntiRollBars[1].mLeftWheel = 2;
    		vehicle_constraint_settings.mAntiRollBars[1].mRightWheel = 3;
    	}

    	state.physics.vehicle.mVehicleConstraint = new JPH::VehicleConstraint(*state.physics.vehicle.mCarBody, vehicle_constraint_settings);

    	// The vehicle settings were tweaked with a buggy implementation of the longitudinal tire impulses, this meant that PhysicsSettings::mNumVelocitySteps times more impulse
    	// could be applied than intended. To keep the behavior of the vehicle the same we increase the max longitudinal impulse by the same factor. In a future version the vehicle
    	// will be retweaked.
    	static_cast<JPH::WheeledVehicleController *>(state.physics.vehicle.mVehicleConstraint->GetController())->SetTireMaxImpulseCallback(
			[](JPH::uint, float &outLongitudinalImpulse, float &outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float, float, float)
			{
				outLongitudinalImpulse = 10.0f * inLongitudinalFriction * inSuspensionImpulse;
				outLateralImpulse = inLateralFriction * inSuspensionImpulse;
			});

    	state.physics.physics_system.AddConstraint(state.physics.vehicle.mVehicleConstraint);
    	state.physics.physics_system.AddStepListener(state.physics.vehicle.mVehicleConstraint);

    	// // Pass the input on to the constraint
    	// JPH::WheeledVehicleController *controller = static_cast<JPH::WheeledVehicleController *>(mVehicleConstraint->GetController());
    	// controller->SetDriverInput(mForward, mRight, mBrake, mHandBrake);

    	// Set the collision tester
    	state.physics.vehicle.mVehicleConstraint->SetVehicleCollisionTester(state.physics.vehicle.mTesters[sCollisionMode]);
    }
}

static void clear_physics_scene() {
    JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	// destroy vehicle
    {
    	state.physics.vehicle.mTesters[0] = nullptr;
    	state.physics.vehicle.mTesters[1] = nullptr;
    	state.physics.vehicle.mTesters[2] = nullptr;

    	state.physics.physics_system.RemoveConstraint(state.physics.vehicle.mVehicleConstraint);
    	state.physics.physics_system.RemoveStepListener(state.physics.vehicle.mVehicleConstraint);
    	state.physics.vehicle.mVehicleConstraint = nullptr;

    	body_interface.RemoveBody(state.physics.vehicle.mCarBody->GetID());
    	body_interface.DestroyBody(state.physics.vehicle.mCarBody->GetID());
    	state.physics.vehicle.mCarBody = nullptr;
    }

	// destroy scene
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
	    camdesc.pos = HMM_V3(0.0f, 5.5f, -25.0f);
	    camdesc.up = HMM_V3(0.0f, 1.0f, 0.0f);
	    camdesc.yaw = 90.0f;
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
	// Jolt Physics Update
	JPH::BodyInterface& body_interface = state.physics.physics_system.GetBodyInterface();

	// jolt vehicle physics update
	{
		static bool	    sLimitedSlipDifferentials = true;
		static float    sMaxEngineTorque = 500.0f;
		static float    sClutchStrength = 10.0f;

		// On user input, assure that the car is active
		if (state.physics.vehicle.control.mRight != 0.0f || state.physics.vehicle.control.mForward != 0.0f || state.physics.vehicle.control.mBrake != 0.0f || state.physics.vehicle.control.mHandBrake != 0.0f)
			body_interface.ActivateBody(state.physics.vehicle.mCarBody->GetID());

		JPH::WheeledVehicleController *controller = static_cast<JPH::WheeledVehicleController *>(state.physics.vehicle.mVehicleConstraint->GetController());

		// Update vehicle statistics
		controller->GetEngine().mMaxTorque = sMaxEngineTorque;
		controller->GetTransmission().mClutchStrength = sClutchStrength;

		// Set slip ratios to the same for everything
		float limited_slip_ratio = sLimitedSlipDifferentials? 1.4f : FLT_MAX;
		controller->SetDifferentialLimitedSlipRatio(limited_slip_ratio);
		for (JPH::VehicleDifferentialSettings &d : controller->GetDifferentials())
			d.mLimitedSlipRatio = limited_slip_ratio;

		// Pass the input on to the constraint
		controller->SetDriverInput(state.physics.vehicle.control.mForward, state.physics.vehicle.control.mRight, state.physics.vehicle.control.mBrake, state.physics.vehicle.control.mHandBrake);

		// // Set the collision tester
		// state.physics.vehicle.mVehicleConstraint->SetVehicleCollisionTester(state.physics.vehicle.mTesters[sCollisionMode]);
	}

	// physics system update
    const int cCollisionSteps = 2;
    state.physics.physics_system.Update((float)sapp_frame_duration(), cCollisionSteps, state.physics.temp_allocator, state.physics.job_system);

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
              "  1: vertex normals\n"
              "  2: texture coords\n"
              "  3: vertex color\n"
              "  4: line/fill mode\n"
              "  R: reset physics\n");

    // render shapes...
    sg_begin_pass({ .action = state.graphics.pass_action, .swapchain = sglue_swapchain() });
    if (state.graphics.line_mode) {
        sg_apply_pipeline(state.graphics.pip_lines);
    } else {
        sg_apply_pipeline(state.graphics.pip_fill);
    }

	// draw main scene
	draw_jolt_object(state.physics.floor);
	for (int i = 0; i < 100; i++) {
		draw_jolt_object(state.physics.spheres[i]);
		draw_jolt_object(state.physics.boxes[i]);
	}

	// draw vehicle
	draw_jolt_vehicle();

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
	vehicle_handle_event(ev);
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

