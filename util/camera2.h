#pragma once
#include <string.h>
#include <math.h>
#include <assert.h>

enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const float YAW         = -90.0f;
const float PITCH       =  0.0f;
const float SPEED       =  10.0f;
const float SENSITIVITY =  0.1f;
const float ZOOM        =  45.0f;

typedef struct {
    HMM_Vec3 pos;
    HMM_Vec3 up;
    float yaw;
    float pitch;
    float nearz;
    float farz;
} camera2_desc_t;

typedef struct {
    // camera Attributes
    HMM_Vec3 Position;
    HMM_Vec3 Front;
    HMM_Vec3 Up;
    HMM_Vec3 Right;
    HMM_Vec3 WorldUp;
    // euler Angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    // clip Z
    float nearz;
    float farz;

    HMM_Mat4 view;
    HMM_Mat4 proj;
    HMM_Mat4 view_proj;
} camera2_t;

static float _cam_def(float val, float def) {
    return ((val == 0.0f) ? def:val);
}

// calculates the front vector from the Camera's (updated) Euler Angles
static void cam_update_vectors(camera2_t* cam)
{
    assert(cam);
    // calculate the new Front vector
    HMM_Vec3 front;
    front.X = cos((cam->Yaw*HMM_DegToRad)) * cos((cam->Pitch*HMM_DegToRad));
    front.Y = sin((cam->Pitch*HMM_DegToRad));
    front.Z = sin((cam->Yaw*HMM_DegToRad)) * cos((cam->Pitch*HMM_DegToRad));
    cam->Front = HMM_NormV3(front);
    // also re-calculate the Right and Up vector
    cam->Right = HMM_NormV3(HMM_Cross(cam->Front, cam->WorldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    cam->Up    = HMM_NormV3(HMM_Cross(cam->Right, cam->Front));
}

/* initialize to default parameters */
static void cam_init(camera2_t* cam, const camera2_desc_t* desc) {
    assert(cam && desc);
    memset(cam, 0, sizeof(camera2_t));
    cam->Position = desc->pos;
    cam->WorldUp = desc->up;
    cam->Yaw = desc->yaw;
    cam->Pitch = desc->pitch;
    cam->nearz = desc->nearz;
    cam->farz = desc->farz;
    cam->MovementSpeed = SPEED;
    cam->MouseSensitivity= SENSITIVITY;
    cam->Zoom = ZOOM;
    cam_update_vectors(cam);
}

static void cam_mouse_movement(camera2_t* cam, float xoffset, float yoffset, bool constrainPitch) {
    assert(cam);
    xoffset *= cam->MouseSensitivity;
    yoffset *= cam->MouseSensitivity;

    cam->Yaw   += xoffset;
    cam->Pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainPitch)
    {
        if (cam->Pitch > 89.0f)
            cam->Pitch = 89.0f;
        if (cam->Pitch < -89.0f)
            cam->Pitch = -89.0f;
    }

    // update Front, Right and Up Vectors using the updated Euler angles
    cam_update_vectors(cam);
}

static void cam_mouse_scroll(camera2_t* cam, float yoffset) {
    assert(cam);
    cam->Zoom -= yoffset;
    if (cam->Zoom < 1.0f)
        cam->Zoom = 1.0f;
    if (cam->Zoom > 45.0f)
        cam->Zoom = 45.0f;
}

static void cam_keyboard(camera2_t* cam, enum Camera_Movement direction, float deltaTime)
{
    assert(cam);
    float velocity = cam->MovementSpeed * deltaTime;
    if (direction == FORWARD)
        cam->Position = HMM_AddV3(cam->Position, HMM_MulV3F(cam->Front, velocity));
    if (direction == BACKWARD)
        cam->Position = HMM_SubV3(cam->Position, HMM_MulV3F(cam->Front, velocity));
    if (direction == LEFT)
        cam->Position = HMM_SubV3(cam->Position, HMM_MulV3F(cam->Right, velocity));
    if (direction == RIGHT)
        cam->Position = HMM_AddV3(cam->Position, HMM_MulV3F(cam->Right , velocity));
}

/* update the view, proj and view_proj matrix */
static void cam_update(camera2_t* cam, int fb_width, int fb_height) {
    assert(cam);
    assert((fb_width > 0) && (fb_height > 0));
    const float w = (float) fb_width;
    const float h = (float) fb_height;
    cam->view = HMM_LookAt_RH(cam->Position, HMM_AddV3(cam->Position, cam->Front), cam->Up);
    cam->proj = HMM_Perspective_RH_ZO(cam->Zoom*HMM_DegToRad, w/h, cam->nearz, cam->farz);
    cam->view_proj = HMM_MulM4(cam->proj, cam->view);
}

/* handle sokol-app input events */
static void cam_handle_event(camera2_t* cam, const sapp_event* ev) {
    assert(cam);
    const float camera_speed = 0.1f;
    const float dt = sapp_frame_duration();
    switch (ev->type) {
        case SAPP_EVENTTYPE_MOUSE_DOWN:
            if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT) {
                sapp_lock_mouse(true);
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_UP:
            if (ev->mouse_button == SAPP_MOUSEBUTTON_LEFT) {
                sapp_lock_mouse(false);
            }
            break;
        case SAPP_EVENTTYPE_MOUSE_SCROLL:
            cam_mouse_scroll(cam, ev->scroll_y);
            break;
        case SAPP_EVENTTYPE_MOUSE_MOVE:
            if (sapp_mouse_locked()) {
                cam_mouse_movement(cam, ev->mouse_dx, ev->mouse_dy, true);
            }
            break;
        case SAPP_EVENTTYPE_KEY_DOWN:
            if (ev->key_code == SAPP_KEYCODE_W) {
                cam_keyboard(cam, FORWARD, dt);
            }
            if (ev->key_code == SAPP_KEYCODE_S) {
                cam_keyboard(cam, BACKWARD, dt);
            }
            if (ev->key_code == SAPP_KEYCODE_A) {
                cam_keyboard(cam, LEFT, dt);
            }
            if (ev->key_code == SAPP_KEYCODE_D) {
                cam_keyboard(cam, RIGHT, dt);
            }
            break;
        default:
            break;
    }
}
