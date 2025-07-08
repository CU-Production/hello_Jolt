//------------------------------------------------------------------------------
//  Simple sokol_shape.h demo.
//------------------------------------------------------------------------------
#define SOKOL_IMPL
// #define SOKOL_GLCORE
#define SOKOL_D3D11
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_log.h"
#include "sokol_glue.h"
#include "util/sokol_shape.h"
#include "util/sokol_debugtext.h"
#include "HandmadeMath.h"
#include "util/camera.h"
#include "shaders/shapes.glsl.h"

typedef struct {
    HMM_Vec3 pos;
    sshape_element_range_t draw;
} shape_t;

enum {
    BOX = 0,
    PLANE,
    SPHERE,
    CYLINDER,
    TORUS,
    NUM_SHAPES
};

static struct {
    sg_pass_action pass_action;
    sg_pipeline pip_fill;
    sg_pipeline pip_lines;
    sg_buffer vbuf;
    sg_buffer ibuf;
    shape_t shapes[NUM_SHAPES];
    shapes_vs_params_t vs_params;
    bool line_mode;
    float rx, ry;
    camera_t camera;
} state;

static void init(void) {
    sg_setup(&(sg_desc){
        .environment = sglue_environment(),
        .logger.func = slog_func,
    });
    sdtx_setup(&(sdtx_desc_t) {
        .fonts[0] = sdtx_font_oric(),
        .logger.func = slog_func,
    });

    state.line_mode = false;

    // clear to black
    state.pass_action = (sg_pass_action) {
        .colors[0] = { .load_action = SG_LOADACTION_CLEAR, .clear_value = { 0.0f, 0.0f, 0.0f, 1.0f } }
    };

    // initialize camera helper
    camera_desc_t camdesc = {0};
    camdesc.min_dist = 1.0f;
    camdesc.max_dist = 100.0f;
    camdesc.center.Y = 1.0f;
    camdesc.distance = 6.0f;
    camdesc.latitude = 10.0f;
    camdesc.longitude = 20.0f;
    camdesc.aspect = 70.0f * HMM_DegToRad;
    cam_init(&state.camera, &camdesc);

    // shader and pipeline object
    state.pip_fill = sg_make_pipeline(&(sg_pipeline_desc){
        .shader = sg_make_shader(shapes_shader_desc(sg_query_backend())),
        .layout = {
            .buffers[0] = sshape_vertex_buffer_layout_state(),
            .attrs = {
                [0] = sshape_position_vertex_attr_state(),
                [1] = sshape_normal_vertex_attr_state(),
                [2] = sshape_texcoord_vertex_attr_state(),
                [3] = sshape_color_vertex_attr_state()
            }
        },
        .index_type = SG_INDEXTYPE_UINT16,
        .cull_mode = SG_CULLMODE_NONE,
        .depth = {
            .compare = SG_COMPAREFUNC_LESS_EQUAL,
            .write_enabled = true
        },
        .label = "fill-pipeline",
    });

    state.pip_lines = sg_make_pipeline(&(sg_pipeline_desc){
        .shader = sg_make_shader(shapes_shader_desc(sg_query_backend())),
        .layout = {
            .buffers[0] = sshape_vertex_buffer_layout_state(),
            .attrs = {
                [0] = sshape_position_vertex_attr_state(),
                [1] = sshape_normal_vertex_attr_state(),
                [2] = sshape_texcoord_vertex_attr_state(),
                [3] = sshape_color_vertex_attr_state()
            }
        },
        .index_type = SG_INDEXTYPE_UINT16,
        .cull_mode = SG_CULLMODE_BACK,
        .primitive_type = SG_PRIMITIVETYPE_LINES,
        .depth = {
            .compare = SG_COMPAREFUNC_LESS_EQUAL,
            .write_enabled = true
        },
        .label = "lines-pipeline",
    });

    // shape positions
    state.shapes[BOX].pos = HMM_V3(-1.0f, 1.0f, 0.0f);
    state.shapes[PLANE].pos = HMM_V3(1.0f, 1.0f, 0.0f);
    state.shapes[SPHERE].pos = HMM_V3(-2.0f, -1.0f, 0.0f);
    state.shapes[CYLINDER].pos = HMM_V3(2.0f, -1.0f, 0.0f);
    state.shapes[TORUS].pos = HMM_V3(0.0f, -1.0f, 0.0f);

    // generate shape geometries
    sshape_vertex_t vertices[6 * 1024];
    uint16_t indices[16 * 1024];
    sshape_buffer_t buf = {
        .vertices.buffer = SSHAPE_RANGE(vertices),
        .indices.buffer  = SSHAPE_RANGE(indices),
    };
    buf = sshape_build_box(&buf, &(sshape_box_t){
        .width  = 1.0f,
        .height = 1.0f,
        .depth  = 1.0f,
        .tiles  = 10,
        .random_colors = true,
    });
    state.shapes[BOX].draw = sshape_element_range(&buf);
    buf = sshape_build_plane(&buf, &(sshape_plane_t){
        .width = 1.0f,
        .depth = 1.0f,
        .tiles = 10,
        .random_colors = true,
    });
    state.shapes[PLANE].draw = sshape_element_range(&buf);
    buf = sshape_build_sphere(&buf, &(sshape_sphere_t) {
        .radius = 0.75f,
        .slices = 36,
        .stacks = 20,
        .random_colors = true,
    });
    state.shapes[SPHERE].draw = sshape_element_range(&buf);
    buf = sshape_build_cylinder(&buf, &(sshape_cylinder_t) {
        .radius = 0.5f,
        .height = 1.5f,
        .slices = 36,
        .stacks = 10,
        .random_colors = true,
    });
    state.shapes[CYLINDER].draw = sshape_element_range(&buf);
    buf = sshape_build_torus(&buf, &(sshape_torus_t) {
        .radius = 0.5f,
        .ring_radius = 0.3f,
        .rings = 36,
        .sides = 18,
        .random_colors = true,
    });
    state.shapes[TORUS].draw = sshape_element_range(&buf);
    assert(buf.valid);

    // one vertex/index-buffer-pair for all shapes
    const sg_buffer_desc vbuf_desc = sshape_vertex_buffer_desc(&buf);
    const sg_buffer_desc ibuf_desc = sshape_index_buffer_desc(&buf);
    state.vbuf = sg_make_buffer(&vbuf_desc);
    state.ibuf = sg_make_buffer(&ibuf_desc);
}

static void frame(void) {
    const int fb_width = sapp_width();
    const int fb_height = sapp_height();
    cam_update(&state.camera, fb_width, fb_height);

    // help text
    sdtx_canvas(fb_width*0.5f, fb_height*0.5f);
    sdtx_pos(0.5f, 0.5f);
    sdtx_puts("press key to switch draw mode:\n\n"
              "  1: vertex normals\n"
              "  2: texture coords\n"
              "  3: vertex color\n"
              "  4: line/fill mode\n");

    // view-projection matrix...
    // HMM_Mat4 proj = HMM_Perspective_RH_ZO(60.0f * HMM_DegToRad, sapp_widthf()/sapp_heightf(), 0.01f, 10.0f);
    // HMM_Mat4 view = HMM_LookAt_RH(HMM_V3(0.0f, 1.5f, 6.0f), HMM_V3(0.0f, 0.0f, 0.0f), HMM_V3(0.0f, 1.0f, 0.0f));
    // HMM_Mat4 view_proj = HMM_MulM4(proj, view);
    HMM_Mat4 proj = state.camera.proj;
    HMM_Mat4 view = state.camera.view;
    HMM_Mat4 view_proj = HMM_MulM4(proj, view);

    // model-rotation matrix
    const float t = (float)(sapp_frame_duration() * 60.0);
    state.rx += 1.0f * t * HMM_DegToRad;
    state.ry += 2.0f * t * HMM_DegToRad;
    HMM_Mat4 rxm = HMM_Rotate_RH(state.rx, HMM_V3(1.0f, 0.0f, 0.0f));
    HMM_Mat4 rym = HMM_Rotate_RH(state.ry, HMM_V3(0.0f, 1.0f, 0.0f));
    HMM_Mat4 rm = HMM_MulM4(rxm, rym);

    // render shapes...
    sg_begin_pass(&(sg_pass){ .action = state.pass_action, .swapchain = sglue_swapchain() });
    if (state.line_mode) {
        sg_apply_pipeline(state.pip_lines);
    } else {
        sg_apply_pipeline(state.pip_fill);
    }
    sg_apply_bindings(&(sg_bindings) {
        .vertex_buffers[0] = state.vbuf,
        .index_buffer = state.ibuf
    });
    for (int i = 0; i < NUM_SHAPES; i++) {
        // per shape model-view-projection matrix
        HMM_Mat4 model = HMM_MulM4(HMM_Translate(state.shapes[i].pos), rm);
        state.vs_params.mvp = HMM_MulM4(view_proj, model);
        sg_apply_uniforms(UB_shapes_vs_params, &SG_RANGE(state.vs_params));
        sg_draw(state.shapes[i].draw.base_element, state.shapes[i].draw.num_elements, 1);
    }
    sdtx_draw();
    sg_end_pass();
    sg_commit();
}

static void input(const sapp_event* ev) {
    if (ev->type == SAPP_EVENTTYPE_KEY_DOWN) {
        switch (ev->key_code) {
            case SAPP_KEYCODE_1: state.vs_params.draw_mode = 0.0f; break;
            case SAPP_KEYCODE_2: state.vs_params.draw_mode = 1.0f; break;
            case SAPP_KEYCODE_3: state.vs_params.draw_mode = 2.0f; break;
            case SAPP_KEYCODE_4: state.line_mode = !state.line_mode; break;
            default: break;
        }
    }
    cam_handle_event(&state.camera, ev);
}

static void cleanup(void) {
    sdtx_shutdown();
    sg_shutdown();
}

sapp_desc sokol_main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    return (sapp_desc) {
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = input,
        .width = 800,
        .height = 600,
        .sample_count = 4,
        .window_title = "sokol-shapes.c",
        .icon.sokol_default = true,
        .logger.func = slog_func,
    };
}
