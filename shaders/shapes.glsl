@ctype mat4 HMM_Mat4

@vs shapes_vs
layout(binding=0) uniform shapes_vs_params {
    mat4 m;
    mat4 vp;
    float draw_mode;
};

layout(location=0) in vec4 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec2 texcoord;
layout(location=3) in vec4 color0;

out vec4 color;

void main() {
    gl_Position = vp * m * position;
    if (draw_mode == 0.0) {
        const vec3 light = normalize( vec3( 0.3,0.7,0.5) );
        vec3 worldNormal = normalize( ( m * vec4(normal, 0.0) ).xyz );
        float NdotL = dot( worldNormal.xyz, light );
        float dif = clamp( (NdotL * 0.5 + 0.5), 0.0, 1.0 );
        color = color0 * dif;
    } else if (draw_mode == 1.0) {
        color = vec4((normal + 1.0) * 0.5, 1.0);
    }
    else if (draw_mode == 2.0) {
        color = vec4(texcoord, 0.0, 1.0);
    }
    else {
        color = color0;
    }
}
@end

@fs shapes_fs
in vec4 color;
out vec4 frag_color;

void main() {
    frag_color = color;
}
@end

@program shapes shapes_vs shapes_fs
