#version 450

flat in uint vertex_id;
uniform uint selected_id;
uniform float border_size;
uniform vec4 point_color;
uniform vec4 border_color;
uniform vec4 selected_color;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_id;

void main() {

    // Rendering
    float point_thresh = 0.5;
    float border_tresh = point_thresh - border_size;
    vec2 coord = gl_PointCoord - vec2(point_thresh);
    float dist = length(coord);

    if (dist > point_thresh) {
        discard;
    } else if (dist > border_tresh) {
        out_color = border_color;
    } else if (selected_id == vertex_id) {
        out_color = selected_color;
    } else {
        out_color = point_color;
    }

    // ID encoding
    uint id = vertex_id + 1;
    float r = float((id >>  0) & 0xFF) / 255.0;
    float g = float((id >>  8) & 0xFF) / 255.0;
    float b = float((id >> 16) & 0xFF) / 255.0;
    out_id = vec4(r, g, b, 1.0);
}