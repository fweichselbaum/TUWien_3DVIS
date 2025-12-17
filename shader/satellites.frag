#version 450

uniform float border_size;
uniform vec4 point_color;
uniform vec4 border_color;

out vec4 fragColor;
out vec4 p3d_FragColor;

void main() {

    // Rendering

    float point_thresh = 0.5;
    float border_tresh = point_thresh - border_size;
    vec2 coord = gl_PointCoord - vec2(point_thresh);
    float dist = length(coord);

    if (dist > point_thresh) {
        discard;
    } else if (dist > border_tresh) {
        fragColor = border_color;
    } else {
        fragColor = point_color;
    }

    // ID encoding

    uint id = vertex_id + 1;
    float r = float((id >>  0) & 0xFF) / 255.0;
    float g = float((id >>  8) & 0xFF) / 255.0;
    float b = float((id >> 16) & 0xFF) / 255.0;

    p3d_FragColor = vec4(r, g, b, 1.0);
}