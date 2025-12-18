#version 450

uniform float border_size;
uniform vec4 point_color;
uniform vec4 border_color;

out vec4 fragColor;

void main() {
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
}