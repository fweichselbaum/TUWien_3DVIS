#version 450

uniform sampler2D p3d_Texture0; // The texture assigned to the node

in vec2 texcoord;

layout(location = 0) out vec4 out_color; // Visual Buffer
layout(location = 1) out vec4 out_id;    // ID Buffer

void main() {
    out_color = texture(p3d_Texture0, texcoord);
    out_id = vec4(0.0, 0.0, 0.0, 1.0); 
}