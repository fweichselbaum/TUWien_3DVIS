#version 460

uniform mat4 p3d_ModelViewProjectionMatrix;
uniform mat4 p3d_ModelViewMatrix;

in vec4 p3d_Vertex;
in vec4 p3d_Color;

uniform float base_point_size;
uniform float scale_factor;

out vec4 v_color;

void main() {
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;

    //vec4 view_pos = p3d_ModelViewMatrix * p3d_Vertex;
    //gl_PointSize = (base_point_size * scale_factor) / length(view_pos.xyz);

    gl_PointSize = 100.0;//(base_point_size * scale_factor) / gl_Position.w;
    
    v_color = p3d_Color;
}