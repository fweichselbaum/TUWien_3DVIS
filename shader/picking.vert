#version 450

uniform mat4 p3d_ModelViewProjectionMatrix;

in vec4 p3d_Vertex;


//uniform float point_size;

flat out uint vertex_id;

void main() {
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
    
    gl_PointSize = max(2.0, 1000 / gl_Position.w);

    vertex_id = gl_VertexID;
}