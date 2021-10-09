#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 200) out;

in VS_OUT{
    vec3 color;
    float selected;
}gs_in[];

out vec4 fColor;

uniform vec4 uColor;
uniform float uPointSize;
uniform bool uSelected;

void build_circle(vec4 position, vec4 color, float len, float z){
    float PI = 3.14159265358979323846;
    float PI2 = 2*PI;
    float step = PI / 4.0;
    float angle = 0.0;

    fColor = color;
    
    while(angle < PI2) {
        gl_Position = position; // center   
        EmitVertex(); 

        float a = angle;
        float x = len * cos(a);
        float y = len * sin(a);
        gl_Position = position + vec4(x, y, 0, 0);
        EmitVertex();

        float a1 = angle + step;
        float x1 = len * cos(a1);
        float y1 = len * sin(a1);
        gl_Position = position + vec4(x1, y1, 0, 0);
        EmitVertex();
        angle += step;
    }
    EndPrimitive();
}


void main() {
    float selected = gs_in[0].selected;
    vec3 color = gs_in[0].color;
    float radius = 0.5 * uPointSize;
    if(selected > 0.5){
      build_circle(gl_in[0].gl_Position, vec4(0.0, 0.0, 0.0, 1.0), radius-0.5*radius, 0);
    }
    build_circle(gl_in[0].gl_Position, vec4(color[0], color[1], color[2], 1.0), radius, 0); 
    build_circle(gl_in[0].gl_Position, vec4(0.0, 0.0, 0.0, 1.0), radius+0.1*radius, 0);
    build_circle(gl_in[0].gl_Position, vec4(1.0, 1.0, 1.0, 1.0), radius+0.2*radius, 0.0); 
}
