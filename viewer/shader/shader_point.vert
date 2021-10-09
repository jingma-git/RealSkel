#version 330 core

uniform mat4 mvp;

in vec3 aPos; //a: attribute
in vec3 aColor;
in float aSelected;

out VS_OUT{
    vec3 color;
    float selected;
}vs_out;

void main()
{
    gl_Position = mvp * vec4(aPos, 1.0);
    vs_out.color = aColor;
    vs_out.selected = aSelected;
}
