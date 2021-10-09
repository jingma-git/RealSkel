#version 330 core

uniform mat4 mvp;

in vec3 aPos; //a: attribute

void main()
{
    gl_Position = mvp * vec4(aPos, 1.0);
}
