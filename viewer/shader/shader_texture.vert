#version 330 core

in vec3 vertex;
in vec2 texCoord;

uniform mat4 mvp;

out vec2 texc;

void main()
{
    gl_Position = mvp * vec4(vertex, 1.0);
    texc = texCoord;
}
