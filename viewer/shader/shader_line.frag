#version 330 core

uniform float width;
uniform vec4 uColor; //u: uniform
out vec4 fragColor;

void main()
{
   fragColor = uColor;
}
