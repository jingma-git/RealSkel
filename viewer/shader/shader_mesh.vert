#version 330 core

in vec3 aPos;
in vec3 aNormal;
in vec3 aColor;

uniform mat4 mvp;
uniform mat4 mv;
uniform mat3 normalMatrix;

out vec3 FragPos;
out vec3 Normal;
out vec3 Color;
out float Depth;

void main(){
   gl_Position = mvp * vec4(aPos, 1.0); 
   FragPos = (mv * vec4(aPos, 1.0)).xyz;
   Normal = normalMatrix * aNormal;
   Color = aColor;
   Depth = gl_Position.z;
}
