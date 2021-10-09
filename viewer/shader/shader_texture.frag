#version 330 core
in vec2 texc;

uniform sampler2D texSampler;

out vec4 fragColor;

void main()
{
   fragColor = texture2D(texSampler, texc); //built-in texture2D() function
   // fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
