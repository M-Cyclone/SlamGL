#version 420 core
layout (location = 0) in vec3 a_position;
layout (location = 1) in vec3 a_color;

out vec3 v_color;

uniform mat4 u_proj;
uniform mat4 u_view;

void main()
{
	v_color = a_color;
	gl_Position = u_proj * u_view * vec4(a_position, 1.0);
}