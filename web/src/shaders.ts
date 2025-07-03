// Shader source code for RSMotion visualization

export const vertexShaderSource = `#version 300 es
in vec3 a_position;
uniform mat4 u_modelMatrix;
uniform mat4 u_viewMatrix;
uniform mat4 u_projectionMatrix;

void main() {
    gl_Position = u_projectionMatrix * u_viewMatrix * u_modelMatrix * vec4(a_position, 1.0);
}`;

export const fragmentShaderSource = `#version 300 es
precision mediump float;

uniform vec4 u_color;
out vec4 fragColor;

void main() {
    fragColor = u_color;
}`;