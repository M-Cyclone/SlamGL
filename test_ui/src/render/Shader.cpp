#include "Shader.h"

Shader::Shader(const std::string& vert_path, const std::string& frag_path)
{
    int success;
    char infoLog[1024];

    auto vertCode = readFile(vert_path);
    auto fragCode = readFile(frag_path);
    const char* vertStr = vertCode.c_str();
    const char* fragStr = fragCode.c_str();

    uint32_t vertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertShader, 1, &vertStr, nullptr);
    glCompileShader(vertShader);
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertShader, 1024, nullptr, infoLog);
        DEBUG_ERROR("Failed to compile vertex shader with error info: {0}", infoLog);
        assert(false);
    }

    uint32_t fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragShader, 1, &fragStr, nullptr);
    glCompileShader(fragShader);
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragShader, 1024, nullptr, infoLog);
        DEBUG_ERROR("Failed to compile fragment shader with error info: {0}", infoLog);
        assert(false);
    }

    m_program = glCreateProgram();
    glAttachShader(m_program, vertShader);
    glAttachShader(m_program, fragShader);
    glLinkProgram(m_program);
    glGetProgramiv(m_program, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(m_program, 1024, nullptr, infoLog);
        DEBUG_ERROR("Failed to link shader to the program with error info: {0}", infoLog);
        assert(false);
    }

    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    assert((m_program != 0) && "Failed to create shader.");
}
