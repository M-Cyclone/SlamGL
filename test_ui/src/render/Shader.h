#pragma once
#include <cassert>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "utils/Log.h"

class Shader
{
public:
    Shader(const std::string& vert_path, const std::string& frag_path);
    Shader(const Shader&)            = delete;
    Shader& operator=(const Shader&) = delete;
    ~Shader() noexcept { glDeleteProgram(m_program); }

    void bind() const { glUseProgram(m_program); }
    void unbind() const { glUseProgram(0); }

    template <typename T>
    void setValue(const std::string& name, const T& value)
    {
        assert(false && "Shader value type not supported.");
    }

private:
    static std::string readFile(const std::string& path)
    {
        std::ifstream file(path);
        assert(file.is_open());

        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }

private:
    uint32_t m_program = 0;
};

template <>
inline void Shader::setValue(const std::string& name, const int& value)
{
    glUniform1i(glGetUniformLocation(m_program, name.c_str()), value);
}
template <>
inline void Shader::setValue(const std::string& name, const float& value)
{
    glUniform1f(glGetUniformLocation(m_program, name.c_str()), value);
}
template <>
inline void Shader::setValue(const std::string& name, const glm::vec2& value)
{
    glUniform2f(glGetUniformLocation(m_program, name.c_str()),
                value.x,
                value.y);
}
template <>
inline void Shader::setValue(const std::string& name, const glm::vec3& value)
{
    glUniform3f(glGetUniformLocation(m_program, name.c_str()),
                value.x,
                value.y,
                value.z);
}
template <>
inline void Shader::setValue(const std::string& name, const glm::vec4& value)
{
    glUniform4f(glGetUniformLocation(m_program, name.c_str()),
                value.x,
                value.y,
                value.z,
                value.w);
}
template <>
inline void Shader::setValue(const std::string& name, const glm::mat3& value)
{
    glUniformMatrix3fv(glGetUniformLocation(m_program, name.c_str()),
                       1,
                       GL_FALSE,
                       glm::value_ptr(value));
}
template <>
inline void Shader::setValue(const std::string& name, const glm::mat4& value)
{
    glUniformMatrix4fv(glGetUniformLocation(m_program, name.c_str()),
                       1,
                       GL_FALSE,
                       glm::value_ptr(value));
}