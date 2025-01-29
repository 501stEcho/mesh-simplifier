#pragma once

#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>
#include <memory>
#include <cmath>
#include <unordered_set>
#include <Eigen/Dense>
#include "Bitmap.hpp"
#include <iomanip>

class InvalidMatrixIndexException : public std::exception
{
    public:
        InvalidMatrixIndexException(std::string msg) : _msg(msg) {};
        const char *what() const noexcept override
        {
            return _msg.c_str();
        }
    
    private:
        std::string _msg;
};

template <typename T>
struct vec3
{
    vec3() = default;
    vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {};
    vec3<T> operator-(vec3<T> &other)
    {
        return vec3<T>(x - other.x, y - other.y, z - other.z);
    }

    vec3<T> operator+(vec3<T> &other)
    {
        return vec3<T>(x + other.x, y + other.y, z + other.z);
    }

    vec3<T> operator*(T &&coef)
    {
        return vec3<T>(x * coef, y * coef, z * coef);
    }

    vec3<T> cross_product(vec3<T> &other)
    {
        return vec3<T>(y * other.z - z * other.y, -1 * (x * other.z - z * other.x), x * other.y - y * other.x);
    }

    T dot_product(vec3<T> &other)
    {
        return x * other.x + y * other.y + z * other.z;
    }

    T length_squared() const {
        return x * x + y * y + z * z;
    }

    vec3<T> &normalize(void)
    {
        T magnitude = sqrt(x * x + y * y + z * z);
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        return *this;
    }

    T x;
    T y;
    T z;
};

template <typename T>
struct vec4
{
    vec4() = default;
    vec4(T xx, T yy, T zz, T ww) : x(xx), y(yy), z(zz), w(ww) {};
    vec4(vec3<T> v, T ww) : x(v.x), y(v.y), z(v.z), w(ww) {};
    vec4<T> operator+(vec4<T> &&other)
    {
        return vec4<T>(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    vec4<T> operator-(vec4<T> &other)
    {
        return vec4<T>(x - other.x, y - other.y, z - other.z, w - other.w);
    }

    double dot(vec4<T> &other)
    {
        return (x * other.x + y * other.y + z * other.z);
    }

    double dot(vec4<T> &&other)
    {
        return (x * other.x + y * other.y + z * other.z);
    }

    vec4<T> &operator+=(vec4<T> &&other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }
    vec4<T> &operator+=(vec4<T> &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    vec4<T> &normalize(void)
    {
        T magnitude = sqrt(x * x + y * y + z * z + w * w);
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        w /= magnitude;
        return *this;
    }

    T x;
    T y;
    T z;
    T w;
};


template <typename T>
std::ostream &operator<<(std::ostream &stream, vec4<T> &vector)
{
    stream << vector.x << " " << vector.y << " " << vector.z << " " << vector.w << std::endl;
    return stream;
}

template <typename T>
std::ostream &operator<<(std::ostream &stream, vec4<T> &&vector)
{
    stream << vector.x << " " << vector.y << " " << vector.z << " " << vector.w << std::endl;
    return stream;
}

template <typename T>
vec4<T> operator*(T coef, vec4<T> &other)
{
    return vec4<T>(other.x * coef, other.y * coef, other.z * coef, other.w * coef);
}

struct VertexData
{
    VertexData()
    {
        adjacentTriangles = std::unordered_set<unsigned int>();
        adjacentVertices = std::unordered_set<unsigned int>();
        matrix = Eigen::Matrix4d::Zero();
        coordinates = Eigen::Vector3d();
    }

    std::unordered_set<unsigned int> adjacentTriangles;
    std::unordered_set<unsigned int> adjacentVertices;
    Eigen::Matrix4d matrix;
    // mat4<double> matrix;
    Eigen::Vector3d coordinates;
    bool isValid = true;
};

struct EdgeIndex
{
    EdgeIndex() = default;
    EdgeIndex(unsigned int vertex_1, unsigned int vertex_2) : v1(vertex_1), v2(vertex_2) {};
    unsigned int v1;
    unsigned int v2;
    bool isValid = true;

    bool operator<(const EdgeIndex& other) const;
};

struct EdgeData
{
    EdgeData() = default;
    EdgeData(double err) : error(err) {};
    double error;
    bool isValid = true;

    bool operator<(const EdgeData& other) const {
        const double epsilon = 1e-9;
        if (std::fabs(error - other.error) > epsilon)
            return error < other.error;
        return false;
    }
};



struct TriangleData
{
    vec3<unsigned int> verticesIndex;
    Eigen::Vector4d plane;
    bool isValid = true;
};

struct Mesh
{
    Mesh()
    {
        activeVertices = Bitmap(true);
        activeTriangles = Bitmap(true);
        triangleNb = 0;
        vertexNb = 0;
        edgeNb = 0;
    }

    static Mesh *getSingleInstance()
    {
        static Mesh *ptr;

        if (!ptr)
            ptr = new Mesh();
        
        return ptr;
    }

    // counts
    unsigned int vertexNb;
    unsigned int triangleNb;
    unsigned int edgeNb;

    std::vector<VertexData> vertices;
    Bitmap activeVertices;
    std::vector<TriangleData> triangles;
    Bitmap activeTriangles;
    std::unordered_map<unsigned int, std::unordered_map<unsigned int, EdgeData>> edgesMap;

    // Buffer size
    unsigned int verticesBufferSize;
    unsigned int trianglesBufferSize;
};
