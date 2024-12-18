#pragma once

#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>
#include <cmath>
#include <unordered_set>

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
    vec3<T> cross_product(vec3<T> &other)
    {
        return vec3<T>(y * other.z - z * other.y, -1 * (x * other.z - z * other.x), x * other.y - y * other.x);
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
    vec4<T> operator+(vec4<T> &&other)
    {
        return vec4<T>(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    double dot(vec4<T> &other)
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

    T x;
    T y;
    T z;
    T w;
};

template <typename T>
struct mat3
{
    mat3()
    {
    }

    T *operator[](unsigned int index)
    {
        if (index > 2)
            throw InvalidMatrixIndexException("Index bigger than 2 for a 3x3 matrix");
        
        return data[index];
    }

    mat3<T> operator+(mat3<T> &other)
    {
        mat3<T> result;

        for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = 0; j < 3; j++) {
                result[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    double getDeterminant(void)
    {
        return data[0][0] * (data[1][1] * data[2][2]- data[1][2]* data[2][1]) -
                 data[0][1] * (data[1][0] * data[2][2]- data[1][2]* data[2][0]) +
                 data[0][2]* (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
    }

    bool invert(mat3<T>& inverse) {
        double det = getDeterminant();

        if (det == 0) {
            return false;
        }

        double invDet = 1.0 / det;

        inverse[0][0] =  (data[1][1] * data[2][2] - data[1][2] * data[2][1]) * invDet;
        inverse[0][1] = -(data[0][1] * data[2][2] - data[0][2] * data[2][1]) * invDet;
        inverse[0][2] =  (data[0][1] * data[1][2] - data[0][2] * data[1][1]) * invDet;

        inverse[1][0] = -(data[1][0] * data[2][2] - data[1][2] * data[2][0]) * invDet;
        inverse[1][1] =  (data[0][0] * data[2][2] - data[0][2] * data[2][0]) * invDet;
        inverse[1][2] = -(data[0][0] * data[1][2] - data[0][2] * data[1][0]) * invDet;

        inverse[2][0] =  (data[1][0] * data[2][1] - data[1][1] * data[2][0]) * invDet;
        inverse[2][1] = -(data[0][0] * data[2][1] - data[0][1] * data[2][0]) * invDet;
        inverse[2][2] =  (data[0][0] * data[1][1] - data[0][1] * data[1][0]) * invDet;

        return true;
    }

    T data[3][3];
};

template <typename T>
struct mat4
{
    mat4()
    {
    }

    T *operator[](unsigned int index)
    {
        if (index > 3)
            throw InvalidMatrixIndexException("Index bigger than 3 for a 4x4 matrix");
        
        return data[index];
    }

    mat4<T> operator+(mat4<T> &other)
    {
        mat4<T> result;

        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++) {
                result[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    vec4<T> operator*(vec4<T> &vec)
    {
        vec4<T> result;

        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
        result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;

        return result;
    }

    vec4<T> operator*(vec4<T> &&vec)
    {
        vec4<T> result;

        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
        result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;

        return result;
    }

    void getCofactor(mat3<T> temp, int p, int q)
    {
        int row = 0, col = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i != p && j != q) {
                    temp[row][col++] = data[i][j];
                    if (col == 3) {
                        col = 0;
                        row++;
                    }
                }
            }
        }
    }

    double getDeterminant(void)
    {
        double det = 0;
        mat3<double> temp;
        int sign = 1;

        for (int j = 0; j < 4; j++) {
            getCofactor(temp, 0, j);
            det += sign * data[0][j] * temp.getDeterminant();
            sign = -sign;
        }

        return det;
    }

    void adjugate(mat4<T> &adj)
    {
        mat3<double> temp;
        int sign;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                getCofactor(temp, i, j);

                sign = ((i + j) % 2 == 0) ? 1 : -1;
                adj[j][i] = sign * temp.getDeterminant();
            }
        }
    }

    bool inverse(mat4<T> &inv) {
        double det = getDeterminant();

        if (fabs(det) < 1e-9) {
            std::cerr << "Matrix is singular and cannot be inverted." << std::endl;
            return false;
        }

        mat4<double> adj;
        adjugate(adj);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                inv[i][j] = adj[i][j] / det;
            }
        }

        return true;
    }

    T data[4][4];
};

template <typename T>
vec4<T> operator*(vec4<T> &vec, mat4<T> matrix)
{
    vec4<T> result;
    result.x = vec.x * matrix[0][0] + vec.y * matrix[1][0] + vec.z * matrix[2][0] + vec.w * matrix[3][0];
    result.y = vec.x * matrix[0][1] + vec.y * matrix[1][1] + vec.z * matrix[2][1] + vec.w * matrix[3][1];
    result.z = vec.x * matrix[0][2] + vec.y * matrix[1][2] + vec.z * matrix[2][2] + vec.w * matrix[3][2];
    result.w = vec.x * matrix[0][3] + vec.y * matrix[1][3] + vec.z * matrix[2][3] + vec.w * matrix[3][3];
    return result;
}

struct VertexData
{
    std::unordered_set<unsigned int> adjacentTriangles;
    std::unordered_set<unsigned int> adjacentVertices;
    mat4<double> matrix;
    vec3<double> coordinates;
    bool isValid = false;
};

struct Edge
{
    unsigned int v1;
    unsigned int v2;
    double error;

    bool operator<(const Edge& other) const {
        return error > other.error;
    }
};

struct TriangleData
{
    vec3<unsigned int> verticesIndex;
    vec4<double> plane;
    bool isValid = false;
};

struct Mesh
{
    Mesh() = default;

    // counts
    unsigned int vertexNb;
    unsigned int triangleNb;
    unsigned int edgeNb;

    VertexData *vertices;
    TriangleData *triangles;
    std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edgesMap;

    // edges
    std::priority_queue<Edge *> edgesPriorityQueue;
};
