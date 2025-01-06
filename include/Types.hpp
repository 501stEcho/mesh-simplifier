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

    T data[3][3];
};

template <typename T>
struct mat4
{
    mat4()
    {
        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++)
                data[i][j] = 0;
        }
    }

    mat4(T a, T b, T c, T d, T e, T f, T g, T h, T i, T j, T k, T l, T m, T n, T o, T p)
    {
        data[0][0] = a;
        data[0][1] = b;
        data[0][2] = c;
        data[0][3] = d;
        data[1][0] = e;
        data[1][1] = f;
        data[1][2] = g;
        data[1][3] = h;
        data[2][0] = i;
        data[2][1] = j;
        data[2][2] = k;
        data[2][3] = l;
        data[3][0] = m;
        data[3][1] = n;
        data[3][2] = o;
        data[3][3] = p;
    }

    mat4(const mat4<T> &other)
    {
        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++)
                data[i][j] = other.data[i][j];
        }
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

    mat4<T> operator+(mat4<T> &&other)
    {
        mat4<T> result;

        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++) {
                result[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    mat4<T> operator*(T &&coef)
    {
        mat4<T> result;

        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++) {
                result[i][j] = data[i][j] * coef;
            }
        }
        return result;
    }

    vec4<T> operator*(const vec4<T> &vec) const
    {
        vec4<T> result;

        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
        result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;

        return result;
    }

    vec4<T> operator*(const vec4<T> &&vec) const
    {
        vec4<T> result;

        result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
        result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
        result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
        result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;

        return result;
    }

    void getTranspose(mat4<T> &transpose)
    {
        for(size_t i = 0; i < 4; i++) {
            for(size_t j = 0; j < 4; j++) {
                transpose[j][i] = data[i][j];
            }
        }
    }

    double getDeterminant() {
        return 
            data[0][0] * (data[1][1] * (data[2][2] * data[3][3] - data[2][3] * data[3][2])
                          - data[1][2] * (data[2][1] * data[3][3] - data[2][3] * data[3][1])
                          + data[1][3] * (data[2][1] * data[3][2] - data[2][2] * data[3][1]))
          - data[0][1] * (data[1][0] * (data[2][2] * data[3][3] - data[2][3] * data[3][2])
                          - data[1][2] * (data[2][0] * data[3][3] - data[2][3] * data[3][0])
                          + data[1][3] * (data[2][0] * data[3][2] - data[2][2] * data[3][0]))
          + data[0][2] * (data[1][0] * (data[2][1] * data[3][3] - data[2][3] * data[3][1])
                          - data[1][1] * (data[2][0] * data[3][3] - data[2][3] * data[3][0])
                          + data[1][3] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]))
          - data[0][3] * (data[1][0] * (data[2][1] * data[3][2] - data[2][2] * data[3][1])
                          - data[1][1] * (data[2][0] * data[3][2] - data[2][2] * data[3][0])
                          + data[1][2] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]));
    }

    bool getInverse(mat4<T> &inverse)
    {
        double augmented[4][8]; // Augmented matrix (original matrix + identity)
    
        // Initialize the augmented matrix
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                augmented[i][j] = data[i][j];
                augmented[i][j + 4] = (i == j) ? 1.0 : 0.0; // Identity matrix
            }
        }

        // Perform Gaussian elimination
        for (int i = 0; i < 4; ++i) {
            // Find the pivot (largest element in column i)
            double maxEl = fabs(augmented[i][i]);
            int row = i;
            for (int k = i + 1; k < 4; ++k) {
                if (fabs(augmented[k][i]) > maxEl) {
                    maxEl = fabs(augmented[k][i]);
                    row = k;
                }
            }

            // Swap rows if necessary
            if (i != row) {
                for (int j = 0; j < 8; ++j) {
                    std::swap(augmented[i][j], augmented[row][j]);
                }
            }

            // Make the pivot element 1 and eliminate below
            double pivot = augmented[i][i];
            if (pivot == 0) {
                return false;
            }

            for (int j = 0; j < 8; ++j) {
                augmented[i][j] /= pivot;
            }

            // Eliminate other elements in the column
            for (int k = 0; k < 4; ++k) {
                if (k != i) {
                    double factor = augmented[k][i];
                    for (int j = 0; j < 8; ++j) {
                        augmented[k][j] -= augmented[i][j] * factor;
                    }
                }
            }
        }

        // Extract the inverse matrix from the augmented matrix
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                inverse[i][j] = augmented[i][j + 4];
            }
        }

        return true;
    }

    static mat4<T> identity()
    {
        mat4<T> result;
        result[0][0] = T(1);
        result[1][1] = T(1);
        result[2][2] = T(1);
        result[3][3] = T(1);
        return result;
    }

    T data[4][4];
};

template <typename T>
std::ostream &operator<<(std::ostream &stream, mat4<T> &matrix)
{
    for (unsigned int i = 0; i < 4; i++) {
        for (unsigned int j = 0; j < 4; j++)
            stream << matrix[i][j] << " ";
        stream << std::endl;
    }
    return stream;
}

template <typename T>
vec4<T> operator*(const vec4<T> &vec, const mat4<T> &matrix)
{
    vec4<T> result;
    result.x = vec.x * matrix.data[0][0] + vec.y * matrix.data[1][0] + vec.z * matrix.data[2][0] + vec.w * matrix.data[3][0];
    result.y = vec.x * matrix.data[0][1] + vec.y * matrix.data[1][1] + vec.z * matrix.data[2][1] + vec.w * matrix.data[3][1];
    result.z = vec.x * matrix.data[0][2] + vec.y * matrix.data[1][2] + vec.z * matrix.data[2][2] + vec.w * matrix.data[3][2];
    result.w = vec.x * matrix.data[0][3] + vec.y * matrix.data[1][3] + vec.z * matrix.data[2][3] + vec.w * matrix.data[3][3];
    return result;
}

template <typename T>
vec4<T> operator*(const mat4<T> &matrix, const vec4<T> &vec)
{
    vec4<T> result;

    result.x = matrix.data[0][0] * vec.x + matrix.data[0][1] * vec.y + matrix.data[0][2] * vec.z + matrix.data[0][3] * vec.w;
    result.y = matrix.data[1][0] * vec.x + matrix.data[1][1] * vec.y + matrix.data[1][2] * vec.z + matrix.data[1][3] * vec.w;
    result.z = matrix.data[2][0] * vec.x + matrix.data[2][1] * vec.y + matrix.data[2][2] * vec.z + matrix.data[2][3] * vec.w;
    result.w = matrix.data[3][0] * vec.x + matrix.data[3][1] * vec.y + matrix.data[3][2] * vec.z + matrix.data[3][3] * vec.w;

    return result;
}

struct VertexData
{
    VertexData()
    {
        adjacentTriangles = std::unordered_set<unsigned int>();
        adjacentVertices = std::unordered_set<unsigned int>();
        matrix = Eigen::Matrix4d();
        coordinates = Eigen::Vector3d();
    }

    VertexData(const VertexData &other)
    {
        adjacentTriangles = other.adjacentTriangles;
        adjacentVertices = other.adjacentVertices;
        matrix = other.matrix;
        coordinates = other.coordinates;
        isValid = other.isValid;
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
