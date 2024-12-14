#pragma once

#include <unordered_map>
#include <queue>

template <typename T>
struct vec3
{
    vec3() = default;
    vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {};
    T x;
    T y;
    T z;
};

template <typename T>
struct vec4
{
    vec3() = default;
    vec3(T xx, T yy, T zz, T ww) : x(xx), y(yy), z(zz), w(ww) {};
    T x;
    T y;
    T z;
    T w;
};

struct VertexData
{
    std::vector<vec3<unsigned int> *> adjacentTriangles;
    vec4<int> matrices;
    vec3<float> coordinates;
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
    vec4<int> planeMatrix;
    bool isValid = false;
};

struct Mesh
{
    Mesh() = default;

    // counts
    unsigned int vertexNb;
    unsigned int triangleNb;
    unsigned int edgeNb;

    vec3<float> *vertices;
    vec3<unsigned int> *triangles;
    std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edgesMap;

    // edges
    std::priority_queue<Edge> edgesPriorityQueue;
};
