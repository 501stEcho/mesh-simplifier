#pragma once

#include "Types.hpp"

// Computes the optimal position and returns the error
double ComputeEdgeOptimalPosition(vec4<double> &optimal_position, Edge &edge, Mesh *mesh);
void ComputePlaneEquation(std::vector<VertexData> &vertexArray, TriangleData &triangle);
void ComputeVertexMatrix(unsigned int vertexIndex, Mesh *mesh, bool recomputePlaneEquation = false);
bool AddEdge(Mesh *mesh, unsigned int v1, unsigned int v2);

