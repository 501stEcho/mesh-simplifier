#pragma once

#include "Types.hpp"
#include <set>

struct CompareEdgePtr {
    bool operator()(const Edge *lhs, const Edge *rhs) const {
        return *lhs < *rhs;
    }
};

class QEMSimplifier {
    public:
        static void Simplify(Mesh *mesh, unsigned int iterationsNb);
    
    protected:
        static void GetSortedEdgeQueue(Mesh *mesh, std::set<Edge *, CompareEdgePtr> &result);
        static void UpdateAdjacentTriangle(unsigned int triangleIndex, Edge &edge, Mesh *mesh, vec4<double> new_position);
        static void UpdateAdjacentVertices(Edge &edge, Mesh *mesh, std::set<Edge *, CompareEdgePtr> &sortedEdges);
        static void DeleteTriangle(unsigned int triangleIndex, Mesh *mesh);
        static void DeleteVertex(unsigned int vertexIndex, Mesh *mesh);
        static unsigned int FindLatestParentVertex(unsigned int v, std::unordered_map<unsigned int, unsigned int> &parent);
};