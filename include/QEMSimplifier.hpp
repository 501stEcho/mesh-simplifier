#pragma once

#include "Types.hpp"
#include <set>

class QEMSimplifier {
    public:
        static void Simplify(Mesh *mesh, unsigned int iterationsNb);
    
    protected:
        static void GetSortedEdgeQueue(Mesh *mesh, std::set<EdgeIndex> &result);
        static void UpdateAdjacentTriangle(unsigned int triangleIndex, EdgeIndex &edge, Mesh *mesh);
        static void UpdateAdjacentVertices(EdgeIndex &edge, Mesh *mesh, std::set<EdgeIndex> &sortedEdges);
        static void DeleteTriangle(unsigned int triangleIndex, Mesh *mesh);
        static void DeleteVertex(unsigned int vertexIndex, Mesh *mesh);
        static unsigned int FindLatestParentVertex(unsigned int v, std::unordered_map<unsigned int, unsigned int> &parent);
};