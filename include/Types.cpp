#include "Types.hpp"

bool EdgeIndex::operator<(const EdgeIndex& other) const
{
    Mesh *mesh = Mesh::getSingleInstance();

    if (mesh->edgesMap[v1].find(v2) != mesh->edgesMap[v1].end()
        && mesh->edgesMap[other.v1].find(other.v2) != mesh->edgesMap[other.v1].end()) {
        return mesh->edgesMap[v1][v2] < mesh->edgesMap[other.v1][other.v2];
    }
    
    return (mesh->edgesMap[v1].find(v2) != mesh->edgesMap[v1].end());
}