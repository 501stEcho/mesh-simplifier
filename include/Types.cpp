#include "Types.hpp"

bool EdgeIndex::operator<(const EdgeIndex& other) const
{
    Mesh *mesh = Mesh::getSingleInstance();

    bool thisExists = mesh->edgesMap[v1].find(v2) != mesh->edgesMap[v1].end();
    bool otherExists = mesh->edgesMap[other.v1].find(other.v2) != mesh->edgesMap[other.v1].end();

    if (thisExists && otherExists) {
        if (mesh->edgesMap[v1][v2].error != mesh->edgesMap[other.v1][other.v2].error) {
            return mesh->edgesMap[v1][v2].error < mesh->edgesMap[other.v1][other.v2].error;
        } else {
            if (v1 != other.v1)
                return v1 < other.v1;
            return v2 < other.v2; 
        }
    }

    if (thisExists != otherExists)
        return thisExists;

    if (v1 != other.v1)
        return v1 < other.v1;
    return v2 < other.v2;
}