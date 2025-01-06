#include "QEMSimplifier.hpp"
#include "Tools.hpp"
#include <iomanip>

std::set<EdgeIndex> sortedEdges;
std::unordered_map<unsigned int, unsigned int> parent;

void QEMSimplifier::Simplify(Mesh *mesh, unsigned int iterationsNb)
{
    GetSortedEdgeQueue(mesh, sortedEdges);

    for (unsigned int iteration = 0; iteration < iterationsNb && mesh->vertexNb > 1;) {
        EdgeIndex index = *sortedEdges.begin();
        sortedEdges.erase(sortedEdges.begin());

        if (!index.isValid || index.v1 == index.v2 || !mesh->activeVertices.isActive(index.v1)
            || !mesh->activeVertices.isActive(index.v2) || mesh->edgesMap[index.v1].find(index.v2) == mesh->edgesMap[index.v1].end()) {
            std::cerr << "Could not find " << index.v1 << "-" << index.v2 << std::endl;
            continue;
        }

        EdgeData &data = mesh->edgesMap[index.v1][index.v2];
        std::cout << "Iteration " << iteration << " : " << index.v1 << "-" << index.v2 << " (" << data.error << ")" << std::endl;

        // Assign new position to vertex 1
        vec4<double> optimal_position;
        ComputeEdgeOptimalPosition(optimal_position, index, mesh);
        mesh->vertices[index.v1].coordinates.x = optimal_position.x;
        mesh->vertices[index.v1].coordinates.y = optimal_position.y;
        mesh->vertices[index.v1].coordinates.z = optimal_position.z;

        // Replace v2 occurences in adjacent triangles by v1
        for (auto &it : mesh->vertices[index.v2].adjacentTriangles) {
            if (mesh->activeTriangles.isActive(it))
               UpdateAdjacentTriangle(it, index, mesh, optimal_position);
        }

        // Add nodes between v1 and v2's adjacent vertices
        UpdateAdjacentVertices(index, mesh, sortedEdges);

        // Delete v2
        DeleteVertex(index.v2, mesh);
        mesh->edgesMap[index.v1].erase(index.v2);
        iteration++;
    }

    // for (unsigned int iteration = 0; iteration < iterationsNb && mesh->vertexNb > 1;) {
    //     unsigned int v1_parent = FindLatestParentVertex(sortedEdges.begin()->v1, parent);
    //     unsigned int v2_parent = FindLatestParentVertex(sortedEdges.begin()->v2, parent);
    //     EdgeIndex index = *sortedEdges.begin();
    //     EdgeIndex index_parent = EdgeIndex(v1_parent, v2_parent);
    //     sortedEdges.erase(sortedEdges.begin());

    //     if (!index.isValid || v1_parent == v2_parent || !mesh->activeVertices.isActive(v1_parent) || !mesh->activeVertices.isActive(v2_parent))
    //         continue;

    //     unsigned int v1 = std::min(v1_parent, v2_parent);
    //     unsigned int v2 = std::max(v1_parent, v2_parent);
    //     EdgeData &data = mesh->edgesMap[v1][v2];
    //     std::cout << "Iteration " << iteration << " : " << v1 << "-" << v2 << " (" << data.error << ")" << std::endl;

    //     if (mesh->edgesMap[v1].find(v2) == mesh->edgesMap[v1].end())
    //         std::cerr << "Could not find " << v1 << "-" << v2 << std::endl;
    
    //     mat4<double> quadric = mesh->vertices[v1].matrix + mesh->vertices[v2].matrix;
    //     vec4<double> optimal_position;
    //     ComputeEdgeOptimalPosition(optimal_position, index_parent, mesh);

    //     mesh->vertices[v1].coordinates.x = optimal_position.x;
    //     mesh->vertices[v1].coordinates.y = optimal_position.y;
    //     mesh->vertices[v1].coordinates.z = optimal_position.z;

    //     for (auto &it : mesh->vertices[v2].adjacentTriangles) {
    //         if (mesh->activeTriangles.isActive(it))
    //            UpdateAdjacentTriangle(it, index_parent, mesh, optimal_position);
    //     }

    //     UpdateAdjacentVertices(index_parent, mesh, sortedEdges);
    //     DeleteVertex(v2 , mesh);
    //     mesh->edgesMap[v1].erase(v2);
    //     parent[v2] = v1;
    //     iteration++;
    // }
}
 
void QEMSimplifier::GetSortedEdgeQueue(Mesh *mesh, std::set<EdgeIndex> &result)
{
    for (auto &it : mesh->edgesMap) {
        for (auto &it2 : it.second) {
            EdgeIndex index(it.first, it2.first);
            mat4<double> quadric = mesh->vertices[index.v1].matrix + mesh->vertices[index.v2].matrix;
            vec4<double> optimal_position;
            ComputeEdgeOptimalPosition(optimal_position, index, mesh);
            // std::cout << "Inserting " << it2.second.v1 << "-"
            // << it2.second.v2 << " : "
            // << it2.second.error << std::endl;
            result.insert(index);
            // std::cout << "---------------------" << std::endl;
        }
    }
}

void QEMSimplifier::UpdateAdjacentTriangle(unsigned int it, EdgeIndex &edge, Mesh *mesh, vec4<double> new_position)
{
    unsigned int neighbour_vertex1;
    unsigned int neighbour_vertex2;

    TriangleData &triangle = mesh->triangles[it];
    if (triangle.verticesIndex.x == edge.v2) {
        if (triangle.verticesIndex.y == edge.v1 || triangle.verticesIndex.z == edge.v1)
            return DeleteTriangle(it, mesh);

        triangle.verticesIndex.x = edge.v1;
    } else if (triangle.verticesIndex.y == edge.v2) {
        if (triangle.verticesIndex.x == edge.v1 || triangle.verticesIndex.z == edge.v1)
            return DeleteTriangle(it, mesh);

        triangle.verticesIndex.y = edge.v1;
    } else if (triangle.verticesIndex.z == edge.v2) {
        if (triangle.verticesIndex.x == edge.v1 || triangle.verticesIndex.y == edge.v1)
            return DeleteTriangle(it, mesh);

        triangle.verticesIndex.z = edge.v1;
    } else
        std::cerr << "Trop bizarre" << std::endl;
    mesh->vertices[edge.v1].adjacentTriangles.insert(it);
}

void QEMSimplifier::UpdateAdjacentVertices(EdgeIndex &edge, Mesh *mesh, std::set<EdgeIndex> &sortedEdge)
{
    // Remove edge connected to v1 from the set to recompute them later
    for (auto &adj_vertex : mesh->vertices[edge.v1].adjacentVertices) {
        if (mesh->activeVertices.isActive(adj_vertex) && adj_vertex != edge.v2) {
            unsigned int v1 = std::min(edge.v1, adj_vertex);
            unsigned int v2 = std::max(edge.v1, adj_vertex);
            if (mesh->edgesMap[v1].find(v2) == mesh->edgesMap[v1].end()) {
                std::cerr << "Edge " << v1 << "-" << v2 << " not in the map" << std::endl;
                continue;
            }

            EdgeIndex index(v1, v2);
            auto it = sortedEdge.find(index);
            if (it != sortedEdge.end()) {
                sortedEdge.erase(it);
            } else {
                mesh->edgesMap[v1][v2].isValid = false;
                std::cout << "Setting " << v1 << "-" << v2 << " as inactive" << std::endl;
            }
        }
    }

    // Remove edge connected to v2 from the set to recompute them later
    for (auto &adj_vertex : mesh->vertices[edge.v2].adjacentVertices) {
        if (mesh->activeVertices.isActive(adj_vertex) && adj_vertex != edge.v1) {
            mesh->vertices[adj_vertex].adjacentVertices.erase(edge.v2);

            // Combine v2's adjacent vertices to v1
            mesh->vertices[edge.v1].adjacentVertices.insert(adj_vertex);
            mesh->vertices[adj_vertex].adjacentVertices.insert(edge.v1);
            unsigned int v1 = std::min(edge.v2, adj_vertex);
            unsigned int v2 = std::max(edge.v2, adj_vertex);
            if (mesh->edgesMap[v1].find(v2) == mesh->edgesMap[v1].end()) {
                std::cerr << "2- Edge " << v1 << "-" << v2 << " not in the map" << std::endl;
                continue;
            }

            EdgeIndex index(v1, v2);
            auto it = sortedEdge.find(index);
            if (it != sortedEdge.end()) {
                sortedEdge.erase(it);
                mesh->edgesMap[v1].erase(v2);
            } else {
                mesh->edgesMap[v1][v2].isValid = false;
                std::cout << "Setting " << v1 << "-" << v2 << " as inactive" << std::endl;
            }
        }
    }

    for (auto &adj_vertex : mesh->vertices[edge.v1].adjacentVertices) {
        if (mesh->activeVertices.isActive(adj_vertex)) {
            ComputeVertexMatrix(adj_vertex, mesh, true);
            unsigned int v1 = std::min(edge.v1, adj_vertex);
            unsigned int v2 = std::max(edge.v1, adj_vertex);

            // Insert those new edges in the set and in the map if they didn't exist before
            AddEdge(mesh, v1, v2);
            EdgeIndex index(v1, v2);
            // std::cout << "Edge " << index.v1 << "-" << index.v2 << " : " << std::endl;
            vec4<double> optimal_position;
            ComputeEdgeOptimalPosition(optimal_position, index, mesh);
            sortedEdge.insert(index);
        }
    }
}

void QEMSimplifier::DeleteTriangle(unsigned int triangleIndex, Mesh *mesh)
{
    mesh->activeTriangles.set(triangleIndex, false);
    mesh->triangleNb--;
}

void QEMSimplifier::DeleteVertex(unsigned int vertexIndex, Mesh *mesh)
{
    mesh->activeVertices.set(vertexIndex, false);
    mesh->vertexNb--;
}

unsigned int QEMSimplifier::FindLatestParentVertex(unsigned int value, std::unordered_map<unsigned int, unsigned int> &parent)
{
    if (parent.find(value) == parent.end())
        return value;

    if (parent[value] != value)
        parent[value] = FindLatestParentVertex(parent[value], parent); // Path compression
    return parent[value];
}
