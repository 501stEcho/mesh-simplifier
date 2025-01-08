#include "QEMSimplifier.hpp"
#include "Tools.hpp"
#include <iomanip>

std::set<EdgeIndex> sortedEdges;
std::unordered_map<unsigned int, unsigned int> parent;

void QEMSimplifier::Simplify(Mesh *mesh, unsigned int iterationsNb)
{
    GetSortedEdgeQueue(mesh, sortedEdges);

    for (unsigned int iteration = 0; iteration < iterationsNb && mesh->vertexNb > 1 && sortedEdges.size() > 0;) {
        EdgeIndex index = *sortedEdges.begin();
        sortedEdges.erase(sortedEdges.begin());

        if (!index.isValid || index.v1 == index.v2 || !mesh->activeVertices.isActive(index.v1)
            || !mesh->activeVertices.isActive(index.v2)
            || mesh->edgesMap[index.v1].find(index.v2) == mesh->edgesMap[index.v1].end()
            || !mesh->edgesMap[index.v1][index.v2].isValid) {
            continue;
        }

        EdgeData &data = mesh->edgesMap[index.v1][index.v2];
        std::cout << "Iteration " << iteration << " : " << index.v1 << "-" << index.v2 << " ( " << data.error << ")" << std::endl;

        // Assign new position to vertex 1
        Eigen::Vector4d optimal_position;
        ComputeEdgeOptimalPosition(optimal_position, index, mesh);
        mesh->vertices[index.v1].coordinates = optimal_position.head<3>();

        // Replace v2 occurences in adjacent triangles by v1
        for (auto &it : mesh->vertices[index.v2].adjacentTriangles) {
            if (mesh->activeTriangles.isActive(it))
               UpdateAdjacentTriangle(it, index, mesh);
        }

        // Add nodes between v1 and v2's adjacent vertices
        UpdateAdjacentVertices(index, mesh, sortedEdges);

        // Delete v2
        DeleteVertex(index.v2, mesh);
        mesh->edgesMap[index.v1].erase(index.v2);
        iteration++;
    }
}
 
void QEMSimplifier::GetSortedEdgeQueue(Mesh *mesh, std::set<EdgeIndex> &result)
{
    for (auto &it : mesh->edgesMap) {
        for (auto &it2 : it.second) {
            EdgeIndex index(it.first, it2.first);
            Eigen::Matrix4d quadric(mesh->vertices[index.v1].matrix + mesh->vertices[index.v2].matrix);
            Eigen::Vector4d optimal_position;
            ComputeEdgeOptimalPosition(optimal_position, index, mesh);
            result.insert(index);
        }
    }
}

void QEMSimplifier::UpdateAdjacentTriangle(unsigned int it, EdgeIndex &edge, Mesh *mesh)
{
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
            }
        }
    }

    for (auto &adj_vertex : mesh->vertices[edge.v1].adjacentVertices) {
        if (mesh->activeVertices.isActive(adj_vertex) && adj_vertex != edge.v2) {
            ComputeVertexMatrix(adj_vertex, mesh, true);
            unsigned int v1 = std::min(edge.v1, adj_vertex);
            unsigned int v2 = std::max(edge.v1, adj_vertex);

            // Insert those new edges in the set and in the map if they didn't exist before
            AddEdge(mesh, v1, v2);
            EdgeIndex index(v1, v2);
            Eigen::Vector4d optimal_position;
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
