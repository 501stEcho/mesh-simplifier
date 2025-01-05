#include "QEMSimplifier.hpp"
#include "Tools.hpp"
#include <iomanip>

std::set<Edge *, CompareEdgePtr> sortedEdges;
std::unordered_map<unsigned int, unsigned int> parent;

void QEMSimplifier::Simplify(Mesh *mesh, unsigned int iterationsNb)
{
    GetSortedEdgeQueue(mesh, sortedEdges);


    for (unsigned int iteration = 0; iteration < iterationsNb && mesh->vertexNb > 1;) {
        unsigned int v1_parent = FindLatestParentVertex((*sortedEdges.begin())->v1, parent);
        unsigned int v2_parent = FindLatestParentVertex((*sortedEdges.begin())->v2, parent);
        Edge *temp_edge = *sortedEdges.begin();
        sortedEdges.erase(sortedEdges.begin());

        if (!temp_edge->isValid || v1_parent == v2_parent || !mesh->activeVertices.isActive(v1_parent) || !mesh->activeVertices.isActive(v2_parent))
            continue;

        unsigned int v1 = std::min(v1_parent, v2_parent);
        unsigned int v2 = std::max(v1_parent, v2_parent);
        Edge &edge = mesh->edgesMap[v1][v2];
        std::cout << "Iteration " << iteration << " : " << v1 << "-" << v2 << " (" << edge.error << ")" << std::endl;

        if (mesh->edgesMap[v1].find(v2) == mesh->edgesMap[v1].end())
            std::cerr << "Could not find " << v1 << "-" << v2 << std::endl;
    
        mat4<double> quadric = mesh->vertices[v1].matrix + mesh->vertices[v2].matrix;
        vec4<double> optimal_position;
        ComputeEdgeOptimalPosition(optimal_position, edge, mesh);

        mesh->vertices[v1].coordinates.x = optimal_position.x;
        mesh->vertices[v1].coordinates.y = optimal_position.y;
        mesh->vertices[v1].coordinates.z = optimal_position.z;

        for (auto &it : mesh->vertices[v2].adjacentTriangles) {
            if (mesh->activeTriangles.isActive(it))
               UpdateAdjacentTriangle(it, edge, mesh, optimal_position);
        }

        UpdateAdjacentVertices(edge, mesh, sortedEdges);
        DeleteVertex(v2 , mesh);
        mesh->edgesMap[v1].erase(v2);
        parent[v2] = v1;
        iteration++;
    }
}
 
void QEMSimplifier::GetSortedEdgeQueue(Mesh *mesh, std::set<Edge *, CompareEdgePtr> &result)
{
    for (auto &it : mesh->edgesMap) {
        for (auto &it2 : it.second) {
            mat4<double> quadric = mesh->vertices[it2.second.v1].matrix + mesh->vertices[it2.second.v2].matrix;
            vec4<double> optimal_position;
            it2.second.error = ComputeEdgeOptimalPosition(optimal_position, it2.second, mesh);
            // std::cout << "Inserting " << it2.second.v1 << "-"
            // << it2.second.v2 << " : "
            // << it2.second.error << std::endl;
            result.insert(&it2.second);
            // std::cout << "---------------------" << std::endl;
        }
    }
}

void QEMSimplifier::UpdateAdjacentTriangle(unsigned int it, Edge &edge, Mesh *mesh, vec4<double> new_position)
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

void QEMSimplifier::UpdateAdjacentVertices(Edge &edge, Mesh *mesh, std::set<Edge *, CompareEdgePtr> &sortedEdge)
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

            Edge *adj_edge = &mesh->edgesMap[v1][v2];
            auto it = sortedEdge.find(adj_edge);
            if (it != sortedEdge.end()) {
                sortedEdge.erase(it);
            } else {
                // adj_edge->isValid = false;
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

            Edge *adj_edge = &mesh->edgesMap[v1][v2];
            auto it = sortedEdge.find(adj_edge);
            if (it != sortedEdge.end()) {
                sortedEdge.erase(it);
                mesh->edgesMap[v1].erase(v2);
            } else {
                // adj_edge->isValid = false;
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
            Edge *new_edge = &mesh->edgesMap[v1][v2];
            std::cout << "Edge " << new_edge->v1 << "-" << new_edge->v2 << " : " << std::endl;
            new_edge->isValid = true;
            vec4<double> optimal_position;
            new_edge->error = ComputeEdgeOptimalPosition(optimal_position, *new_edge, mesh);
            sortedEdge.insert(new_edge);
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
