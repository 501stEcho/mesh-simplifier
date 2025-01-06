#include "Tools.hpp"

// Computes the optimal position and returns the error
void ComputeEdgeOptimalPosition(vec4<double> &optimal_position, EdgeIndex &edge, Mesh *mesh)
{
    unsigned int v1 = edge.v1;
    unsigned int v2 = edge.v2;
    mat4<double> quadric = mesh->vertices[v1].matrix + mesh->vertices[v2].matrix;
    mat4<double> temp_position(quadric);

    for (unsigned int j = 0; j < 3; j++)
        temp_position[3][j] = 0;
    temp_position[3][3] = 1;
    mat4<double> inverse;

    std::cout << temp_position << std::endl;
    double det = temp_position.getDeterminant();
    if (fabs(det) >= 1e-6 && temp_position.getInverse(inverse)) {
        std::cout << v1 << "-" << v2 << " invertible" << std::endl;
        optimal_position = inverse * vec4<double>(0,0,0,1);
    } else {
        std::cout << v1 << "-" << v2 << " not invertible" << std::endl;
        optimal_position = vec4<double>(mesh->vertices[v1].coordinates ,1);
        // vec4<double> vertex_1 = vec4<double>(mesh->vertices[v1].coordinates, 1);
        // vec4<double> vertex_2 = vec4<double>(mesh->vertices[v2].coordinates, 1);

        // vec4<double> delta = vertex_2 - vertex_1;
        // vec4<double> qv1_delta = quadric * vertex_1;
        // vec4<double> qv2_delta = quadric * vertex_2;

        // double A = delta.dot(quadric * delta);
        // double B = 2 * (qv1_delta.dot(delta) - qv2_delta.dot(delta));
        // double C = vertex_1.dot(quadric * vertex_1) + vertex_2.dot(quadric * vertex_2);

        // double discriminant = B * B - 4 * A * C;
        // double result_delta = -1;

        // if (A == 0 || discriminant < 0) {
        //     result_delta = 0.5;
        // } else {
        //     double t1 = (-B - sqrt(discriminant)) / (2 * A);
        //     double t2 = (-B + sqrt(discriminant)) / (2 * A);

        //     if (t1 >= 0 && t1 <= 1)
        //         result_delta = t1;
        //     if (t2 >= 0 && t2 <= 1)
        //         result_delta = t2;
            
        //     if (result_delta < 0)
        //         result_delta = 0.5;

        // }
        // std::cout << vertex_1.x <<
        //     " " << result_delta <<
        //     " " << delta.x << std::endl;
        // optimal_position = vertex_1 + result_delta * delta;
    }
    mesh->edgesMap[v1][v2].error = (optimal_position * quadric).dot(optimal_position);
    // std::cout << mesh->edgesMap[v1][v2].error << std::endl;
}

void ComputePlaneEquation(std::vector<VertexData> &vertexArray, TriangleData &triangle)
{
    vec3<double> u = vertexArray[triangle.verticesIndex.x].coordinates - vertexArray[triangle.verticesIndex.z].coordinates;
    vec3<double> v = vertexArray[triangle.verticesIndex.y].coordinates - vertexArray[triangle.verticesIndex.z].coordinates;

    vec3<double> planeNormal = u.cross_product(v);
    if (planeNormal.length_squared() < 1e-9) {
        triangle.plane = vec4<double>(0, 0, 0, 0);
        return;
    }

    double d = -1 * (planeNormal.x * vertexArray[triangle.verticesIndex.x].coordinates.x
        + planeNormal.y * vertexArray[triangle.verticesIndex.x].coordinates.y
        + planeNormal.z * vertexArray[triangle.verticesIndex.x].coordinates.z);
    // planeNormal.normalize();
    
    triangle.plane.x = planeNormal.x;
    triangle.plane.y = planeNormal.y;
    triangle.plane.z = planeNormal.z;
    triangle.plane.w = d;
    triangle.plane.normalize();
}

void ComputeVertexMatrix(unsigned int vertexIndex, Mesh *mesh, bool recomputePlaneEquation)
{
    VertexData &vertex = mesh->vertices[vertexIndex];
    vertex.matrix = mat4<double>();

    for (auto &it : vertex.adjacentTriangles) {
        if (mesh->activeTriangles.isActive(it)) {
            if (recomputePlaneEquation)
                ComputePlaneEquation(mesh->vertices, mesh->triangles[it]);
            TriangleData &triangle = mesh->triangles[it];
            vertex.matrix[0][0] += triangle.plane.x * triangle.plane.x;
            vertex.matrix[0][1] += triangle.plane.x * triangle.plane.y;
            vertex.matrix[0][2] += triangle.plane.x * triangle.plane.z;
            vertex.matrix[0][3] += triangle.plane.x * triangle.plane.w;
            vertex.matrix[1][0] += triangle.plane.y * triangle.plane.x;
            vertex.matrix[1][1] += triangle.plane.y * triangle.plane.y;
            vertex.matrix[1][2] += triangle.plane.y * triangle.plane.z;
            vertex.matrix[1][3] += triangle.plane.y * triangle.plane.w;
            vertex.matrix[2][0] += triangle.plane.z * triangle.plane.x;
            vertex.matrix[2][1] += triangle.plane.z * triangle.plane.y;
            vertex.matrix[2][2] += triangle.plane.z * triangle.plane.z;
            vertex.matrix[2][3] += triangle.plane.z * triangle.plane.w;
            vertex.matrix[3][0] += triangle.plane.w * triangle.plane.x;
            vertex.matrix[3][1] += triangle.plane.w * triangle.plane.y;
            vertex.matrix[3][2] += triangle.plane.w * triangle.plane.z;
            vertex.matrix[3][3] += triangle.plane.w * triangle.plane.w;
        }
    }
}

bool AddEdge(Mesh *mesh, unsigned int v1, unsigned int v2)
{
    unsigned int edge_v1 = std::min(v1, v2);
    unsigned int edge_v2 = std::max(v1, v2);

    if (mesh->edgesMap.find(edge_v1) == mesh->edgesMap.end())
        mesh->edgesMap[edge_v1] = std::unordered_map<unsigned int, EdgeData>();
    
    if (mesh->edgesMap[edge_v1].find(edge_v2) == mesh->edgesMap[edge_v1].end()) {
        mesh->edgesMap[edge_v1][edge_v2] = EdgeData();
        mesh->edgeNb++;
        return true;
    }
    return false;
}
