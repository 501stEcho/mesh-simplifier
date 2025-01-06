#include "Tools.hpp"

// Computes the optimal position and returns the error
void ComputeEdgeOptimalPosition(Eigen::Vector4d &optimal_position, EdgeIndex &edge, Mesh *mesh)
{
    unsigned int v1 = edge.v1;
    unsigned int v2 = edge.v2;
    Eigen::Matrix4d quadric = mesh->vertices[v1].matrix + mesh->vertices[v2].matrix;
    Eigen::Matrix4d temp_position(quadric);

    for (unsigned int j = 0; j < 3; j++)
        temp_position(3, j) = 0;
    temp_position(3, 3) = 1;
    Eigen::Matrix4d inverse = temp_position.inverse();

    // std::cout << mesh->vertices[v1].matrix << std::endl;
    double det = temp_position.determinant();
    if (fabs(det) >= 1e-6) {
        // std::cout << v1 << "-" << v2 << " invertible" << std::endl;
        optimal_position = inverse * Eigen::Vector4d(0,0,0,1);
    } else {
        // std::cout << v1 << "-" << v2 << " not invertible" << std::endl;
        Eigen::Vector4d vertex_1 = mesh->vertices[v1].coordinates.homogeneous();
        Eigen::Vector4d vertex_2 = mesh->vertices[v2].coordinates.homogeneous();

        Eigen::Vector4d delta = vertex_2 - vertex_1;
        Eigen::Vector4d qv1_delta = quadric * vertex_1;
        Eigen::Vector4d qv2_delta = quadric * vertex_2;

        double A = delta.dot(quadric * delta);
        double B = 2 * (qv1_delta.dot(delta) - qv2_delta.dot(delta));
        double C = vertex_1.dot(quadric * vertex_1) + vertex_2.dot(quadric * vertex_2);

        double discriminant = B * B - 4 * A * C;
        double result_delta = -1;

        if (A == 0 || discriminant < 0) {
            result_delta = 0.5;
        } else {
            double t1 = (-B - sqrt(discriminant)) / (2 * A);
            double t2 = (-B + sqrt(discriminant)) / (2 * A);

            if (t1 >= 0 && t1 <= 1)
                result_delta = t1;
            if (t2 >= 0 && t2 <= 1)
                result_delta = t2;
            
            if (result_delta < 0)
                result_delta = 0.5;
        }
        optimal_position = vertex_1 + result_delta * delta;
    }
    // std::cout << "optimal : " << optimal_position << std::endl;
    mesh->edgesMap[v1][v2].error = optimal_position.transpose() * quadric * optimal_position;
    // std::cout << "error : " << mesh->edgesMap[v1][v2].error << std::endl;
}

void ComputePlaneEquation(std::vector<VertexData> &vertexArray, TriangleData &triangle)
{
    Eigen::Vector3d u = vertexArray[triangle.verticesIndex.x].coordinates - vertexArray[triangle.verticesIndex.z].coordinates;
    Eigen::Vector3d v = vertexArray[triangle.verticesIndex.y].coordinates - vertexArray[triangle.verticesIndex.z].coordinates;

    Eigen::Vector3d planeNormal = u.cross(v);
    if (planeNormal.isZero(1e-10)) {
        triangle.plane = Eigen::Vector4d::Zero();
        return;
    }

    double d = -1 * (planeNormal(0) * vertexArray[triangle.verticesIndex.x].coordinates(0)
        + planeNormal(1) * vertexArray[triangle.verticesIndex.x].coordinates(1)
        + planeNormal(2) * vertexArray[triangle.verticesIndex.x].coordinates(2));

    triangle.plane.head<3>() = planeNormal;
    triangle.plane(3) = d;
    triangle.plane.normalize();
}

void ComputeVertexMatrix(unsigned int vertexIndex, Mesh *mesh, bool recomputePlaneEquation)
{
    VertexData &vertex = mesh->vertices[vertexIndex];
    vertex.matrix = Eigen::Matrix4d::Zero();
    Eigen::Vector4d coord = vertex.coordinates.homogeneous();

    unsigned int iter = 0;
    for (auto &it : vertex.adjacentTriangles) {
        if (mesh->activeTriangles.isActive(it)) {
            if (recomputePlaneEquation)
                ComputePlaneEquation(mesh->vertices, mesh->triangles[it]);
            TriangleData &triangle = mesh->triangles[it];
            vertex.matrix(0, 0) += triangle.plane(0) * triangle.plane(0);
            vertex.matrix(0, 1) += triangle.plane(0) * triangle.plane(1);
            vertex.matrix(0, 2) += triangle.plane(0) * triangle.plane(2);
            vertex.matrix(0, 3) += triangle.plane(0) * triangle.plane(3);

            vertex.matrix(1, 0) += triangle.plane(1) * triangle.plane(0);
            vertex.matrix(1, 1) += triangle.plane(1) * triangle.plane(1);
            vertex.matrix(1, 2) += triangle.plane(1) * triangle.plane(2);
            vertex.matrix(1, 3) += triangle.plane(1) * triangle.plane(3);

            vertex.matrix(2, 0) += triangle.plane(2) * triangle.plane(0);
            vertex.matrix(2, 1) += triangle.plane(2) * triangle.plane(1);
            vertex.matrix(2, 2) += triangle.plane(2) * triangle.plane(2);
            vertex.matrix(2, 3) += triangle.plane(2) * triangle.plane(3);

            vertex.matrix(3, 0) += triangle.plane(3) * triangle.plane(0);
            vertex.matrix(3, 1) += triangle.plane(3) * triangle.plane(1);
            vertex.matrix(3, 2) += triangle.plane(3) * triangle.plane(2);
            vertex.matrix(3, 3) += triangle.plane(3) * triangle.plane(3);
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
