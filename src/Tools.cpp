#include "Tools.hpp"

void compare_optimal_position(Eigen::Vector4d &test_position, Eigen::Matrix4d quadric,
    double &current_min, Eigen::Vector4d &optimal_position)
{
    double test_error = test_position.transpose() * quadric * test_position;
    if (test_error < current_min)
    {
        optimal_position = test_position;
        current_min = test_error;
    }
}

// Computes the optimal position and returns the error
void ComputeEdgeOptimalPosition(Eigen::Vector4d &optimal_position, EdgeIndex &edge, Mesh *mesh)
{
    unsigned int v1 = edge.v1;
    unsigned int v2 = edge.v2;
    Eigen::Matrix4d quadric = mesh->vertices[v1].matrix + mesh->vertices[v2].matrix;
    Eigen::Matrix4d temp_position(quadric);

    temp_position.row(3) << 0, 0, 0, 1;
    Eigen::Matrix4d inverse = temp_position.inverse();

    double det = temp_position.determinant();
    if (fabs(det) >= 1e-6) {
        optimal_position = inverse * Eigen::Vector4d(0,0,0,1);
    } else {
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

        if (!(A == 0 || discriminant < 0)) {
            double t1 = (-B - sqrt(discriminant)) / (2 * A);
            double t2 = (-B + sqrt(discriminant)) / (2 * A);

            if (t1 >= 0 && t1 <= 1)
                result_delta = t1;
            if (t2 >= 0 && t2 <= 1)
                result_delta = t2;
        }

        if (result_delta < 0) {
            double current_min = vertex_1.transpose() * quadric * vertex_1;
            compare_optimal_position(vertex_2, quadric, current_min, optimal_position);
            Eigen::Vector4d halfway = vertex_1 + 0.5 * delta;
            compare_optimal_position(halfway, quadric, current_min, optimal_position);
        } else {
            optimal_position = vertex_1 + result_delta * delta;
        }
    }
    mesh->edgesMap[v1][v2].error = optimal_position.transpose() * quadric * optimal_position;
}

void ComputePlaneEquation(std::vector<VertexData> &vertexArray, TriangleData &triangle)
{
    Eigen::Vector3d u = vertexArray[triangle.verticesIndex.y].coordinates - vertexArray[triangle.verticesIndex.x].coordinates;
    Eigen::Vector3d v = vertexArray[triangle.verticesIndex.z].coordinates - vertexArray[triangle.verticesIndex.x].coordinates;

    Eigen::Vector3d normal = u.cross(v);
    if (normal.norm() < 1e-9) {
        triangle.plane = Eigen::Vector4d::Zero();
        return;
    }

    double d = -normal.dot(vertexArray[triangle.verticesIndex.x].coordinates);
    triangle.plane = Eigen::Vector4d(normal[0], normal[1], normal[2], d);
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
            Eigen::Matrix4d triangleQuadric = triangle.plane * triangle.plane.transpose();
            vertex.matrix += triangleQuadric;
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
