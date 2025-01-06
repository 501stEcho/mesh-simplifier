#include "include.h"
#include "Parser.hpp"
#include "QEMSimplifier.hpp"
#include "Writer.hpp"

unsigned int testId = 0;

void compare_boolean(bool val1, bool val2)
{
    std::cout << "Test " << testId++ << ": ";
    if (val1 == val2) {
        std::cout << "SUCCESS" << std::endl;
    } else {
        std::cout << "FAILED" << std::endl;
        std::cout << "Value 1 : " << std::endl << val1 << std::endl;
        std::cout << "Value 2 : " << std::endl << val2 << std::endl;
    }
}

void compare_vectors(Eigen::Vector4d &vec1, Eigen::Vector4d &vec2)
{
    std::cout << "Test " << testId++ << ": ";
    if (vec1.isApprox(vec2)) {
        std::cout << "SUCCESS" << std::endl;
    } else {
        std::cout << "FAILED" << std::endl;
        std::cout << "Vector 1 : " << std::endl << vec1 << std::endl;
        std::cout << "Vector 2 : " << std::endl << vec2 << std::endl;
    }
}

void compare_matrices(Eigen::Matrix4d &mat1, Eigen::Matrix4d &mat2)
{
    const double epsilon = 1e-9;
    std::cout << "Test " << testId++ << ": ";
    if (mat1.isApprox(mat2)) {
        std::cout << "SUCCESS" << std::endl;
    } else {
        std::cout << "FAILED" << std::endl;
        std::cout << "Matrix 1 : " << std::endl << mat1 << std::endl;
        std::cout << "Matrix 2 : " << std::endl << mat2 << std::endl;
    }
}

void test_matrix_functions()
{
    Eigen::Matrix4d init;
    init << 4.0, -2.0, 1.0, 3.0,
            3.0, 3.0, 1.0, 4.0,
            2.0, -1.0, 4.0, 2.0,
            1.0, 1.0, 2.0, 5.0;
    Eigen::Matrix4d correct;
    correct << 0.143518518518519, 0.199074074074074, 0.046296296296296, -0.263888888888889,
                -0.185185185185185, 0.259259259259259, 0.037037037037037, -0.111111111111111,
                -0.152777777777778, 0.013888888888889, 0.305555555555556, -0.041666666666667,
                0.06944444444444, -0.09722222222222, -0.13888888888888, 0.2916666666667;
    Eigen::Matrix4d inverse = init.inverse();
    compare_matrices(inverse, correct);

    init << 2, 1, 3, 4,
            1, 2, 4, 1,
            3, 4, 1, 2,
            4, 1, 2, 3;
    correct << -0.30701754385964912277, 0.02631578947368421055, -0.043859649122807017553, 0.42982456140350877193,
                0.0263157894736842105, 0.02631578947368421055, 0.28947368421052631578, -0.23684210526315789473,
                -0.04385964912280701753, 0.28947368421052631578, -0.14912280701754385964, 0.06140350877192982456,
                0.42982456140350877192, -0.23684210526315789475, 0.061403508771929824579, -0.20175438596491228071;
    inverse = init.inverse();
    compare_matrices(inverse, correct);
}

void test_triangle_functions()
{
    VertexData v1, v2, v3;
    TriangleData triangle;
    std::vector<VertexData> vertices;
    v1.coordinates = Eigen::Vector3d(1,-34,3);
    v2.coordinates = Eigen::Vector3d(5,19,5);
    v3.coordinates = Eigen::Vector3d(73,20,1);
    triangle.verticesIndex = vec3<unsigned int>(0, 1, 2);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    ComputePlaneEquation(vertices, triangle);

    Eigen::Vector4d correct(107, -76, 1800, -8091);
    correct.normalize();
    compare_vectors(triangle.plane, correct);
}

void test_edge_optimal_location()
{
    VertexData v1, v2, v3;
    TriangleData triangle;
    std::vector<VertexData> vertices;
    Eigen::Vector4d optimal_position;
    v1.coordinates = Eigen::Vector3d(1,-34,3);
    v2.coordinates = Eigen::Vector3d(5,19,5);
    // v1.matrix = Eigen::Matrix4d(4.0, -2.0, 1.0, 3.0, 3.0, 3.0, 1.0, 4.0, 2.0, -1.0, 4.0, 2.0, 1.0, 1.0, 2.0, 5.0);
    // v2.matrix = Eigen::Matrix4d(2, 1, 3, 4, 1, 2, 4, 1, 3, 4, 1, 2, 4, 1, 2, 3);
    Mesh *mesh = new Mesh();
    mesh->vertices.push_back(v1);
    mesh->vertices.push_back(v2);

    EdgeIndex edge(0, 1);
    mesh->edgesMap[0][1] = EdgeData();
    ComputeEdgeOptimalPosition(optimal_position, edge, mesh);
    // std::cout << "Error : " << error << std::endl;

    mesh->vertices[0].matrix = Eigen::Matrix4d();
    mesh->vertices[1].matrix = Eigen::Matrix4d();
    ComputeEdgeOptimalPosition(optimal_position, edge, mesh);
    // std::cout << "Error : " << error << std::endl;
}

void test_mesh_functions()
{
    Mesh *mesh = Mesh::getSingleInstance();
    std::cout << "Test " << testId++ << ": ";

    mesh->vertexNb = 50;
    mesh->triangleNb = 150;

    Mesh *mesh2 = Mesh::getSingleInstance();
    if (mesh->vertexNb != mesh2->vertexNb || mesh->triangleNb != mesh2->triangleNb) {
        std::cout << "FAILED" << std::endl;
        std::cout << "Mesh 1 : " << mesh->vertexNb << " | " << mesh->triangleNb << std::endl;
        std::cout << "Mesh 2 : " << mesh2->vertexNb << " | " << mesh2->triangleNb << std::endl;
    } else
        std::cout << "SUCCESS" << std::endl;
}

int main(int argc, char** argv)
{
    test_matrix_functions();
    test_triangle_functions();
    test_edge_optimal_location();
    test_mesh_functions();
}