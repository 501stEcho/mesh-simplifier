#include "Writer.hpp"
#include <fstream>

void OBJWriter::SaveMesh(std::string path, Mesh *mesh)
{
    std::ofstream outputFile(path);
    std::unordered_map<unsigned int, unsigned int> newVerticesIndex;

    unsigned int nextVertexId = 1;

    for (unsigned int i = 0; i < mesh->verticesBufferSize && nextVertexId < mesh->vertexNb + 1; i++) {
        if (mesh->activeVertices.isActive(i)) {
            VertexData &vertex = mesh->vertices[i];
            outputFile << "v  " << vertex.coordinates(0) << " " << vertex.coordinates(1) << " " << vertex.coordinates(2) << "\n";
            newVerticesIndex[i] = nextVertexId++;
        }
    }

    std::cout << "result" << std::endl;
    std::cout << "number of vertices : " << mesh->vertexNb << std::endl;
    std::cout << "number of triangles : " << mesh->triangleNb << std::endl;

    unsigned int nextTriangleIndex = 0;
    for (unsigned int i = 0; i < mesh->trianglesBufferSize && nextTriangleIndex < mesh->triangleNb; i++) {
        if (mesh->activeTriangles.isActive(i)) {
            TriangleData &triangle = mesh->triangles[i];
            outputFile << "f " << newVerticesIndex[triangle.verticesIndex.x] << " "
                << newVerticesIndex[triangle.verticesIndex.y] << " " << newVerticesIndex[triangle.verticesIndex.z] << "\n";
            nextTriangleIndex++;
        }
    }

    outputFile.close();
}
