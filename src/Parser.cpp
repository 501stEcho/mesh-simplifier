#include <fstream>
#include <iostream>
#include <sstream>
#include <optional>
#include <algorithm>
#include "Parser.hpp"

Mesh *OBJLoader::LoadMesh(std::string path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "Invalid path" << std::endl;
        return nullptr;
    }

    Mesh *LoadedMesh = Mesh::getSingleInstance();

    AllocateMeshData(LoadedMesh, file);
    FillMeshData(LoadedMesh, file);

    std::cout << "Number of triangles : " << LoadedMesh->triangleNb << std::endl;
    std::cout << "Number of edges : " << LoadedMesh->edgeNb << std::endl;
    std::cout << "Number of vertices : " << LoadedMesh->vertexNb << std::endl;

    LoadedMesh->verticesBufferSize = LoadedMesh->vertexNb;
    LoadedMesh->trianglesBufferSize = LoadedMesh->triangleNb;

    return LoadedMesh;
}

void OBJLoader::AllocateMeshData(Mesh *LoadedMesh, std::ifstream &file)
{
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string lineType;
        lineStream >> lineType;

        if (lineType.compare("v") == 0)
            LoadedMesh->vertexNb++;
        if (lineType.compare("f") == 0)
            CountTriangleAndEdges(lineStream, LoadedMesh);
    }

    LoadedMesh->vertices = std::vector<VertexData>(LoadedMesh->vertexNb);
    LoadedMesh->triangles = std::vector<TriangleData>(LoadedMesh->triangleNb);
}

void OBJLoader::FillMeshData(Mesh *LoadedMesh, std::ifstream &file)
{
    std::string line;
    unsigned int currentVertexIndex = 0;
    unsigned int currentTriangleIndex = 0;
    unsigned int lineNb;

    file.clear();
    file.seekg(0);

    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string lineType;
        lineStream >> lineType;

        if (lineType.compare("v") == 0) {
            try {
                ParseVertexLine(lineStream, LoadedMesh, currentVertexIndex);
            } catch (const InvalidLineFormatException &exc) {
                std::cerr << "Error in line " << lineNb << " : " << exc.what() << std::endl;
            }
        }
        if (lineType.compare("f") == 0) {
            std::vector<vec3<unsigned int>> triangles;
            try {
                ParseFaceLine(lineStream, LoadedMesh, currentTriangleIndex);
            } catch (const InvalidLineFormatException &exc) {
                std::cerr << "Error in line " << lineNb << " : " << exc.what() << std::endl;
            }
        }
        lineNb++;
    }

    for (unsigned int i = 0; i < LoadedMesh->vertexNb; i++) {
        ComputeVertexMatrix(i, LoadedMesh);
    }
}

void OBJLoader::ParseVertexLine(std::istringstream &lineStream, Mesh *result, unsigned int &availableVertexIndex)
{
    unsigned int i = 0;

    if (!(lineStream >> result->vertices[availableVertexIndex].coordinates.x))
        throw InvalidLineFormatException("");
    if (!(lineStream >> result->vertices[availableVertexIndex].coordinates.y))
        throw InvalidLineFormatException("");
    if (!(lineStream >> result->vertices[availableVertexIndex].coordinates.z))
        throw InvalidLineFormatException("");

    // std::cout << result->vertices[availableVertexIndex].coordinates.x << " " << result->vertices[availableVertexIndex].coordinates.y << " " << result->vertices[availableVertexIndex].coordinates.z << std::endl;
    result->vertices[availableVertexIndex++].isValid = true;
}

void OBJLoader::ParseFaceLine(std::istringstream &lineStream, Mesh *result, unsigned int &availableIndex)
{
    std::vector<unsigned int> faceVerticesIndex;
    std::vector<unsigned int> faceVerticesTextureIndex;
    std::vector<unsigned int> faceVerticesNormalIndex;
    std::string vertexIndexes;

    while (lineStream >> vertexIndexes)
        ExtractIndexes(vertexIndexes, faceVerticesIndex, faceVerticesTextureIndex, faceVerticesNormalIndex);
    
    for (unsigned int i = 1; i + 1 < faceVerticesIndex.size(); i++) {
        result->triangles[availableIndex] = TriangleData();
        result->triangles[availableIndex].verticesIndex.x = faceVerticesIndex[0];
        result->triangles[availableIndex].verticesIndex.y = faceVerticesIndex[i];
        result->triangles[availableIndex].verticesIndex.z = faceVerticesIndex[i + 1];
        ComputePlaneEquation(result->vertices, result->triangles[availableIndex]);
        result->triangles[availableIndex].isValid = true;

        result->vertices[faceVerticesIndex[0]].adjacentTriangles.insert(availableIndex);
        result->vertices[faceVerticesIndex[0]].adjacentVertices.insert(faceVerticesIndex[i]);
        result->vertices[faceVerticesIndex[0]].adjacentVertices.insert(faceVerticesIndex[i + 1]);

        result->vertices[faceVerticesIndex[i]].adjacentTriangles.insert(availableIndex);
        result->vertices[faceVerticesIndex[i]].adjacentVertices.insert(faceVerticesIndex[0]);
        result->vertices[faceVerticesIndex[i]].adjacentVertices.insert(faceVerticesIndex[i + 1]);

        result->vertices[faceVerticesIndex[i + 1]].adjacentTriangles.insert(availableIndex);
        result->vertices[faceVerticesIndex[i + 1]].adjacentVertices.insert(faceVerticesIndex[0]);
        result->vertices[faceVerticesIndex[i + 1]].adjacentVertices.insert(faceVerticesIndex[i]);
        
        availableIndex++;
    }
}

void OBJLoader::CountTriangleAndEdges(std::istringstream &lineStream, Mesh *result)
{
    std::string buffer;
    unsigned int edgeNb;
    std::vector<unsigned int> verticesIndex;

    while (lineStream >> buffer) {
        unsigned int temp;
        ExtractFirstIndex(buffer, temp);
        verticesIndex.push_back(temp);
    }
    
    for (unsigned int i = 1; i + 1 < verticesIndex.size(); i++) {
        result->triangleNb++;
        AddEdge(result, verticesIndex[0], verticesIndex[i]);
        AddEdge(result, verticesIndex[i], verticesIndex[i + 1]);
        AddEdge(result, verticesIndex[i + 1], verticesIndex[0]);
    }
}

void OBJLoader::ExtractFirstIndex(std::string line, unsigned int &vertexIndex)
{
    bool numberExtracted = false;
    unsigned int temp = 0;
    unsigned int step = 0;
    std::optional<unsigned int> v;
    for (unsigned int i = 0; line[i]; i++) {
        if (line[i] == '/') {
            break;
        } else if (!std::isdigit(line[i])) {
            throw InvalidLineFormatException("Unexpected nonnumeric character : \"" + std::to_string(line[i]) + "\"");
        } else {
            numberExtracted = true;
            temp *= 10;
            temp += line[i] - 48;
        }
    }
    if (numberExtracted) {
        vertexIndex = temp - 1;
    } else
        throw InvalidLineFormatException("No vertex index detected");
}

void OBJLoader::ExtractIndexes(std::string line, std::vector<unsigned int> &faceVerticesIndex,
    std::vector<unsigned int> &faceVerticesTextureIndex, std::vector<unsigned int> &faceVerticesNormalIndex)
{
    bool numberExtracted = false;
    unsigned int temp = 0;
    unsigned int step = 0;
    std::optional<unsigned int> v;
    std::optional<unsigned int> vt;
    std::optional<unsigned int> vn;
    for (unsigned int i = 0; line[i]; i++) {
        if (line[i] == '/') {
            switch (step) {
                case (0):
                    if (numberExtracted)
                        v = temp;
                    else
                        throw InvalidLineFormatException("No vertex index detected");
                    break;
                case (1):
                    if (numberExtracted)
                        vt = temp;
                    break;
                case (2):
                    if (numberExtracted)
                        vn = temp;
                    break;
                default:
                    throw InvalidLineFormatException("Too many separators");
            }
            numberExtracted = false;
            temp = 0;
            step++;
        } else if (!std::isdigit(line[i])) {
            throw InvalidLineFormatException("Unexpected nonnumeric character : \"" + std::to_string(line[i]) + "\"");
        } else {
            numberExtracted = true;
            temp *= 10;
            temp += line[i] - 48;
        }
    }

    if (!v.has_value() && numberExtracted)
        v = temp;

    faceVerticesIndex.push_back(v.value() - 1);
    if (vt.has_value())
        faceVerticesTextureIndex.push_back(vt.value() - 1);
    if (vn.has_value())
        faceVerticesNormalIndex.push_back(vn.value() - 1);
}

