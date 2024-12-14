#include "Parser.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <optional>
#include <algorithm>

Mesh *OBJLoader::LoadMesh(std::string path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "Invalid path" << std::endl;
        return nullptr;
    }

    Mesh *LoadedMesh = new Mesh();

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

    file.clear();

    LoadedMesh->vertices = (vec3<float> *)malloc(sizeof(vec3<int>) * LoadedMesh->vertexNb);
    LoadedMesh->triangles = (vec3<unsigned int> *)malloc(sizeof(vec3<int>) * LoadedMesh->triangleNb);

    unsigned int currentVertexIndex = 0;
    unsigned int currentTriangleIndex = 0;
    unsigned int lineNb;

    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string lineType;
        lineStream >> lineType;

        if (lineType.compare("v") == 0) {
            try {
                LoadedMesh->vertices[currentVertexIndex++] = ParseVertexLine(lineStream);
            } catch (const InvalidLineFormatException &exc) {
                std::cerr << "Error in line " << lineNb << " : " << exc.what() << std::endl;
            }
        }
        if (lineType.compare("f") == 0) {
            std::vector<vec3<unsigned int>> triangles;
            try {
                ParseFaceLine(lineStream, triangles);
            } catch (const InvalidLineFormatException &exc) {
                std::cerr << "Error in line " << lineNb << " : " << exc.what() << std::endl;
            }
            for (auto &it : triangles)
                LoadedMesh->triangles[currentTriangleIndex++] = it;
        }
        lineNb++;
    }

    for (auto &it : LoadedMesh->edgesDataMap) {
        for (auto &it2 : it.second) {
            LoadedMesh->edgesDataArray.insert(it2.second);
        }
    }

    std::cout << "Number of triangles : " << LoadedMesh->triangleNb << std::endl;
    std::cout << "Number of edges : " << LoadedMesh->edgeNb << std::endl;
    std::cout << "Number of vertices : " << LoadedMesh->vertexNb << std::endl;

    return LoadedMesh;
}

vec3<float> OBJLoader::ParseVertexLine(std::istringstream &lineStream)
{
    vec3<float> vertexCoordinates;
    unsigned int i = 0;

    if (!(lineStream >> vertexCoordinates.x))
        throw InvalidLineFormatException("");
    if (!(lineStream >> vertexCoordinates.y))
        throw InvalidLineFormatException("");
    if (!(lineStream >> vertexCoordinates.z))
        throw InvalidLineFormatException("");
    
    return vertexCoordinates;
}

void OBJLoader::ParseFaceLine(std::istringstream &lineStream, std::vector<vec3<unsigned int>> &trianglesVertices)
{
    std::vector<unsigned int> faceVerticesIndex;
    std::vector<unsigned int> faceVerticesTextureIndex;
    std::vector<unsigned int> faceVerticesNormalIndex;
    std::string vertexIndexes;

    while (lineStream >> vertexIndexes)
        ExtractIndexes(vertexIndexes, faceVerticesIndex, faceVerticesTextureIndex, faceVerticesNormalIndex);
    
    for (unsigned int i = 1; i + 1 < faceVerticesIndex.size(); i++) {
        trianglesVertices.push_back(vec3<unsigned int>(faceVerticesIndex[0], faceVerticesIndex[i], faceVerticesIndex[i + 1]));
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
            if (numberExtracted) {
                vertexIndex = temp;
                return;
            } else
                throw InvalidLineFormatException("No vertex index detected");
        } else if (!std::isdigit(line[i])) {
            throw InvalidLineFormatException("Unexpected nonnumeric character : \"" + std::to_string(line[i]) + "\"");
        } else {
            numberExtracted = true;
            temp *= 10;
            temp += line[i] - 48;
        }
    }
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

    faceVerticesIndex.push_back(v.value());
    if (vt.has_value())
        faceVerticesTextureIndex.push_back(vt.value());
    if (vn.has_value())
        faceVerticesNormalIndex.push_back(vn.value());
}

void OBJLoader::AddEdge(Mesh *result, unsigned int v1, unsigned int v2)
{
    if (!result)
        throw LoadingFailedException("Loaded mesh pointer is not allocated");

    if (result->edgesDataMap.find(v1) == result->edgesDataMap.end())
        result->edgesDataMap[v1] = std::unordered_map<unsigned int, EdgeData *>();
    if (result->edgesDataMap.find(v2) == result->edgesDataMap.end())
        result->edgesDataMap[v2] = std::unordered_map<unsigned int, EdgeData *>();
    
    EdgeData *newEdge = new EdgeData();

    if (result->edgesDataMap[v1].find(v2) == result->edgesDataMap[v1].end()) {
        result->edgesDataMap[v1][v2] = newEdge;
        result->edgeNb++;
    }
    if (result->edgesDataMap[v2].find(v1) == result->edgesDataMap[v2].end()) {
        result->edgesDataMap[v2][v1] = newEdge;
        result->edgeNb++;
    }
}
