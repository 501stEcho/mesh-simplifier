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

    AllocateMeshData(LoadedMesh, file);
    FillMeshData(LoadedMesh, file);

    std::cout << "Number of triangles : " << LoadedMesh->triangleNb << std::endl;
    std::cout << "Number of edges : " << LoadedMesh->edgeNb << std::endl;
    std::cout << "Number of vertices : " << LoadedMesh->vertexNb << std::endl;

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

    file.clear();

    LoadedMesh->vertices = (VertexData *)malloc(sizeof(VertexData) * LoadedMesh->vertexNb);
    LoadedMesh->triangles = (TriangleData *)malloc(sizeof(TriangleData) * LoadedMesh->triangleNb);
}

void OBJLoader::FillMeshData(Mesh *LoadedMesh, std::ifstream &file)
{
    std::string line;
    unsigned int currentVertexIndex = 0;
    unsigned int currentTriangleIndex = 0;
    unsigned int lineNb;

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
        for (auto &it : LoadedMesh->vertices[i].adjacentTriangles) {
            TriangleData &triangle = LoadedMesh->triangles[it];
            mat4<double> fundamental_error_quadric;
            fundamental_error_quadric[0][0] = triangle.plane.x * triangle.plane.x;
            fundamental_error_quadric[0][1] = triangle.plane.x * triangle.plane.y;
            fundamental_error_quadric[0][2] = triangle.plane.x * triangle.plane.z;
            fundamental_error_quadric[0][3] = triangle.plane.x * triangle.plane.w;
            fundamental_error_quadric[1][0] = triangle.plane.y * triangle.plane.x;
            fundamental_error_quadric[1][1] = triangle.plane.y * triangle.plane.y;
            fundamental_error_quadric[1][2] = triangle.plane.y * triangle.plane.z;
            fundamental_error_quadric[1][3] = triangle.plane.y * triangle.plane.w;
            fundamental_error_quadric[2][0] = triangle.plane.z * triangle.plane.x;
            fundamental_error_quadric[2][1] = triangle.plane.z * triangle.plane.y;
            fundamental_error_quadric[2][2] = triangle.plane.z * triangle.plane.z;
            fundamental_error_quadric[2][3] = triangle.plane.z * triangle.plane.w;
            fundamental_error_quadric[3][0] = triangle.plane.w * triangle.plane.x;
            fundamental_error_quadric[3][1] = triangle.plane.w * triangle.plane.y;
            fundamental_error_quadric[3][2] = triangle.plane.w * triangle.plane.z;
            fundamental_error_quadric[3][3] = triangle.plane.w * triangle.plane.w;
        }
    }

    for (auto &it : LoadedMesh->edgesMap) {
        for (auto &it2 : it.second) {
            mat4<double> quadric = LoadedMesh->vertices[it2.second->v1].matrix + LoadedMesh->vertices[it2.second->v2].matrix;
            mat4<double> temp_position(quadric);

            for (unsigned int j = 0; j < 3; j++)
                temp_position[3][j] = 0;
            temp_position[3][3] = 1;
            double deter = temp_position.getDeterminant();

            vec4<double> optimal_position;
            if (deter == 0) {
                // Let's do that later
            } else {
                mat4<double> inverse;
                temp_position.inverse(inverse);
                optimal_position = inverse * vec4<double>(0,0,0,1);
            }
            it2.second->error = (optimal_position * quadric).dot(optimal_position);
            LoadedMesh->edgesPriorityQueue.push(it2.second);
        }
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
        result->triangles[availableIndex].verticesIndex.x = faceVerticesIndex[0];
        result->triangles[availableIndex].verticesIndex.y = faceVerticesIndex[i];
        result->triangles[availableIndex].verticesIndex.z = faceVerticesIndex[i + 1];
        result->triangles[availableIndex].plane = ComputePlaneEquation(result->vertices, result->triangles[availableIndex].verticesIndex);
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
    
    unsigned int edge_v1 = std::min(v1, v2);
    unsigned int edge_v2 = std::max(v1, v2);

    if (result->edgesMap.find(edge_v1) == result->edgesMap.end())
        result->edgesMap[edge_v1] = std::unordered_map<unsigned int, std::shared_ptr<Edge>>();
    
    if (result->edgesMap[edge_v1].find(edge_v2) == result->edgesMap[edge_v1].end()) {
        result->edgesMap[edge_v1][edge_v2] = std::make_shared<Edge>();
        result->edgeNb++;
    }
}

vec4<double> OBJLoader::ComputePlaneEquation(VertexData *vertexArray, vec3<unsigned int> &verticesIndex)
{
    vec3<double> u = vertexArray[verticesIndex.x].coordinates - vertexArray[verticesIndex.z].coordinates;
    vec3<double> v = vertexArray[verticesIndex.y].coordinates - vertexArray[verticesIndex.z].coordinates;

    vec3<double> planeNormal = u.cross_product(v);
    double d = -1 * (planeNormal.x * vertexArray[verticesIndex.x].coordinates.x
        + planeNormal.y * vertexArray[verticesIndex.x].coordinates.y
        + planeNormal.z * vertexArray[verticesIndex.x].coordinates.z);
    return vec4<double>(planeNormal.x, planeNormal.y, planeNormal.z, d);
}
