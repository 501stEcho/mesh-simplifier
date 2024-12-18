#pragma once

#include <string>
#include <vector>
#include "Types.hpp"

class InvalidLineFormatException : public std::exception
{
    public:
        InvalidLineFormatException(const std::string msg) : _msg(msg) {};
        const char *what() const noexcept override
        {
            return _msg.c_str();
        }
    
    private:
        std::string _msg;
};

class LoadingFailedException : public std::exception {
    public:
        LoadingFailedException(const std::string msg) : _msg(msg) {};
        const char *what() const noexcept override
        {
            return _msg.c_str();
        }
    
    private:
        std::string _msg; 
};

class OBJLoader
{
    public:
        static Mesh *LoadMesh(std::string path);
    
    protected:
        static void AllocateMeshData(Mesh *result, std::ifstream &file);
        static void FillMeshData(Mesh *result, std::ifstream &file);
        static void ParseVertexLine(std::istringstream &lineStream, Mesh *result, unsigned int &availableVertexIndex);
        static void ParseFaceLine(std::istringstream &lineStream, Mesh *result, unsigned int &availableIndex);
        static void CountTriangleAndEdges(std::istringstream &lineStream, Mesh *result);
    
    private:
        static void ExtractFirstIndex(std::string line, unsigned int &vertexIndex);
        static void ExtractIndexes(std::string line, std::vector<unsigned int> &faceVerticesIndex,
            std::vector<unsigned int> &faceVerticesTextureIndex, std::vector<unsigned int> &faceVerticesNormalIndex);
        static void AddEdge(Mesh *result, unsigned int v1, unsigned int v2);
        static vec4<double> ComputePlaneEquation(VertexData *vertexArray, vec3<unsigned int> &verticesIndex);
};
