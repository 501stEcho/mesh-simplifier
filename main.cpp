
#include "include.h"
#include "Parser.hpp"
#include "QEMSimplifier.hpp"
#include "Writer.hpp"

void print_help(void)
{
    std::cout << "Usage" << std::endl << std::endl;
    std::cout << "  ./mesh-simplifier <path-to-source> <iterations> [path-of-destination]" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        print_help();
        return (1);
    }

    Mesh *mesh = OBJLoader::LoadMesh(argv[1]);
    unsigned int iterations_nb = atol(argv[2]);
    std::string destination(argc > 3 ? argv[3] : "test_meshes/result.obj");

    QEMSimplifier::Simplify(mesh, iterations_nb);
    OBJWriter::SaveMesh(std::string(destination), mesh);
}