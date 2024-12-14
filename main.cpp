
#include "include.h"
#include "Parser.hpp"

void print_help(void)
{
    std::cout << "Usage" << std::endl << std::endl;
    std::cout << "  ./mesh-simplifier <path-to-source> [path-of-destination]" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        print_help();
        return (1);
    }

    OBJLoader::LoadMesh(argv[1]);
}