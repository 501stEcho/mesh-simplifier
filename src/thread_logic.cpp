#include "thread_logic.hpp"

void insertSetElement(std::set<EdgeIndex> &sortedEdges, EdgeIndex &&index, std::mutex &setMutex)
{
    std::lock_guard<std::mutex> lock(setMutex);
    sortedEdges.insert(index);
}

void AddEdgeMapChunk(Mesh *mesh, unsigned int threadIndex, unsigned int threadNb,
    std::set<EdgeIndex> &sortedEdges, std::mutex &setMutex)
{
    auto it = mesh->edgesMap.begin();
    std::advance(it, threadIndex);

    while (it != mesh->edgesMap.end()) {
        for (auto &it2 : it->second) {
            EdgeIndex index(it->first, it2.first);
            Eigen::Matrix4d quadric(mesh->vertices[index.v1].matrix + mesh->vertices[index.v2].matrix);
            Eigen::Vector4d optimal_position;
            ComputeEdgeOptimalPosition(optimal_position, index, mesh);
            insertSetElement(sortedEdges, std::move(index), setMutex);
        }

        for (unsigned int i = 0; i < threadNb; i++) {
            it++;
            if (it == mesh->edgesMap.end())
                break;
        }
    }
}
