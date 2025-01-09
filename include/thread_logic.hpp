#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <set>
#include "Types.hpp"
#include "Tools.hpp"

void insertSetElement(std::set<EdgeIndex> &sortedEdges, EdgeIndex &&index, std::mutex &setMutex);
void AddEdgeMapChunk(Mesh *mesh, unsigned int threadIndex, unsigned int threadNb,
    std::set<EdgeIndex> &sortedEdges, std::mutex &setMutex);
void CleanupEdgeSet(std::set<EdgeIndex> &sortedEdges, std::mutex &setMutex, std::atomic<bool> &running);
