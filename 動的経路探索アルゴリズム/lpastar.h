#pragma once
#include "common.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

// LPA* implementation (4-neighborhood) using an indexed binary heap.
class LPAStar : public ISolver {
private:
    struct Key {
        double k1; // min(g, rhs) + h
        double k2; // min(g, rhs)

        bool operator<(const Key& other) const {
            if (std::abs(k1 - other.k1) > EPS) return k1 < other.k1;
            return k2 < other.k2 - EPS;
        }
        bool operator>(const Key& other) const {
            if (std::abs(k1 - other.k1) > EPS) return k1 > other.k1;
            return k2 > other.k2 + EPS;
        }
        bool operator<=(const Key& other) const { return !(*this > other); }
        bool operator>=(const Key& other) const { return !(*this < other); }
    };

    struct NodeData {
        double g = INF;
        double rhs = INF;
        double h = 0.0;
        int parent = -1;
        int heapIndex = -1;
    };

    class IndexedHeap {
        struct HeapItem { Key key; int u; };
        std::vector<HeapItem> heap;
        std::vector<NodeData>* nodesPtr = nullptr;

        void Swap(int i, int j) {
            std::swap(heap[i], heap[j]);
            (*nodesPtr)[heap[i].u].heapIndex = i;
            (*nodesPtr)[heap[j].u].heapIndex = j;
        }

        void UpHeap(int i) {
            while (i > 0) {
                int p = (i - 1) / 2;
                if (heap[i].key < heap[p].key) {
                    Swap(i, p);
                    i = p;
                } else break;
            }
        }

        void DownHeap(int i) {
            int n = (int)heap.size();
            while (true) {
                int l = 2 * i + 1;
                int r = 2 * i + 2;
                int s = i;
                if (l < n && heap[l].key < heap[s].key) s = l;
                if (r < n && heap[r].key < heap[s].key) s = r;
                if (s != i) { Swap(i, s); i = s; }
                else break;
            }
        }

    public:
        void Initialize(std::vector<NodeData>* ptr) {
            nodesPtr = ptr;
            heap.clear();
            heap.reserve(ptr->size() / 4);
        }

        bool Empty() const { return heap.empty(); }

        std::pair<Key, int> Top() const { return {heap[0].key, heap[0].u}; }

        void Pop() {
            if (heap.empty()) return;
            int last = (int)heap.size() - 1;
            (*nodesPtr)[heap[0].u].heapIndex = -1;
            if (last > 0) {
                heap[0] = heap[last];
                (*nodesPtr)[heap[0].u].heapIndex = 0;
                heap.pop_back();
                DownHeap(0);
            } else {
                heap.pop_back();
            }
        }

        void InsertOrUpdate(int u, const Key& k) {
            int idx = (*nodesPtr)[u].heapIndex;
            if (idx == -1) {
                idx = (int)heap.size();
                heap.push_back({k, u});
                (*nodesPtr)[u].heapIndex = idx;
                UpHeap(idx);
            } else {
                Key oldKey = heap[idx].key;
                heap[idx].key = k;
                if (k < oldKey) UpHeap(idx);
                else DownHeap(idx);
            }
        }

        void Remove(int u) {
            int idx = (*nodesPtr)[u].heapIndex;
            if (idx == -1) return;
            int last = (int)heap.size() - 1;
            (*nodesPtr)[u].heapIndex = -1;
            if (idx == last) {
                heap.pop_back();
                return;
            }
            heap[idx] = heap[last];
            (*nodesPtr)[heap[idx].u].heapIndex = idx;
            heap.pop_back();
            UpHeap(idx);
            DownHeap(idx);
        }
    };

    std::vector<NodeData> nodes;
    IndexedHeap openList;
    int startNode = -1;
    int goalNode = -1;

    inline Key CalculateKey(int u) const {
        double m = std::min(nodes[u].g, nodes[u].rhs);
        return { m + nodes[u].h, m };
    }

    void UpdateVertex(int u) {
        if (u != startNode) {
            double minRhs = INF;
            int bestP = -1;
            for (const auto& edge : graph->GetPredecessors(u)) {
                int pred = edge.target;
                double gpred = nodes[pred].g;
                if (gpred == INF) continue;
                double cand = gpred + edge.cost;
                if (cand < minRhs) {
                    minRhs = cand;
                    bestP = pred;
                }
            }
            nodes[u].rhs = minRhs;
            nodes[u].parent = bestP;
        }

        if (std::abs(nodes[u].g - nodes[u].rhs) > EPS) {
            openList.InsertOrUpdate(u, CalculateKey(u));
        } else {
            openList.Remove(u);
        }
    }

public:
    void Initialize(int start, int goal) override {
        startNode = start;
        goalNode = goal;
        metrics.Reset();

        int nodeCount = graph->GetNodeCount();
        if ((int)nodes.size() != nodeCount) nodes.assign(nodeCount, NodeData{});
        for (int i = 0; i < nodeCount; ++i) {
            nodes[i].g = INF;
            nodes[i].rhs = INF;
            nodes[i].parent = -1;
            nodes[i].heapIndex = -1;
            nodes[i].h = graph->H(i, goalNode);
        }

        openList.Initialize(&nodes);
        nodes[startNode].rhs = 0.0;
        openList.InsertOrUpdate(startNode, CalculateKey(startNode));
    }

    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        while (!openList.Empty()) {
            auto [topKey, u] = openList.Top();
            Key goalKey = CalculateKey(goalNode);
            if (!(topKey < goalKey) && std::abs(nodes[goalNode].rhs - nodes[goalNode].g) < EPS) {
                break;
            }

            openList.Pop();
            metrics.nodesExpanded++;

            if (nodes[u].g > nodes[u].rhs) {
                nodes[u].g = nodes[u].rhs;
                for (const auto& edge : graph->GetSuccessors(u)) {
                    UpdateVertex(edge.target);
                    metrics.nodesGenerated++;
                }
            } else {
                nodes[u].g = INF;
                UpdateVertex(u);
                for (const auto& edge : graph->GetSuccessors(u)) {
                    UpdateVertex(edge.target);
                    metrics.nodesGenerated++;
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        metrics.pathCost = nodes[goalNode].g;
        return nodes[goalNode].g != INF;
    }

    void NotifyObstacleChange(int u, bool /*isBlocked*/) {
        UpdateVertex(u);
        for (const auto& edge : graph->GetSuccessors(u)) {
            UpdateVertex(edge.target);
        }
        for (const auto& edge : graph->GetPredecessors(u)) {
            UpdateVertex(edge.target);
        }
    }

    std::vector<int> GetPath() const override {
        std::vector<int> path;
        if (nodes[goalNode].g == INF) return path;

        int curr = goalNode;
        path.push_back(curr);
        int maxStep = (int)nodes.size();
        int step = 0;
        while (curr != startNode && step++ < maxStep) {
            int p = nodes[curr].parent;
            if (p == -1 || p == curr) break;
            curr = p;
            path.push_back(curr);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};
