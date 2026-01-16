#pragma once
#include "common.h"
#include <queue>
#include <vector>
#include <algorithm>

class AStar : public ISolver {
protected:
    struct NodeState {
        Cost g = INF;
        NodeID parent = -1;
        bool closed = false;
    };

    struct PQNode {
        Cost f;
        NodeID id;
        bool operator>(const PQNode& other) const { return f > other.f; }
    };

    std::vector<NodeState> states;
    NodeID startNode, goalNode;

public:
    void Initialize(NodeID start, NodeID goal) override {
        startNode = start;
        goalNode = goal;
        metrics.Reset();
        
        int nodeCount = graph->GetNodeCount();
        states.assign(nodeCount, NodeState{});
        states[start].g = 0;
    }

    bool ComputePath() override {
        Timer timer; timer.Start(); // 計測開始

        if (!graph) return false;

        std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> openList;
        
        openList.push({graph->H(startNode, goalNode), startNode});
        metrics.nodesGenerated++;

        bool found = false;

        while (!openList.empty()) {
            PQNode current = openList.top();
            openList.pop();
            NodeID u = current.id;

            if (u == goalNode) {
                metrics.pathCost = states[goalNode].g;
                found = true;
                break;
            }

            if (states[u].closed) continue;
            states[u].closed = true;
            metrics.nodesExpanded++;

            for (const auto& edge : graph->GetSuccessors(u)) {
                NodeID v = edge.target;
                Cost newG = states[u].g + edge.cost;

                if (newG < states[v].g) {
                    states[v].g = newG;
                    states[v].parent = u;
                    Cost f = newG + graph->H(v, goalNode);
                    openList.push({f, v});
                    metrics.nodesGenerated++;
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds(); // 計測終了
        return found;
    }

    std::vector<NodeID> GetPath() const override {
        std::vector<NodeID> path;
        if (states.empty() || states[goalNode].g == INF) return path;

        NodeID curr = goalNode;
        while (curr != -1) {
            path.push_back(curr);
            curr = states[curr].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};