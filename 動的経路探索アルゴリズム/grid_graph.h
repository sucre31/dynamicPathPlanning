#pragma once
#include "common.h"
#include <vector>

class GridGraph : public IGraph {
private:
    int width, height;
    std::vector<bool> obstacles;

public:
    GridGraph(int w, int h) : width(w), height(h), obstacles(w * h, false) {}

    void SetObstacle(int x, int y, bool blocked) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            obstacles[y * width + x] = blocked;
        }
    }
    
    bool IsObstacle(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return obstacles[y * width + x];
        }
        return true;
    }

    int GetNodeCount() const override { return width * height; }
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }

    std::vector<Edge> GetSuccessors(NodeID u) const override {
        std::vector<Edge> successors;
        int x = u % width;
        int y = u / width;
        const int dx[] = {0, 0, 1, -1};
        const int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int nextId = ny * width + nx;
                if (!obstacles[nextId]) {
                    successors.push_back({nextId, 1.0});
                }
            }
        }
        return successors;
    }

    // ★追加: 無向グラフ(グリッド)なのでSuccessorsと同じ
    // 有向グラフの場合は「自分へ入ってくるエッジ」を返す必要がある
    std::vector<Edge> GetPredecessors(NodeID u) const override {
        std::vector<Edge> predecessors;
        // Successorsと同じロジック
        int x = u % width;
        int y = u / width;
        const int dx[] = {0, 0, 1, -1};
        const int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int prevId = ny * width + nx;
                // 注意: 「自分(u)が障害物でない」なら入ってこれるはずだが、
                // ここでは「相手(prevId)が障害物でない」ならエッジがあるとする
                if (!obstacles[prevId]) {
                    // Edge.target はここでは「接続先(source)」の意味で使う
                    predecessors.push_back({prevId, 1.0});
                }
            }
        }
        return predecessors;
    }

    // ヒューリスティック
    Cost H(NodeID from, NodeID to) const override {
        int x1 = from % width, y1 = from / width;
        int x2 = to % width, y2 = to / width;
        return static_cast<Cost>(std::abs(x1 - x2) + std::abs(y1 - y2));
    }
};