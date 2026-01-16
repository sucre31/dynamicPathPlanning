#pragma once
#include "astar.h"
#include "grid_graph.h"
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>
#include <algorithm>

class MAStar : public AStar {
public:
    struct DSquare {
        int x, y, size, scale;
        std::vector<NodeID> boundaryNodes;
        bool isLeaf = false;
        DSquare* children[4] = {nullptr, nullptr, nullptr, nullptr};
        
        DSquare(int _x, int _y, int _s, int _sc) : x(_x), y(_y), size(_s), scale(_sc) {}
        ~DSquare() { for(int i=0; i<4; ++i) if(children[i]) delete children[i]; }
    };

    struct Beamlet {
        Cost cost;
        std::vector<NodeID> pixels; // 境界間を結ぶ最短路の全ピクセル
    };

private:
    DSquare* root = nullptr;
    // (始点, 終点) -> ビームレット情報
    std::map<std::pair<NodeID, NodeID>, Beamlet> beamletCache;
    std::vector<NodeID> resultPath;

    const GridGraph* GetGrid() const { return static_cast<const GridGraph*>(graph); }

    void GetCoord(NodeID id, int& x, int& y) const {
        int w = GetGrid()->GetWidth();
        x = id % w;
        y = id / w;
    }

    // 1. PFR-RDPの構築 [cite: 157, 281, 976]
    DSquare* BuildPFRRDP(int x, int y, int size, int scale) {
        DSquare* ds = new DSquare(x, y, size, scale);
        int sx, sy, gx, gy;
        GetCoord(startNode, sx, sy);
        GetCoord(goalNode, gx, gy);

        bool hasStart = (sx >= x && sx < x + size && sy >= y && sy < y + size);
        bool hasGoal = (gx >= x && gx < x + size && gy >= y && gy < y + size);

        if ((hasStart || hasGoal) && size > 1) {
            int nSize = size / 2;
            ds->children[0] = BuildPFRRDP(x, y, nSize, scale + 1);
            ds->children[1] = BuildPFRRDP(x + nSize, y, nSize, scale + 1);
            ds->children[2] = BuildPFRRDP(x, y + nSize, nSize, scale + 1);
            ds->children[3] = BuildPFRRDP(x + nSize, y + nSize, nSize, scale + 1);
        } else {
            ds->isLeaf = true;
            // 境界の自由ピクセルを頂点として抽出 [cite: 159, 1020]
            for (int i = 0; i < size; ++i) {
                AddBoundary(x + i, y, ds);              // 上
                AddBoundary(x + i, y + size - 1, ds);   // 下
                if (size > 1) {
                    AddBoundary(x, y + i, ds);          // 左
                    AddBoundary(x + size - 1, y + i, ds); // 右
                }
            }
            if (hasStart) ds->boundaryNodes.push_back(this->startNode);
            if (hasGoal) ds->boundaryNodes.push_back(this->goalNode);
            
            std::sort(ds->boundaryNodes.begin(), ds->boundaryNodes.end());
            ds->boundaryNodes.erase(std::unique(ds->boundaryNodes.begin(), ds->boundaryNodes.end()), ds->boundaryNodes.end());
        }
        return ds;
    }

    void AddBoundary(int x, int y, DSquare* ds) {
        if (x >= 0 && x < GetGrid()->GetWidth() && y >= 0 && y < GetGrid()->GetHeight()) {
            if (!GetGrid()->IsObstacle(x, y)) {
                ds->boundaryNodes.push_back(y * GetGrid()->GetWidth() + x);
            }
        }
    }

    // 2. Bottom-Up Fusion: 正方形内部の最短路（ビームレット）を計算 [cite: 193, 323, 1028]
    void RunBottomUpFusion(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) RunBottomUpFusion(ds->children[i]);
        }

        // 葉ノード内の全境界ノードペアに対しダイクストラを実行
        for (auto u : ds->boundaryNodes) {
            std::priority_queue<std::pair<Cost, NodeID>, std::vector<std::pair<Cost, NodeID>>, std::greater<>> pq;
            std::unordered_map<NodeID, Cost> dists;
            std::unordered_map<NodeID, NodeID> prev;

            dists[u] = 0;
            pq.push({0, u});

            while (!pq.empty()) {
                NodeID curr = pq.top().second;
                Cost d = pq.top().first;
                pq.pop();

                if (d > dists[curr]) continue;

                int cx, cy; GetCoord(curr, cx, cy);
                int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
                for (int i = 0; i < 4; ++i) {
                    int vx = cx + dx[i], vy = cy + dy[i];
                    if (vx < ds->x || vx >= ds->x + ds->size || vy < ds->y || vy >= ds->y + ds->size) continue;
                    if (GetGrid()->IsObstacle(vx, vy)) continue;

                    NodeID v = vy * GetGrid()->GetWidth() + vx;
                    if (dists.find(v) == dists.end() || d + 1.0 < dists[v]) {
                        dists[v] = d + 1.0;
                        prev[v] = curr;
                        pq.push({dists[v], v});
                    }
                }
            }

            for (auto v : ds->boundaryNodes) {
                if (u == v || dists.find(v) == dists.end()) continue;
                Beamlet b;
                b.cost = dists[v];
                NodeID step = v;
                while (step != u) {
                    b.pixels.push_back(step);
                    step = prev[step];
                }
                b.pixels.push_back(u);
                std::reverse(b.pixels.begin(), b.pixels.end());
                beamletCache[{u, v}] = b;
            }
        }
    }

    // 3. 正方形を跨ぐ物理的な接続 (Neighbor Connectivity) [cite: 196, 1032, 1102]
    void ConnectLeavesPhysically(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) ConnectLeavesPhysically(ds->children[i]);
            return;
        }
        int w = GetGrid()->GetWidth();
        int h = GetGrid()->GetHeight();
        for (auto u : ds->boundaryNodes) {
            int ux, uy; GetCoord(u, ux, uy);
            int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i) {
                int vx = ux + dx[i], vy = uy + dy[i];
                if (vx < 0 || vx >= w || vy < 0 || vy >= h) continue;
                if (!GetGrid()->IsObstacle(vx, vy)) {
                    NodeID v = vy * w + vx;
                    // 隣接するセル同士をコスト1.0で接続
                    Beamlet b; b.cost = 1.0; b.pixels = {u, v};
                    beamletCache[{u, v}] = b;
                }
            }
        }
    }

public:
    void Initialize(NodeID start, NodeID goal) override {
        AStar::Initialize(start, goal);
        if (root) { delete root; root = nullptr; }
        beamletCache.clear();
        resultPath.clear();
    }

    std::vector<NodeID> GetPath() const override { return resultPath; }

    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        // ステップ1: 階層分割
        root = BuildPFRRDP(0, 0, GetGrid()->GetWidth(), 0);
        // ステップ2: 正方形内の最短路計算
        RunBottomUpFusion(root);
        // ステップ3: 正方形間の隣接接続
        ConnectLeavesPhysically(root);

        // ステップ4: 構築されたビームレット・グラフ上で探索 [cite: 222, 1062]
        bool success = SearchOnAbstractGraph();
        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        return success;
    }

    bool SearchOnAbstractGraph() {
        using Item = std::pair<Cost, NodeID>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
        std::unordered_map<NodeID, Cost> gScore;
        std::unordered_map<NodeID, NodeID> parent;

        gScore[startNode] = 0;
        open.push({graph->H(startNode, goalNode), startNode});

        while (!open.empty()) {
            NodeID u = open.top().second;
            open.pop();
            metrics.nodesExpanded++;

            if (u == goalNode) {
                // パスの完全復元（ビームレットピクセルを連結）
                resultPath.clear();
                std::vector<NodeID> nodes;
                NodeID curr = goalNode;
                while (curr != startNode) {
                    nodes.push_back(curr);
                    curr = parent[curr];
                }
                nodes.push_back(startNode);
                std::reverse(nodes.begin(), nodes.end());

                for (size_t i = 0; i < nodes.size() - 1; ++i) {
                    auto& b = beamletCache[{nodes[i], nodes[i+1]}];
                    for (size_t j = 0; j < b.pixels.size() - 1; ++j) {
                        resultPath.push_back(b.pixels[j]);
                    }
                }
                resultPath.push_back(goalNode);

                metrics.pathCost = gScore[goalNode];
                if (goalNode < (NodeID)states.size()) states[goalNode].g = gScore[goalNode];
                return true;
            }

            // 抽象グラフ上のエッジ走査
            for (auto const& [key, beamlet] : beamletCache) {
                if (key.first != u) continue;
                NodeID v = key.second;
                Cost tentative_g = gScore[u] + beamlet.cost;
                if (gScore.find(v) == gScore.end() || tentative_g < gScore[v] - EPS) {
                    gScore[v] = tentative_g;
                    parent[v] = u;
                    open.push({tentative_g + graph->H(v, goalNode), v});
                }
            }
        }
        return false;
    }
};