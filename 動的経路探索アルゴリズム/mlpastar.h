#pragma once
#include "lpastar.h"
#include "mastar.h"
#include <map>
#include <unordered_set>
#include <unordered_map>

class MLPAStar : public LPAStar {
    using DSquare = MAStar::DSquare;
    using Beamlet = MAStar::Beamlet;

    DSquare* root = nullptr;
    // 抽象グラフのエッジ: (始点, 終点) -> ビームレット情報 [cite: 188, 549]
    std::map<std::pair<NodeID, NodeID>, Beamlet> beamletCache;
    std::vector<NodeID> fullPixelPath;
    
    // LPA*ロジックで使用するための抽象隣接リスト 
    std::unordered_map<NodeID, std::vector<std::pair<NodeID, Cost>>> adj;
    std::unordered_map<NodeID, std::vector<std::pair<NodeID, Cost>>> revAdj;
    std::unordered_set<NodeID> affectedNodes;

public:
    ~MLPAStar() { if (root) delete root; }

    void Initialize(NodeID start, NodeID goal) override {
        // メモリ確保等は基底クラスで行う [cite: 350]
        LPAStar::Initialize(start, goal);
        this->startNode = start;
        this->goalNode = goal;

        if (root) { delete root; root = nullptr; }
        beamletCache.clear();
        affectedNodes.clear();
        adj.clear();
        revAdj.clear();

        // 1. PFR-RDPの構築 [cite: 157, 388]
        int mapSize = std::min(GetGrid()->GetWidth(), GetGrid()->GetHeight());
        root = BuildInitialPFRRDP(0, 0, mapSize, 0);
        
        // 2. ボトムアップ・フージョン [cite: 193, 389]
        RunFullBottomUpFusion(root);
        
        // 抽象グラフとLPA*の状態を同期
        SyncWithLPA();
    }

    // 障害物変化時の動的再計画 (Algorithm 3) [cite: 393, 1143, 1144]
    void NotifyObstacleChange(NodeID node, bool blocked) override {
        int vx, vy;
        GetCoord(node, vx, vy);

        // 階層の更新と再分割 [cite: 245, 394]
        InvalidateAffectedHierarchy(root, vx, vy);
        RefineTree(root, vx, vy); 
        UpdateLocalFusion(root, vx, vy);

        // 変化をLPA*へ通知 [cite: 319, 395]
        SyncWithLPA();
    }

    // ビームレット・グラフ上での増分探索の実行 
    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        while (!openList.empty()) {
            Key topKey = openList.begin()->first;
            Key goalKey = CalculateKey(goalNode);

            if (topKey >= goalKey && std::abs(states[goalNode].rhs - states[goalNode].g) < EPS) {
                break;
            }

            NodeID u = openList.begin()->second;
            openList.erase(openList.begin());
            inOpenList[u] = false;
            metrics.nodesExpanded++;

            if (states[u].g > states[u].rhs) {
                states[u].g = states[u].rhs;
                // 抽象グラフ上の後継ノードを更新
                if (adj.count(u)) {
                    for (const auto& edge : adj.at(u)) {
                        NodeID s = edge.first;
                        Cost c = edge.second;
                        if (states[s].rhs > states[u].g + c) {
                            states[s].rhs = states[u].g + c;
                            states[s].parent = u;
                            UpdateVertex(s);
                        }
                    }
                }
            } else {
                states[u].g = INF;
                UpdateVertex(u); 
                // 親がuである抽象後継ノードを再計算
                if (adj.count(u)) {
                    for (const auto& edge : adj.at(u)) {
                        NodeID s = edge.first;
                        if (states[s].parent == u) {
                            RecalculateRhs(s);
                            UpdateVertex(s);
                        }
                    }
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        metrics.pathCost = states[goalNode].g;
        
        if (states[goalNode].g < INF) {
            ReconstructFullPixelPath();
            return true;
        }
        return false;
    }

    std::vector<NodeID> GetPath() const override { return fullPixelPath; }

protected:
    // 抽象グラフを用いてrhsを再計算するようにオーバーライド
    void RecalculateRhs(NodeID u) {
        if (u == startNode) {
            states[u].rhs = 0;
            states[u].parent = -1;
            return;
        }

        Cost minRhs = INF;
        NodeID bestP = -1;

        if (revAdj.count(u)) {
            for (const auto& edge : revAdj.at(u)) {
                NodeID pred = edge.first;
                Cost c = edge.second;
                double cost = (states[pred].g == INF) ? INF : (states[pred].g + c);
                
                if (cost < minRhs) {
                    minRhs = cost;
                    bestP = pred;
                }
            }
        }
        states[u].rhs = minRhs;
        states[u].parent = bestP;
    }

private:
    const GridGraph* GetGrid() const { return static_cast<const GridGraph*>(graph); }

    void GetCoord(NodeID id, int& x, int& y) const {
        int w = GetGrid()->GetWidth();
        x = id % w; y = id / w;
    }

    DSquare* BuildInitialPFRRDP(int x, int y, int size, int scale) {
        DSquare* ds = new DSquare(x, y, size, scale);
        if (IsNearSG(x, y, size) && size > 1) {
            int ns = size / 2;
            for(int i=0; i<4; ++i)
                ds->children[i] = BuildInitialPFRRDP(x + (i%2)*ns, y + (i/2)*ns, ns, scale + 1);
        } else {
            ds->isLeaf = true;
            ExtractBoundary(ds);
        }
        return ds;
    }

    void RefineTree(DSquare* ds, int vx, int vy) {
        if (!ds || ds->size <= 1) return;
        if (vx >= ds->x && vx < ds->x + ds->size && vy >= ds->y && vy < ds->y + ds->size) {
            if (ds->isLeaf) {
                // 分割前に古い抽象エッジを削除 [cite: 315]
                for (auto u : ds->boundaryNodes)
                    for (auto v : ds->boundaryNodes)
                        if (beamletCache.erase({u, v})) affectedNodes.insert(v);

                ds->isLeaf = false;
                int ns = ds->size / 2;
                for(int i=0; i<4; ++i) {
                    ds->children[i] = new DSquare(ds->x + (i%2)*ns, ds->y + (i/2)*ns, ns, ds->scale + 1);
                    ds->children[i]->isLeaf = true;
                    ExtractBoundary(ds->children[i]);
                }
            }
            for(int i=0; i<4; ++i) RefineTree(ds->children[i], vx, vy);
        }
    }

    void UpdateLocalFusion(DSquare* ds, int vx, int vy) {
        if (!ds) return;
        if (vx >= ds->x && vx < ds->x + ds->size && vy >= ds->y && vy < ds->y + ds->size) {
            if (!ds->isLeaf) for(int i=0; i<4; ++i) UpdateLocalFusion(ds->children[i], vx, vy);
            ComputeBeamletsInDSquare(ds);
        }
    }

    void ComputeBeamletsInDSquare(DSquare* ds) {
        if (!ds->isLeaf) return;
        for (auto u : ds->boundaryNodes) {
            std::priority_queue<std::pair<Cost, NodeID>, std::vector<std::pair<Cost, NodeID>>, std::greater<>> pq;
            std::unordered_map<NodeID, Cost> d;
            std::unordered_map<NodeID, NodeID> p;
            d[u] = 0; pq.push({0, u});

            while(!pq.empty()){
                NodeID curr = pq.top().second; Cost dist = pq.top().first; pq.pop();
                if(dist > d[curr]) continue;
                int cx, cy; GetCoord(curr, cx, cy);
                int dx[]={1,-1,0,0}, dy[]={0,0,1,-1};
                for(int i=0; i<4; ++i){
                    int nx = cx+dx[i], ny = cy+dy[i];
                    if(nx < ds->x || nx >= ds->x+ds->size || ny < ds->y || ny >= ds->y+ds->size) continue;
                    if(GetGrid()->IsObstacle(nx, ny)) continue;
                    NodeID v = ny * GetGrid()->GetWidth() + nx;
                    if(d.find(v) == d.end() || dist + 1.0 < d[v]){
                        d[v] = dist + 1.0; p[v] = curr; pq.push({d[v], v});
                    }
                }
            }
            for (auto v : ds->boundaryNodes) {
                if (u == v) continue;
                if (d.count(v)) {
                    Beamlet b; b.cost = d[v];
                    NodeID step = v;
                    while(step != u) { b.pixels.push_back(step); step = p[step]; }
                    b.pixels.push_back(u); std::reverse(b.pixels.begin(), b.pixels.end());
                    beamletCache[{u, v}] = b;
                } else {
                    beamletCache.erase({u, v});
                }
                affectedNodes.insert(v);
            }
        }
    }

    void SyncWithLPA() {
        // 正方形間の隣接エッジを追加 [cite: 197, 946]
        AddInterSquareEdges(root);
        
        // 抽象隣接リストの再構築
        adj.clear();
        revAdj.clear();
        for (auto const& [key, b] : beamletCache) {
            if (b.cost < INF) {
                adj[key.first].push_back({key.second, b.cost});
                revAdj[key.second].push_back({key.first, b.cost});
            }
        }

        // 影響を受けたノードのrhsを更新 [cite: 1146-1150]
        for (NodeID v : affectedNodes) {
            RecalculateRhs(v);
            UpdateVertex(v);
        }
        affectedNodes.clear();
    }

    void AddInterSquareEdges(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) AddInterSquareEdges(ds->children[i]);
            return;
        }
        int w = GetGrid()->GetWidth();
        for (auto u : ds->boundaryNodes) {
            int ux, uy; GetCoord(u, ux, uy);
            int dx[]={1,-1,0,0}, dy[]={0,0,1,-1};
            for(int i=0; i<4; ++i){
                int vx = ux+dx[i], vy = uy+dy[i];
                if(vx < 0 || vx >= w || vy < 0 || vy >= GetGrid()->GetHeight()) continue;
                if(!GetGrid()->IsObstacle(vx, vy)){
                    NodeID v = vy * w + vx;
                    if (beamletCache.find({u, v}) == beamletCache.end()) {
                        beamletCache[{u, v}] = {1.0, {u, v}};
                        affectedNodes.insert(v);
                    }
                }
            }
        }
    }

    void InvalidateAffectedHierarchy(DSquare* ds, int vx, int vy) {
        if (!ds) return;
        if (vx >= ds->x && vx < ds->x + ds->size && vy >= ds->y && vy < ds->y + ds->size) {
            for (auto u : ds->boundaryNodes)
                for (auto v : ds->boundaryNodes)
                    if (beamletCache.erase({u, v})) affectedNodes.insert(v);
            if (!ds->isLeaf) for (int i = 0; i < 4; ++i) InvalidateAffectedHierarchy(ds->children[i], vx, vy);
        }
    }

    void ReconstructFullPixelPath() {
        fullPixelPath.clear();
        if (states[goalNode].g >= INF) return;

        std::vector<NodeID> abstractPath;
        NodeID curr = goalNode;
        while (curr != startNode && curr != -1) {
            abstractPath.push_back(curr);
            NodeID next = states[curr].parent;
            if (next == -1 || next == curr) break;
            curr = next;
        }
        if (curr != startNode) return;
        abstractPath.push_back(startNode);
        std::reverse(abstractPath.begin(), abstractPath.end());

        for (size_t i = 0; i < abstractPath.size() - 1; ++i) {
            auto it = beamletCache.find({abstractPath[i], abstractPath[i+1]});
            if (it == beamletCache.end() || it->second.cost >= INF) return;
            for (size_t j = 0; j < it->second.pixels.size() - 1; ++j)
                fullPixelPath.push_back(it->second.pixels[j]);
        }
        fullPixelPath.push_back(goalNode);
    }

    bool IsNearSG(int x, int y, int size) {
        int sx, sy, gx, gy;
        GetCoord(startNode, sx, sy); GetCoord(goalNode, gx, gy);
        return (sx >= x && sx < x + size && sy >= y && sy < y + size) ||
               (gx >= x && gx < x + size && gy >= y && gy < y + size);
    }

    void ExtractBoundary(DSquare* ds) {
        ds->boundaryNodes.clear();
        for (int i = 0; i < ds->size; ++i) {
            AddIfFree(ds->x + i, ds->y, ds);
            AddIfFree(ds->x + i, ds->y + ds->size - 1, ds);
            if (ds->size > 1) {
                AddIfFree(ds->x, ds->y + i, ds);
                AddIfFree(ds->x + ds->size - 1, ds->y + i, ds);
            }
        }
        if (IsNearSG(ds->x, ds->y, ds->size)) {
            ds->boundaryNodes.push_back(this->startNode);
            ds->boundaryNodes.push_back(this->goalNode);
        }
        std::sort(ds->boundaryNodes.begin(), ds->boundaryNodes.end());
        ds->boundaryNodes.erase(std::unique(ds->boundaryNodes.begin(), ds->boundaryNodes.end()), ds->boundaryNodes.end());
    }

    void AddIfFree(int x, int y, DSquare* ds) {
        if (x >= 0 && x < GetGrid()->GetWidth() && y >= 0 && y < GetGrid()->GetHeight())
            if (!GetGrid()->IsObstacle(x, y)) ds->boundaryNodes.push_back(y * GetGrid()->GetWidth() + x);
    }

    void RunFullBottomUpFusion(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) for (int i = 0; i < 4; ++i) RunFullBottomUpFusion(ds->children[i]);
        ComputeBeamletsInDSquare(ds);
    }
};