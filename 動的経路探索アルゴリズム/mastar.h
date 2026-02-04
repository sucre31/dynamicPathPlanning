#pragma once
#include "common.h"
#include "grid_graph.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

class MAStar {
public:
    struct Beamlet {
        Cost cost = 0;
        std::vector<NodeID> pixels;
    };

    struct DSquare {
        int x, y;
        int size;
        int level;
        bool isLeaf = false;
        std::vector<NodeID> boundary;
        std::unordered_map<uint64_t, Beamlet> table;
        DSquare* children[4]{nullptr, nullptr, nullptr, nullptr};

        DSquare(int _x, int _y, int _s, int _l)
            : x(_x), y(_y), size(_s), level(_l) {}

        ~DSquare() {
            for (int i = 0; i < 4; ++i)
                if (children[i]) delete children[i];
        }
    };

private:
    const GridGraph* graph = nullptr;
    NodeID startNode = -1;
    NodeID goalNode  = -1;

    DSquare* root = nullptr;

    std::map<std::pair<NodeID, NodeID>, Beamlet> beamlets;
    std::unordered_map<NodeID, std::vector<std::pair<NodeID, Cost>>> adj;
    std::vector<NodeID> path;
    SearchMetrics metrics;
    double preprocessSeconds = 0.0;

public:
    ~MAStar() {
        if (root) delete root;
    }

    void SetGraph(const GridGraph* g) { graph = g; }
    void ResetMetrics() { metrics.Reset(); }
    const SearchMetrics& GetMetrics() const { return metrics; }
    double GetPreprocessSeconds() const { return preprocessSeconds; }

    void Initialize(NodeID s, NodeID t) {
        Initialize(graph, s, t);
    }

    void Initialize(const GridGraph* g, NodeID s, NodeID t) {
        graph = g;
        startNode = s;
        goalNode  = t;
        metrics.Reset();
        preprocessSeconds = 0.0;

        if (root) { delete root; root = nullptr; }
        beamlets.clear();
        adj.clear();
        path.clear();

        Timer prepTimer; prepTimer.Start();
        int size = std::min(graph->GetWidth(), graph->GetHeight());
        root = BuildPFRRDP(0, 0, size, 0);
        BottomUpFusion(root);
        BuildBeamletGraph();
        preprocessSeconds = prepTimer.GetElapsedSeconds();
    }

    bool ComputePath() {
        Timer timer; timer.Start();
        std::unordered_map<NodeID, Cost> gScore;
        std::unordered_map<NodeID, NodeID> parent;

        auto heuristic = [&](NodeID v){
            int x,y,gx,gy;
            GetCoord(v,x,y);
            GetCoord(goalNode,gx,gy);
            return std::abs(x-gx) + std::abs(y-gy);
        };

        using Q = std::pair<Cost, NodeID>;
        std::priority_queue<Q, std::vector<Q>, std::greater<Q>> open;

        gScore[startNode] = 0;
        open.push({heuristic(startNode), startNode});

        while (!open.empty()) {
            NodeID u = open.top().second;
            open.pop();
            metrics.nodesExpanded++;

            if (u == goalNode) break;

            auto itAdj = adj.find(u);
            if (itAdj == adj.end()) continue;
            for (const auto& edge : itAdj->second) {
                NodeID v = edge.first;
                Cost c   = edge.second;

                Cost ng = gScore[u] + c;
                if (!gScore.count(v) || ng < gScore[v]) {
                    gScore[v] = ng;
                    parent[v] = u;
                    open.push({ng + heuristic(v), v});
                    metrics.nodesGenerated++;
                }
            }
        }

        if (!gScore.count(goalNode)) return false;
        metrics.pathCost = gScore[goalNode];
        metrics.runtimeSeconds = timer.GetElapsedSeconds();

        // 抽象パス復元
        std::vector<NodeID> abs;
        for (NodeID v = goalNode; v != startNode; v = parent[v])
            abs.push_back(v);
        abs.push_back(startNode);
        std::reverse(abs.begin(), abs.end());

        // ピクセルパス復元
        for (size_t i = 0; i + 1 < abs.size(); ++i) {
            auto it = beamlets.find({abs[i], abs[i+1]});
            for (size_t j = 0; j + 1 < it->second.pixels.size(); ++j)
                path.push_back(it->second.pixels[j]);
        }
        path.push_back(goalNode);
        return true;
    }

    const std::vector<NodeID>& GetPath() const { return path; }

private:
    // ---------- core algorithms ----------

    DSquare* BuildPFRRDP(int x, int y, int size, int level) {
        DSquare* ds = new DSquare(x, y, size, level);
        ExtractBoundary(ds);
        if (size > 1 && IsNearSG(x, y, size)) {
            int ns = size / 2;
            for (int i = 0; i < 4; ++i) {
                ds->children[i] = BuildPFRRDP(
                    x + (i % 2) * ns,
                    y + (i / 2) * ns,
                    ns,
                    level + 1
                );
            }
            ds->isLeaf = false;
        } else {
            ds->isLeaf = true;
        }
        return ds;
    }

    void BottomUpFusion(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) BottomUpFusion(ds->children[i]);
            ComputeBeamletsByFusion(ds);
        } else {
            ComputeBeamletsInLeaf(ds);
        }
    }

    void ComputeBeamletsInLeaf(DSquare* ds) {
        ds->table.clear();
        for (NodeID u : ds->boundary) {
            std::queue<NodeID> q;
            std::unordered_map<NodeID, Cost> dist;
            std::unordered_map<NodeID, NodeID> prev;

            dist[u] = 0;
            q.push(u);

            while (!q.empty()) {
                NodeID cur = q.front(); q.pop();
                int cx, cy;
                GetCoord(cur, cx, cy);
                int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};
                for (int i = 0; i < 4; ++i) {
                    int nx = cx + dx[i], ny = cy + dy[i];
                    if (nx < ds->x || nx >= ds->x + ds->size ||
                        ny < ds->y || ny >= ds->y + ds->size) continue;
                    if (graph->IsObstacle(nx, ny)) continue;
                    NodeID v = ny * graph->GetWidth() + nx;
                    if (!dist.count(v)) {
                        dist[v] = dist[cur] + 1;
                        prev[v] = cur;
                        q.push(v);
                    }
                }
            }

            for (NodeID v : ds->boundary) {
                if (u == v || !dist.count(v)) continue;
                Beamlet b;
                b.cost = dist[v];
                for (NodeID c = v; c != u; c = prev[c])
                    b.pixels.push_back(c);
                b.pixels.push_back(u);
                std::reverse(b.pixels.begin(), b.pixels.end());
                ds->table[Key(u, v)] = b;
            }
        }
    }

    void ComputeBeamletsByFusion(DSquare* ds) {
        struct FusedEdge {
            NodeID to;
            Cost cost;
            std::vector<NodeID> pixels;
        };

        ds->table.clear();

        std::unordered_set<NodeID> nodeSet;
        for (int i = 0; i < 4; ++i) {
            DSquare* child = ds->children[i];
            if (!child) continue;
            for (NodeID n : child->boundary) nodeSet.insert(n);
        }
        if (nodeSet.empty()) return;

        std::unordered_map<NodeID, std::vector<FusedEdge>> adj;
        for (int i = 0; i < 4; ++i) {
            DSquare* child = ds->children[i];
            if (!child) continue;
            for (const auto& kv : child->table) {
                NodeID a = static_cast<NodeID>(kv.first >> 32);
                NodeID b = static_cast<NodeID>(kv.first & 0xffffffffu);
                const Beamlet& bl = kv.second;
                adj[a].push_back({b, bl.cost, bl.pixels});
            }
        }

        for (NodeID u : nodeSet) {
            int ux, uy; GetCoord(u, ux, uy);
            int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};
            for (int i = 0; i < 4; ++i) {
                int vx = ux + dx[i], vy = uy + dy[i];
                if (vx < ds->x || vx >= ds->x + ds->size ||
                    vy < ds->y || vy >= ds->y + ds->size) continue;
                if (graph->IsObstacle(vx, vy)) continue;
                NodeID v = vy * graph->GetWidth() + vx;
                if (nodeSet.count(v)) {
                    adj[u].push_back({v, 1.0, {u, v}});
                }
            }
        }

        for (NodeID source : ds->boundary) {
            using Q = std::pair<Cost, NodeID>;
            std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
            std::unordered_map<NodeID, Cost> dist;
            struct Prev {
                NodeID prev;
                std::vector<NodeID> edgePixels;
            };
            std::unordered_map<NodeID, Prev> prev;

            dist[source] = 0;
            pq.push({0, source});

            while (!pq.empty()) {
                auto [d, u] = pq.top(); pq.pop();
                if (d > dist[u]) continue;
                auto it = adj.find(u);
                if (it == adj.end()) continue;
                for (const auto& e : it->second) {
                    Cost nd = d + e.cost;
                    if (!dist.count(e.to) || nd < dist[e.to]) {
                        dist[e.to] = nd;
                        prev[e.to] = {u, e.pixels};
                        pq.push({nd, e.to});
                    }
                }
            }

            for (NodeID target : ds->boundary) {
                if (source == target) continue;
                if (!dist.count(target)) continue;
                std::vector<std::vector<NodeID>> segments;
                NodeID cur = target;
                while (cur != source) {
                    auto pit = prev.find(cur);
                    if (pit == prev.end()) break;
                    segments.push_back(pit->second.edgePixels);
                    cur = pit->second.prev;
                }
                if (cur != source) continue;
                std::reverse(segments.begin(), segments.end());
                Beamlet b;
                b.cost = dist[target];
                for (const auto& seg : segments) {
                    if (seg.empty()) continue;
                    if (b.pixels.empty()) {
                        b.pixels.insert(b.pixels.end(), seg.begin(), seg.end());
                    } else {
                        b.pixels.insert(b.pixels.end(), seg.begin() + 1, seg.end());
                    }
                }
                ds->table[Key(source, target)] = b;
            }
        }
    }

    // ---------- helpers ----------

    uint64_t Key(NodeID a, NodeID b) const {
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    }

    void CollectDSquares(DSquare* ds, std::vector<DSquare*>& out) {
        if (!ds) return;
        out.push_back(ds);
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) CollectDSquares(ds->children[i], out);
        }
    }

    void BuildBeamletGraph() {
        std::vector<DSquare*> nodes;
        CollectDSquares(root, nodes);

        std::unordered_set<NodeID> boundarySet;
        for (auto* ds : nodes) {
            for (NodeID v : ds->boundary) boundarySet.insert(v);
        }

        adj.clear();
        for (auto* ds : nodes) {
            for (const auto& kv : ds->table) {
                NodeID a = static_cast<NodeID>(kv.first >> 32);
                NodeID b = static_cast<NodeID>(kv.first & 0xffffffffu);
                const Beamlet& bl = kv.second;
                auto key = std::make_pair(a, b);
                auto it = beamlets.find(key);
                if (it == beamlets.end() || bl.cost < it->second.cost) {
                    beamlets[key] = bl;
                }
            }
        }

        for (NodeID u : boundarySet) {
            int ux, uy; GetCoord(u, ux, uy);
            int dx[4]={1,-1,0,0}, dy[4]={0,0,1,-1};
            for (int i = 0; i < 4; ++i) {
                int vx = ux + dx[i], vy = uy + dy[i];
                if (vx < 0 || vx >= graph->GetWidth() ||
                    vy < 0 || vy >= graph->GetHeight()) continue;
                if (graph->IsObstacle(vx, vy)) continue;
                NodeID v = vy * graph->GetWidth() + vx;
                if (!boundarySet.count(v)) continue;
                Beamlet b;
                b.cost = 1.0;
                b.pixels = {u, v};
                auto key = std::make_pair(u, v);
                auto it = beamlets.find(key);
                if (it == beamlets.end() || b.cost < it->second.cost) {
                    beamlets[key] = b;
                }
            }
        }

        // build adjacency list for faster A*
        for (const auto& kv : beamlets) {
            adj[kv.first.first].push_back({kv.first.second, kv.second.cost});
        }
    }

    void ExtractBoundary(DSquare* ds) {
        ds->boundary.clear();
        for (int i = 0; i < ds->size; ++i) {
            AddIfFree(ds->x + i, ds->y, ds);
            AddIfFree(ds->x + i, ds->y + ds->size - 1, ds);
            if (ds->size > 1) {
                AddIfFree(ds->x, ds->y + i, ds);
                AddIfFree(ds->x + ds->size - 1, ds->y + i, ds);
            }
        }
        std::sort(ds->boundary.begin(), ds->boundary.end());
        ds->boundary.erase(std::unique(ds->boundary.begin(), ds->boundary.end()),
                           ds->boundary.end());
    }

    void AddIfFree(int x, int y, DSquare* ds) {
        if (x >= 0 && x < graph->GetWidth() &&
            y >= 0 && y < graph->GetHeight() &&
            !graph->IsObstacle(x, y))
            ds->boundary.push_back(y * graph->GetWidth() + x);
    }

    bool IsNearSG(int x, int y, int size) const {
        int sx,sy,gx,gy;
        GetCoord(startNode, sx, sy);
        GetCoord(goalNode,  gx, gy);
        return (sx>=x && sx<x+size && sy>=y && sy<y+size) ||
               (gx>=x && gx<x+size && gy>=y && gy<y+size);
    }

    void GetCoord(NodeID id, int& x, int& y) const {
        int w = graph->GetWidth();
        x = id % w;
        y = id / w;
    }
};
