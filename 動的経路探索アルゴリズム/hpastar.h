#pragma once
#include "common.h"
#include "grid_graph.h"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <algorithm>

// HPA* (Hierarchical Path-Finding A*) implementation based on Botea et al.
// One abstract level (clusters + entrances), with optional path refinement.
class HPAStar : public ISolver {
    struct Cluster {
        int id;
        int x, y, w, h;
        std::vector<NodeID> nodes; // abstract nodes in this cluster
    };

    struct Edge {
        NodeID to;
        Cost cost;
    };

    const GridGraph* GetGrid() const { return static_cast<const GridGraph*>(graph); }

    int clusterSize = 10;
    int entranceThreshold = 6;
    int width = 0;
    int height = 0;

    std::vector<Cluster> clusters;
    std::unordered_map<int, int> cellToCluster; // cell -> cluster id

    std::unordered_map<NodeID, std::vector<Edge>> adj; // abstract graph
    std::unordered_map<uint64_t, std::vector<NodeID>> edgePaths; // for refinement
    std::unordered_set<NodeID> abstractNodes;

    NodeID startNode = -1;
    NodeID goalNode = -1;
    std::vector<NodeID> finalPath;

public:
    HPAStar(int cs = 10, int entranceW = 6) : clusterSize(cs), entranceThreshold(entranceW) {}

    void Initialize(NodeID start, NodeID goal) override {
        startNode = start;
        goalNode = goal;
        finalPath.clear();
        metrics.Reset();

        width = GetGrid()->GetWidth();
        height = GetGrid()->GetHeight();

        BuildClusters();
        BuildEntrancesAndGraph();
        BuildIntraEdges();
        InsertNode(startNode);
        InsertNode(goalNode);
    }

    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        std::unordered_map<NodeID, Cost> g;
        std::unordered_map<NodeID, NodeID> parent;
        struct QItem {
            Cost f;
            NodeID node;
            bool operator>(const QItem& other) const { return f > other.f; }
        };
        std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

        for (NodeID n : abstractNodes) g[n] = INF;
        if (!abstractNodes.count(startNode) || !abstractNodes.count(goalNode)) return false;

        g[startNode] = 0.0;
        open.push({Heuristic(startNode, goalNode), startNode});

        while (!open.empty()) {
            NodeID u = open.top().node;
            open.pop();
            metrics.nodesExpanded++;

            if (u == goalNode) break;
            auto it = adj.find(u);
            if (it == adj.end()) continue;
            for (const auto& e : it->second) {
                metrics.nodesGenerated++;
                Cost ng = g[u] + e.cost;
                if (ng + EPS < g[e.to]) {
                    g[e.to] = ng;
                    parent[e.to] = u;
                    open.push({ng + Heuristic(e.to, goalNode), e.to});
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        metrics.pathCost = g.count(goalNode) ? g[goalNode] : INF;
        if (!g.count(goalNode) || g[goalNode] >= INF) return false;

        ReconstructPath(parent);
        return true;
    }

    std::vector<NodeID> GetPath() const override { return finalPath; }

private:
    uint64_t Key(NodeID a, NodeID b) const {
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    }

    Cost Heuristic(NodeID a, NodeID b) const {
        return GetGrid()->H(a, b);
    }

    void BuildClusters() {
        clusters.clear();
        cellToCluster.clear();
        int id = 0;
        for (int y = 0; y < height; y += clusterSize) {
            for (int x = 0; x < width; x += clusterSize) {
                Cluster c;
                c.id = id++;
                c.x = x;
                c.y = y;
                c.w = std::min(clusterSize, width - x);
                c.h = std::min(clusterSize, height - y);
                clusters.push_back(c);
                for (int yy = c.y; yy < c.y + c.h; ++yy) {
                    for (int xx = c.x; xx < c.x + c.w; ++xx) {
                        cellToCluster[yy * width + xx] = c.id;
                    }
                }
            }
        }
    }

    bool InCluster(const Cluster& c, int x, int y) const {
        return x >= c.x && x < c.x + c.w && y >= c.y && y < c.y + c.h;
    }

    void BuildEntrancesAndGraph() {
        adj.clear();
        edgePaths.clear();
        abstractNodes.clear();
        for (auto& c : clusters) c.nodes.clear();

        int clustersPerRow = (width + clusterSize - 1) / clusterSize;
        int clustersPerCol = (height + clusterSize - 1) / clusterSize;

        for (const auto& c : clusters) {
            int cx = c.id % clustersPerRow;
            int cy = c.id / clustersPerRow;

            if (cx + 1 < clustersPerRow) {
                const Cluster& right = clusters[c.id + 1];
                BuildEntrancesBetween(c, right, true);
            }
            if (cy + 1 < clustersPerCol) {
                int downId = c.id + clustersPerRow;
                if (downId >= 0 && downId < (int)clusters.size()) {
                    const Cluster& down = clusters[downId];
                    BuildEntrancesBetween(c, down, false);
                }
            }
        }
    }

    void BuildEntrancesBetween(const Cluster& c1, const Cluster& c2, bool horizontal) {
        // horizontal: c2 is to the right of c1, border is vertical
        if (horizontal) {
            int x1 = c1.x + c1.w - 1;
            int x2 = c2.x;
            int yStart = std::max(c1.y, c2.y);
            int yEnd = std::min(c1.y + c1.h, c2.y + c2.h);
            BuildEntrancesOnBorder(c1, c2, x1, x2, yStart, yEnd, true);
        } else {
            int y1 = c1.y + c1.h - 1;
            int y2 = c2.y;
            int xStart = std::max(c1.x, c2.x);
            int xEnd = std::min(c1.x + c1.w, c2.x + c2.w);
            BuildEntrancesOnBorder(c1, c2, y1, y2, xStart, xEnd, false);
        }
    }

    void BuildEntrancesOnBorder(const Cluster& c1, const Cluster& c2,
                                int fixed1, int fixed2, int start, int end,
                                bool verticalBorder) {
        // verticalBorder: border varies along y, fixed x on each side
        int i = start;
        while (i < end) {
            int len = 0;
            while (i + len < end) {
                int x1 = verticalBorder ? fixed1 : (i + len);
                int y1 = verticalBorder ? (i + len) : fixed1;
                int x2 = verticalBorder ? fixed2 : (i + len);
                int y2 = verticalBorder ? (i + len) : fixed2;
                if (GetGrid()->IsObstacle(x1, y1) || GetGrid()->IsObstacle(x2, y2)) break;
                len++;
            }
            if (len > 0) {
                if (len < entranceThreshold) {
                    int mid = i + len / 2;
                    AddTransitionPair(c1, c2, verticalBorder, fixed1, fixed2, mid);
                } else {
                    int a = i;
                    int b = i + len - 1;
                    AddTransitionPair(c1, c2, verticalBorder, fixed1, fixed2, a);
                    AddTransitionPair(c1, c2, verticalBorder, fixed1, fixed2, b);
                }
                i += len;
            } else {
                i++;
            }
        }
    }

    void AddTransitionPair(const Cluster& c1, const Cluster& c2, bool verticalBorder,
                           int fixed1, int fixed2, int var) {
        int x1 = verticalBorder ? fixed1 : var;
        int y1 = verticalBorder ? var : fixed1;
        int x2 = verticalBorder ? fixed2 : var;
        int y2 = verticalBorder ? var : fixed2;
        NodeID n1 = y1 * width + x1;
        NodeID n2 = y2 * width + x2;
        AddAbstractNode(n1, c1.id);
        AddAbstractNode(n2, c2.id);
        AddEdge(n1, n2, 1.0, {n1, n2});
        AddEdge(n2, n1, 1.0, {n2, n1});
    }

    void AddAbstractNode(NodeID n, int clusterId) {
        if (!abstractNodes.count(n)) {
            abstractNodes.insert(n);
        }
        clusters[clusterId].nodes.push_back(n);
    }

    void AddEdge(NodeID a, NodeID b, Cost cost, const std::vector<NodeID>& path) {
        adj[a].push_back({b, cost});
        edgePaths[Key(a, b)] = path;
    }

    void BuildIntraEdges() {
        for (const auto& c : clusters) {
            if (c.nodes.empty()) continue;
            for (NodeID n : c.nodes) {
                auto distPrev = BFSWithinCluster(c, n);
                const auto& dist = distPrev.first;
                const auto& prev = distPrev.second;
                for (NodeID m : c.nodes) {
                    if (n == m) continue;
                    auto it = dist.find(m);
                    if (it == dist.end()) continue;
                    std::vector<NodeID> path = ReconstructLocalPath(n, m, prev);
                    AddEdge(n, m, it->second, path);
                }
            }
        }
    }

    std::pair<std::unordered_map<NodeID, Cost>, std::unordered_map<NodeID, NodeID>>
    BFSWithinCluster(const Cluster& c, NodeID start) {
        std::queue<NodeID> q;
        std::unordered_map<NodeID, Cost> dist;
        std::unordered_map<NodeID, NodeID> prev;
        dist[start] = 0;
        q.push(start);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        while (!q.empty()) {
            NodeID cur = q.front(); q.pop();
            int cx = cur % width;
            int cy = cur / width;
            for (int i = 0; i < 4; ++i) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];
                if (!InCluster(c, nx, ny)) continue;
                if (GetGrid()->IsObstacle(nx, ny)) continue;
                NodeID v = ny * width + nx;
                if (!dist.count(v)) {
                    dist[v] = dist[cur] + 1;
                    prev[v] = cur;
                    q.push(v);
                }
            }
        }
        return {dist, prev};
    }

    std::vector<NodeID> ReconstructLocalPath(NodeID start, NodeID goal,
                                             const std::unordered_map<NodeID, NodeID>& prev) {
        std::vector<NodeID> path;
        NodeID cur = goal;
        while (cur != start) {
            path.push_back(cur);
            auto it = prev.find(cur);
            if (it == prev.end()) return {};
            cur = it->second;
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    void InsertNode(NodeID s) {
        int sid = cellToCluster[s];
        Cluster& c = clusters[sid];
        AddAbstractNode(s, sid);

        auto distPrev = BFSWithinCluster(c, s);
        const auto& dist = distPrev.first;
        const auto& prev = distPrev.second;
        for (NodeID n : c.nodes) {
            if (n == s) continue;
            auto it = dist.find(n);
            if (it == dist.end()) continue;
            std::vector<NodeID> path = ReconstructLocalPath(s, n, prev);
            AddEdge(s, n, it->second, path);
            std::vector<NodeID> rpath = path;
            std::reverse(rpath.begin(), rpath.end());
            AddEdge(n, s, it->second, rpath);
        }
    }

    void ReconstructPath(const std::unordered_map<NodeID, NodeID>& parent) {
        finalPath.clear();
        if (!parent.count(goalNode) && goalNode != startNode) return;
        std::vector<NodeID> abs;
        NodeID cur = goalNode;
        abs.push_back(cur);
        while (cur != startNode) {
            auto it = parent.find(cur);
            if (it == parent.end()) return;
            cur = it->second;
            abs.push_back(cur);
        }
        std::reverse(abs.begin(), abs.end());

        for (size_t i = 0; i + 1 < abs.size(); ++i) {
            auto it = edgePaths.find(Key(abs[i], abs[i + 1]));
            if (it == edgePaths.end()) return;
            const auto& seg = it->second;
            for (size_t k = 0; k + 1 < seg.size(); ++k) {
                finalPath.push_back(seg[k]);
            }
        }
        finalPath.push_back(goalNode);
    }
};
