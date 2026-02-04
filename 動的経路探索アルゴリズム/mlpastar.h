#pragma once
#include "lpastar.h"
#include "mastar.h"
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <queue>
#include <vector>
#include <algorithm>

#ifndef ENABLE_MLPASTAR
class MLPAStar : public ISolver {
public:
    void Initialize(NodeID, NodeID) override {}
    bool ComputePath() override { return false; }
    std::vector<NodeID> GetPath() const override { return {}; }
    void NotifyObstacleChange(NodeID, bool) {}
};
#else
// m-LPA* implementation aligned to the paper flow:
// DPFR-RDP -> BottomUpFusion -> BeamletGraph -> LPA* replanning.
class MLPAStar : public LPAStar {
    using DSquare = MAStar::DSquare;
    using Beamlet = MAStar::Beamlet;

    DSquare* root = nullptr;

    // beamlet graph (directed)
    std::unordered_map<NodeID, std::vector<std::pair<NodeID, Cost>>> adj;
    std::unordered_map<NodeID, std::vector<std::pair<NodeID, Cost>>> revAdj;

    // beamlet table for path reconstruction
    std::map<std::pair<NodeID, NodeID>, Beamlet> beamletCache;
    std::vector<NodeID> fullPixelPath;

public:
    ~MLPAStar() { if (root) delete root; }

    void Initialize(NodeID start, NodeID goal) override {
        // base initialization (g/rhs, openlist, heuristic cache)
        LPAStar::Initialize(start, goal);
        this->startNode = start;
        this->goalNode = goal;

        if (root) { delete root; root = nullptr; }
        adj.clear();
        revAdj.clear();
        beamletCache.clear();
        fullPixelPath.clear();

        // DPFR-RDP (initial)
        int mapSize = std::min(GetGrid()->GetWidth(), GetGrid()->GetHeight());
        root = BuildDPFRRDP(0, 0, mapSize, 0);

        // Bottom-Up Fusion on all d-squares
        RunFullBottomUpFusion(root);

        // Build beamlet graph and sync LPA*
        RebuildBeamletGraph();
        SyncAllStates();
    }

    void NotifyObstacleChange(NodeID node, bool /*blocked*/) override {
        int vx, vy;
        GetCoord(node, vx, vy);

        // DPFR-RDP update: refine only along the path to the updated vertex
        std::vector<DSquare*> path;
        if (!CollectPath(root, vx, vy, path)) return;
        if (path.empty()) return;

        // Update boundaries on the affected path (obstacle may touch boundary cells)
        for (DSquare* ds : path) {
            ExtractBoundary(ds);
        }

        // Recompute beamlets bottom-up along the path and collect changed edges
        std::unordered_set<uint64_t> changedEdges;
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            DSquare* ds = *it;
            if (ds->isLeaf) {
                ComputeBeamletsInLeaf(ds, changedEdges);
            } else {
                ComputeBeamletsByFusion(ds, changedEdges);
            }
        }

        // update beamlet cache for changed edges and collect changed sources/targets
        std::unordered_set<NodeID> changedSources;
        std::unordered_set<NodeID> changedTargets;
        UpdateBeamletCacheFromChanges(changedEdges, changedSources, changedTargets);

        // Update start/goal connections if they are not in the abstract boundary set
        UpdateStartGoalConnectionsIfNeeded(vx, vy, changedSources, changedTargets);

        // rebuild adjacency for changed nodes only
        RebuildAdjForChangedNodes(changedSources, changedTargets);

        // Algorithm 3: for all directed edges (u,w) with changed cost, UpdateState(u)
        for (NodeID u : changedSources) UpdateState(u);
        // Ensure targets whose rhs changed are updated (revAdj is partially rebuilt)
        for (NodeID v : changedTargets) UpdateState(v);
    }

    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        while (!openList.empty()) {
            auto topKey = openList.begin()->first;
            auto goalKey = CalculateKey(goalNode);

            if (topKey >= goalKey && std::abs(states[goalNode].rhs - states[goalNode].g) < EPS) {
                break;
            }

            NodeID u = openList.begin()->second;
            openList.erase(openList.begin());
            inOpenList[u] = false;
            metrics.nodesExpanded++;

            if (states[u].g > states[u].rhs) {
                states[u].g = states[u].rhs;
                if (adj.count(u)) {
                    for (const auto& edge : adj.at(u)) {
                        UpdateState(edge.first);
                    }
                }
            } else {
                states[u].g = INF;
                UpdateState(u);
                if (adj.count(u)) {
                    for (const auto& edge : adj.at(u)) {
                        UpdateState(edge.first);
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
    void UpdateState(NodeID u) {
        if (u != startNode) {
            RecalculateRhs(u);
        } else {
            states[u].rhs = 0;
            states[u].parent = -1;
        }

        // Follow Algorithm 3: remove from OPEN, reinsert only if g != rhs
        if (inOpenList[u]) {
            openList.erase(openListRefs[u]);
            inOpenList[u] = false;
        }
        if (std::abs(states[u].g - states[u].rhs) > EPS) {
            auto k = CalculateKey(u);
            auto res = openList.insert({k, u});
            openListRefs[u] = res.first;
            inOpenList[u] = true;
        }
    }

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

    uint64_t Key(NodeID a, NodeID b) const {
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    }

    // ---- DPFR-RDP ----
    DSquare* BuildDPFRRDP(int x, int y, int size, int level) {
        DSquare* ds = new DSquare(x, y, size, level);
        ExtractBoundary(ds);
        if (size > 1 && IsNearSG(x, y, size)) {
            int ns = size / 2;
            for (int i = 0; i < 4; ++i) {
                ds->children[i] = BuildDPFRRDP(x + (i % 2) * ns, y + (i / 2) * ns, ns, level + 1);
            }
            ds->isLeaf = false;
        } else {
            ds->isLeaf = true;
        }
        return ds;
    }

    void RefineAt(DSquare* ds) {
        if (!ds || ds->size <= 1) return;
        if (ds->isLeaf) {
            ds->isLeaf = false;
            int ns = ds->size / 2;
            for (int i = 0; i < 4; ++i) {
                ds->children[i] = new DSquare(ds->x + (i % 2) * ns, ds->y + (i / 2) * ns, ns, ds->level + 1);
                ExtractBoundary(ds->children[i]);
                ds->children[i]->isLeaf = true;
            }
        }
    }

    bool CollectPath(DSquare* ds, int vx, int vy, std::vector<DSquare*>& out) {
        if (!ds) return false;
        if (vx < ds->x || vx >= ds->x + ds->size || vy < ds->y || vy >= ds->y + ds->size) return false;
        out.push_back(ds);
        if (ds->isLeaf) return true;
        for (int i = 0; i < 4; ++i) {
            if (CollectPath(ds->children[i], vx, vy, out)) return true;
        }
        return false;
    }

    void RefineAlongPath(std::vector<DSquare*>& /*path*/, int /*vx*/, int /*vy*/) {
        // DPFR-RDP update per paper: do not further subdivide beyond the current
        // minimal d-square that contains the update. Only recompute fusion.
    }

    // ---- Bottom-Up Fusion ----
    void UpdateLocalFusion(DSquare* ds, int vx, int vy) {
        if (!ds) return;
        if (vx < ds->x || vx >= ds->x + ds->size || vy < ds->y || vy >= ds->y + ds->size) return;
        std::unordered_set<uint64_t> dummy;
        if (!ds->isLeaf) {
            for (int i = 0; i < 4; ++i) UpdateLocalFusion(ds->children[i], vx, vy);
            ComputeBeamletsByFusion(ds, dummy);
        } else {
            ComputeBeamletsInLeaf(ds, dummy);
        }
    }

    void ComputeBeamletsInLeaf(DSquare* ds, std::unordered_set<uint64_t>& changedEdges) {
        std::unordered_map<uint64_t, Beamlet> newTable;
        for (NodeID u : ds->boundary) {
            std::queue<NodeID> q;
            std::unordered_map<NodeID, Cost> dist;
            std::unordered_map<NodeID, NodeID> prev;
            dist[u] = 0;
            q.push(u);

            while (!q.empty()) {
                NodeID cur = q.front(); q.pop();
                int cx, cy; GetCoord(cur, cx, cy);
                int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
                for (int i = 0; i < 4; ++i) {
                    int nx = cx + dx[i], ny = cy + dy[i];
                    if (nx < ds->x || nx >= ds->x + ds->size ||
                        ny < ds->y || ny >= ds->y + ds->size) continue;
                    if (GetGrid()->IsObstacle(nx, ny)) continue;
                    NodeID v = ny * GetGrid()->GetWidth() + nx;
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
                for (NodeID c = v; c != u; c = prev[c]) b.pixels.push_back(c);
                b.pixels.push_back(u);
                std::reverse(b.pixels.begin(), b.pixels.end());
                newTable[Key(u, v)] = b;
            }
        }
        UpdateTableWithDiff(ds, newTable, changedEdges);
    }

    void ComputeBeamletsByFusion(DSquare* ds, std::unordered_set<uint64_t>& changedEdges) {
        struct FusedEdge { NodeID to; Cost cost; std::vector<NodeID> pixels; };
        std::unordered_map<uint64_t, Beamlet> newTable;
        std::unordered_set<NodeID> nodeSet;
        for (int i = 0; i < 4; ++i) {
            DSquare* child = ds->children[i];
            if (!child) continue;
            for (NodeID n : child->boundary) nodeSet.insert(n);
        }
        if (nodeSet.empty()) return;

        std::unordered_map<NodeID, std::vector<FusedEdge>> adjLocal;
        for (int i = 0; i < 4; ++i) {
            DSquare* child = ds->children[i];
            if (!child) continue;
            for (const auto& kv : child->table) {
                NodeID a = static_cast<NodeID>(kv.first >> 32);
                NodeID b = static_cast<NodeID>(kv.first & 0xffffffffu);
                const Beamlet& bl = kv.second;
                adjLocal[a].push_back({b, bl.cost, bl.pixels});
            }
        }

        for (NodeID u : nodeSet) {
            int ux, uy; GetCoord(u, ux, uy);
            int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i) {
                int vx = ux + dx[i], vy = uy + dy[i];
                if (vx < ds->x || vx >= ds->x + ds->size ||
                    vy < ds->y || vy >= ds->y + ds->size) continue;
                if (GetGrid()->IsObstacle(vx, vy)) continue;
                NodeID v = vy * GetGrid()->GetWidth() + vx;
                if (nodeSet.count(v)) adjLocal[u].push_back({v, 1.0, {u, v}});
            }
        }

        for (NodeID source : ds->boundary) {
            using Q = std::pair<Cost, NodeID>;
            std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
            std::unordered_map<NodeID, Cost> dist;
            struct Prev { NodeID prev; std::vector<NodeID> edgePixels; };
            std::unordered_map<NodeID, Prev> prev;

            dist[source] = 0;
            pq.push({0, source});
            while (!pq.empty()) {
                auto [d, u] = pq.top(); pq.pop();
                if (d > dist[u]) continue;
                auto it = adjLocal.find(u);
                if (it == adjLocal.end()) continue;
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
                if (source == target || !dist.count(target)) continue;
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
                    if (b.pixels.empty()) b.pixels.insert(b.pixels.end(), seg.begin(), seg.end());
                    else b.pixels.insert(b.pixels.end(), seg.begin() + 1, seg.end());
                }
                newTable[Key(source, target)] = b;
            }
        }
        UpdateTableWithDiff(ds, newTable, changedEdges);
    }

    void UpdateTableWithDiff(DSquare* ds,
                             const std::unordered_map<uint64_t, Beamlet>& newTable,
                             std::unordered_set<uint64_t>& changedEdges) {
        // detect removed or changed edges
        for (const auto& kv : ds->table) {
            auto it = newTable.find(kv.first);
            if (it == newTable.end() || std::abs(it->second.cost - kv.second.cost) > EPS) {
                changedEdges.insert(kv.first);
            }
        }
        // detect added or changed edges
        for (const auto& kv : newTable) {
            auto it = ds->table.find(kv.first);
            if (it == ds->table.end() || std::abs(it->second.cost - kv.second.cost) > EPS) {
                changedEdges.insert(kv.first);
            }
        }

        // commit new table
        ds->table.clear();
        for (const auto& kv : newTable) {
            ds->table[kv.first] = kv.second;
        }
    }

    void RunFullBottomUpFusion(DSquare* ds) {
        if (!ds) return;
        if (!ds->isLeaf) for (int i = 0; i < 4; ++i) RunFullBottomUpFusion(ds->children[i]);
        std::unordered_set<uint64_t> dummy;
        if (ds->isLeaf) ComputeBeamletsInLeaf(ds, dummy);
        else ComputeBeamletsByFusion(ds, dummy);
    }

    void CollectDSquares(DSquare* ds, std::vector<DSquare*>& out) {
        if (!ds) return;
        out.push_back(ds);
        if (!ds->isLeaf) for (int i = 0; i < 4; ++i) CollectDSquares(ds->children[i], out);
    }

    void RebuildBeamletGraph() {
        beamletCache.clear();
        adj.clear();
        revAdj.clear();

        std::vector<DSquare*> nodes;
        CollectDSquares(root, nodes);

        std::unordered_set<NodeID> boundarySet;
        for (auto* ds : nodes) {
            for (NodeID v : ds->boundary) boundarySet.insert(v);
            for (const auto& kv : ds->table) {
                NodeID a = static_cast<NodeID>(kv.first >> 32);
                NodeID b = static_cast<NodeID>(kv.first & 0xffffffffu);
                const Beamlet& bl = kv.second;
                auto key = std::make_pair(a, b);
                auto it = beamletCache.find(key);
                if (it == beamletCache.end() || bl.cost < it->second.cost) {
                    beamletCache[key] = bl;
                }
            }
        }

        // Ensure start/goal are present in the abstract graph when they are not on any boundary
        if (!boundarySet.count(startNode)) {
            DSquare* anchor = FindAnchorSquare(startNode);
            AddStartGoalBeamletsInSquare(anchor, startNode);
        }
        if (!boundarySet.count(goalNode)) {
            DSquare* anchor = FindAnchorSquare(goalNode);
            AddStartGoalBeamletsInSquare(anchor, goalNode);
        }

        for (const auto& kv : beamletCache) {
            adj[kv.first.first].push_back({kv.first.second, kv.second.cost});
            revAdj[kv.first.second].push_back({kv.first.first, kv.second.cost});
        }
    }

    bool FindBestEdge(NodeID a, NodeID b, Beamlet& out) const {
        bool found = false;
        std::vector<DSquare*> stack;
        if (root) stack.push_back(root);
        while (!stack.empty()) {
            DSquare* ds = stack.back(); stack.pop_back();
            auto it = ds->table.find(Key(a, b));
            if (it != ds->table.end()) {
                if (!found || it->second.cost < out.cost) {
                    out = it->second;
                    found = true;
                }
            }
            if (!ds->isLeaf) {
                for (int i = 0; i < 4; ++i) if (ds->children[i]) stack.push_back(ds->children[i]);
            }
        }
        return found;
    }

    void UpdateBeamletCacheFromChanges(const std::unordered_set<uint64_t>& changedEdges,
                                       std::unordered_set<NodeID>& changedSources,
                                       std::unordered_set<NodeID>& changedTargets) {
        for (uint64_t key : changedEdges) {
            NodeID a = static_cast<NodeID>(key >> 32);
            NodeID b = static_cast<NodeID>(key & 0xffffffffu);
            Beamlet best;
            bool has = FindBestEdge(a, b, best);

            auto it = beamletCache.find({a, b});
            if (has) {
                if (it == beamletCache.end() || std::abs(it->second.cost - best.cost) > EPS) {
                    beamletCache[{a, b}] = best;
                    changedSources.insert(a);
                    changedTargets.insert(b);
                }
            } else {
                if (it != beamletCache.end()) {
                    beamletCache.erase(it);
                    changedSources.insert(a);
                    changedTargets.insert(b);
                }
            }
        }
    }

    void RebuildAdjForChangedNodes(const std::unordered_set<NodeID>& changedSources,
                                   const std::unordered_set<NodeID>& changedTargets) {
        for (NodeID u : changedSources) adj[u].clear();
        for (NodeID v : changedTargets) revAdj[v].clear();
        for (const auto& kv : beamletCache) {
            NodeID u = kv.first.first;
            NodeID v = kv.first.second;
            if (changedSources.count(u)) adj[u].push_back({v, kv.second.cost});
            if (changedTargets.count(v)) revAdj[v].push_back({u, kv.second.cost});
        }
    }

    void SyncAllStates() {
        std::unordered_set<NodeID> nodes;
        for (const auto& kv : adj) {
            nodes.insert(kv.first);
            for (const auto& e : kv.second) nodes.insert(e.first);
        }
        for (NodeID v : nodes) {
            UpdateState(v);
        }
    }

    bool IsNearSG(int x, int y, int size) {
        int sx, sy, gx, gy;
        GetCoord(startNode, sx, sy); GetCoord(goalNode, gx, gy);
        return (sx >= x && sx < x + size && sy >= y && sy < y + size) ||
               (gx >= x && gx < x + size && gy >= y && gy < y + size);
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
        ds->boundary.erase(std::unique(ds->boundary.begin(), ds->boundary.end()), ds->boundary.end());
    }

    void AddIfFree(int x, int y, DSquare* ds) {
        if (x >= 0 && x < GetGrid()->GetWidth() && y >= 0 && y < GetGrid()->GetHeight())
            if (!GetGrid()->IsObstacle(x, y)) ds->boundary.push_back(y * GetGrid()->GetWidth() + x);
    }

    bool Contains(DSquare* ds, int vx, int vy) const {
        return ds && vx >= ds->x && vx < ds->x + ds->size &&
               vy >= ds->y && vy < ds->y + ds->size;
    }

    DSquare* FindAnchorSquare(NodeID node) {
        int x, y;
        GetCoord(node, x, y);
        std::vector<DSquare*> path;
        if (!CollectPath(root, x, y, path) || path.empty()) return nullptr;
        DSquare* ds = path.back();
        if (ds->size <= 1 && path.size() >= 2) ds = path[path.size() - 2];
        return ds;
    }

    void AddStartGoalBeamletsInSquare(DSquare* ds, NodeID src) {
        if (!ds) return;
        if (GetGrid()->IsObstacle(src % GetGrid()->GetWidth(), src / GetGrid()->GetWidth())) return;

        std::queue<NodeID> q;
        std::unordered_map<NodeID, Cost> dist;
        std::unordered_map<NodeID, NodeID> prev;
        dist[src] = 0;
        q.push(src);

        int w = GetGrid()->GetWidth();
        while (!q.empty()) {
            NodeID cur = q.front(); q.pop();
            int cx = cur % w;
            int cy = cur / w;
            int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i) {
                int nx = cx + dx[i], ny = cy + dy[i];
                if (nx < ds->x || nx >= ds->x + ds->size ||
                    ny < ds->y || ny >= ds->y + ds->size) continue;
                if (GetGrid()->IsObstacle(nx, ny)) continue;
                NodeID v = ny * w + nx;
                if (!dist.count(v)) {
                    dist[v] = dist[cur] + 1;
                    prev[v] = cur;
                    q.push(v);
                }
            }
        }

        for (NodeID v : ds->boundary) {
            if (v == src) continue;
            auto it = dist.find(v);
            if (it == dist.end()) continue;
            Beamlet b;
            b.cost = it->second;
            NodeID step = v;
            while (step != src) {
                b.pixels.push_back(step);
                step = prev[step];
            }
            b.pixels.push_back(src);
            std::reverse(b.pixels.begin(), b.pixels.end());
            beamletCache[{src, v}] = b;

            Beamlet br = b;
            std::reverse(br.pixels.begin(), br.pixels.end());
            beamletCache[{v, src}] = br;
        }
    }

    void UpdateStartGoalConnectionsIfNeeded(int vx, int vy,
                                            std::unordered_set<NodeID>& changedSources,
                                            std::unordered_set<NodeID>& changedTargets) {
        // Only add special connections if start/goal are not already boundary nodes
        bool startIsBoundary = IsBoundaryNode(startNode);
        bool goalIsBoundary = IsBoundaryNode(goalNode);

        if (!startIsBoundary) {
            DSquare* ds = FindAnchorSquare(startNode);
            if (ds && Contains(ds, vx, vy)) {
                UpdateStartGoalBeamletsInSquare(ds, startNode, changedSources, changedTargets);
            }
        }
        if (!goalIsBoundary) {
            DSquare* ds = FindAnchorSquare(goalNode);
            if (ds && Contains(ds, vx, vy)) {
                UpdateStartGoalBeamletsInSquare(ds, goalNode, changedSources, changedTargets);
            }
        }
    }

    void UpdateStartGoalBeamletsInSquare(DSquare* ds, NodeID src,
                                         std::unordered_set<NodeID>& changedSources,
                                         std::unordered_set<NodeID>& changedTargets) {
        if (!ds) return;
        if (GetGrid()->IsObstacle(src % GetGrid()->GetWidth(), src / GetGrid()->GetWidth())) return;

        std::queue<NodeID> q;
        std::unordered_map<NodeID, Cost> dist;
        std::unordered_map<NodeID, NodeID> prev;
        dist[src] = 0;
        q.push(src);

        int w = GetGrid()->GetWidth();
        while (!q.empty()) {
            NodeID cur = q.front(); q.pop();
            int cx = cur % w;
            int cy = cur / w;
            int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
            for (int i = 0; i < 4; ++i) {
                int nx = cx + dx[i], ny = cy + dy[i];
                if (nx < ds->x || nx >= ds->x + ds->size ||
                    ny < ds->y || ny >= ds->y + ds->size) continue;
                if (GetGrid()->IsObstacle(nx, ny)) continue;
                NodeID v = ny * w + nx;
                if (!dist.count(v)) {
                    dist[v] = dist[cur] + 1;
                    prev[v] = cur;
                    q.push(v);
                }
            }
        }

        for (NodeID v : ds->boundary) {
            if (v == src) continue;
            auto it = dist.find(v);
            if (it == dist.end()) {
                if (beamletCache.erase({src, v}) > 0) {
                    changedSources.insert(src);
                    changedTargets.insert(v);
                }
                if (beamletCache.erase({v, src}) > 0) {
                    changedSources.insert(v);
                    changedTargets.insert(src);
                }
                continue;
            }

            Beamlet b;
            b.cost = it->second;
            NodeID step = v;
            while (step != src) {
                b.pixels.push_back(step);
                step = prev[step];
            }
            b.pixels.push_back(src);
            std::reverse(b.pixels.begin(), b.pixels.end());

            auto it1 = beamletCache.find({src, v});
            if (it1 == beamletCache.end() || std::abs(it1->second.cost - b.cost) > EPS) {
                beamletCache[{src, v}] = b;
                changedSources.insert(src);
                changedTargets.insert(v);
            }

            Beamlet br = b;
            std::reverse(br.pixels.begin(), br.pixels.end());
            auto it2 = beamletCache.find({v, src});
            if (it2 == beamletCache.end() || std::abs(it2->second.cost - br.cost) > EPS) {
                beamletCache[{v, src}] = br;
                changedSources.insert(v);
                changedTargets.insert(src);
            }
        }
    }

    bool IsBoundaryNode(NodeID node) const {
        std::vector<DSquare*> stack;
        if (root) stack.push_back(root);
        while (!stack.empty()) {
            DSquare* ds = stack.back(); stack.pop_back();
            if (std::binary_search(ds->boundary.begin(), ds->boundary.end(), node)) return true;
            if (!ds->isLeaf) {
                for (int i = 0; i < 4; ++i) if (ds->children[i]) stack.push_back(ds->children[i]);
            }
        }
        return false;
    }

    void ReconstructFullPixelPath() {
        fullPixelPath.clear();
        if (states[goalNode].g >= INF) return;

        std::vector<NodeID> abstractPath;
        NodeID curr = goalNode;
        std::unordered_set<NodeID> seen;
        while (curr != startNode && curr != -1) {
            if (seen.count(curr)) return;
            seen.insert(curr);
            abstractPath.push_back(curr);
            NodeID next = states[curr].parent;
            if (next == -1 || next == curr) break;
            curr = next;
        }
        if (curr != startNode) return;
        abstractPath.push_back(startNode);
        std::reverse(abstractPath.begin(), abstractPath.end());

        for (size_t i = 0; i + 1 < abstractPath.size(); ++i) {
            auto it = beamletCache.find({abstractPath[i], abstractPath[i + 1]});
            if (it == beamletCache.end() || it->second.cost >= INF) return;
            for (size_t j = 0; j + 1 < it->second.pixels.size(); ++j)
                fullPixelPath.push_back(it->second.pixels[j]);
        }
        fullPixelPath.push_back(goalNode);
    }
};
#endif
