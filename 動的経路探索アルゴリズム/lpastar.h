#pragma once
#include "common.h"
#include <vector>
#include <set>
#include <algorithm>
#include <tuple>
#include <cmath>
#include <limits>

class LPAStar : public ISolver {
protected:
    // 優先度キー
    struct Key {
        double k1; // min(g, rhs) + h
        double k2; // min(g, rhs)

        // 辞書順比較 (Lexicographical order)
        bool operator<(const Key& other) const {
            if (std::abs(k1 - other.k1) > EPS) return k1 < other.k1;
            return k2 < other.k2 - EPS;
        }
        bool operator>(const Key& other) const { return other < *this; }
        bool operator<=(const Key& other) const { return !(*this > other); }
        bool operator>=(const Key& other) const { return !(*this < other); }
        bool operator==(const Key& other) const {
            return std::abs(k1 - other.k1) < EPS && std::abs(k2 - other.k2) < EPS;
        }
        bool operator!=(const Key& other) const { return !(*this == other); }
    };

    // ノードの状態
    struct NodeState {
        Cost g = INF;
        Cost rhs = INF;
        Cost h = 0;
        NodeID parent = -1; // 最適化のための親ポインタ p(s)
    };

    std::vector<NodeState> states;
    NodeID startNode = -1;
    NodeID goalNode = -1;

    // 優先度付きキュー (Open List)
    using QueueItem = std::pair<Key, NodeID>;
    std::set<QueueItem> openList;
    std::vector<std::set<QueueItem>::iterator> openListRefs;
    std::vector<bool> inOpenList;

    // ヒューリスティック関数
    Cost H(NodeID u) const {
        return graph->H(u, goalNode);
    }

    // Key 計算 (CalculateKey)
    Key CalculateKey(NodeID u) const {
        double min_g_rhs = std::min(states[u].g, states[u].rhs);
        return { min_g_rhs + states[u].h, min_g_rhs };
    }

    // ノードの整合性をチェックし、OpenListを更新する
    void UpdateVertex(NodeID u) {
        // 既にキューにある場合は削除（キー更新のため）
        if (inOpenList[u]) {
            openList.erase(openListRefs[u]);
            inOpenList[u] = false;
        }

        // 局所整合性が取れていない (g != rhs) ならキューに挿入
        if (std::abs(states[u].g - states[u].rhs) > EPS) {
            Key k = CalculateKey(u);
            auto res = openList.insert({k, u});
            openListRefs[u] = res.first;
            inOpenList[u] = true;
        }
    }

    // 親ノードを全走査して rhs を再計算するヘルパー関数
    // Figure 7 の arg min ... の部分に相当
    void RecalculateRhs(NodeID u) {
        if (u == startNode) {
            states[u].rhs = 0;
            states[u].parent = -1;
            return;
        }

        Cost minRhs = INF;
        NodeID bestP = -1;

        for (const auto& edge : graph->GetPredecessors(u)) {
            NodeID pred = edge.target; // GridGraphの実装依存: 入ってくるエッジの始点
            double cost = (states[pred].g == INF) ? INF : (states[pred].g + edge.cost);
            
            if (cost < minRhs) {
                minRhs = cost;
                bestP = pred;
            }
        }
        states[u].rhs = minRhs;
        states[u].parent = bestP;
    }

public:
    void Initialize(NodeID start, NodeID goal) override {
        startNode = start;
        goalNode = goal;
        metrics.Reset();

        int nodeCount = graph->GetNodeCount();
        states.assign(nodeCount, NodeState{});
        openListRefs.assign(nodeCount, openList.end());
        inOpenList.assign(nodeCount, false);
        openList.clear();

        // ヒューリスティックの計算
        for (int i = 0; i < nodeCount; ++i) {
            states[i].h = H(i);
            states[i].g = INF;
            states[i].rhs = INF;
            states[i].parent = -1;
        }

        // スタートノードの初期化
        states[startNode].rhs = 0;
        
        // スタートノードをキューに入れる
        Key k = CalculateKey(startNode);
        auto res = openList.insert({k, startNode});
        openListRefs[startNode] = res.first;
        inOpenList[startNode] = true;
    }

    // 最短経路計算 (Figure 7: ComputeShortestPath)
    bool ComputePath() override {
        Timer timer; timer.Start();
        if (!graph) return false;

        while (!openList.empty()) {
            Key topKey = openList.begin()->first;
            Key goalKey = CalculateKey(goalNode);

            // 終了条件: TopKey >= Key(goal) かつ rhs(goal) == g(goal)
            if (topKey >= goalKey && std::abs(states[goalNode].rhs - states[goalNode].g) < EPS) {
                break;
            }

            NodeID u = openList.begin()->second;
            openList.erase(openList.begin());
            inOpenList[u] = false;
            metrics.nodesExpanded++;

            if (states[u].g > states[u].rhs) {
                // ■ ケース1: Locally Overconsistent (コスト改善)
                states[u].g = states[u].rhs;
                
                // 最適化: u から出るエッジのみを見て、改善できるかチェック
                for (const auto& edge : graph->GetSuccessors(u)) {
                    NodeID s = edge.target;
                    // u を経由することで s のコストが下がる場合のみ更新
                    if (states[s].rhs > states[u].g + edge.cost) {
                        states[s].rhs = states[u].g + edge.cost;
                        states[s].parent = u;
                        UpdateVertex(s);
                    }
                }
            } else {
                // ■ ケース2: Locally Underconsistent (コスト悪化)
                states[u].g = INF;
                
                // u 自身と、u を親としていた後継ノードを更新
                // まず u 自身 (自分自身へのループバックがある場合や、整合性確保のため)
                UpdateVertex(u); 

                for (const auto& edge : graph->GetSuccessors(u)) {
                    NodeID s = edge.target;
                    
                    // 最適化: 親が u であった場合のみ、再スキャンが必要
                    if (states[s].parent == u) {
                        RecalculateRhs(s);
                        UpdateVertex(s);
                    }
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        metrics.pathCost = states[goalNode].g;
        return states[goalNode].g != INF;
    }

    // 環境変化時の通知 (Figure 7: Main loop logic adapted for single node change)
    // ノード u の通行可否が変わったときに呼ばれる
    virtual void NotifyObstacleChange(NodeID u, bool isBlocked) {
        // グリッドグラフでノード u が変化すると、
        // 1. u に入るエッジ (v->u) のコスト変化
        // 2. u から出るエッジ (u->v) のコスト変化
        // の両方が発生します。

        // --- 1. ノード u 自身の更新 (v->u の影響) ---
        if (isBlocked) {
            // 障害物になった -> コスト無限大
            states[u].rhs = INF;
            states[u].parent = -1;
        } else {
            // 障害物が消えた -> 周囲から入れるようになるので再計算
            RecalculateRhs(u);
        }
        UpdateVertex(u);

        // --- 2. 周囲のノード v の更新 (u->v の影響) ---
        for (const auto& edge : graph->GetSuccessors(u)) {
            NodeID v = edge.target;
            
            if (isBlocked) {
                // コスト増加 (u->v が切断)
                // v の親が u だった場合のみ、v の代替ルートを探す必要がある
                if (states[v].parent == u) {
                    RecalculateRhs(v);
                    UpdateVertex(v);
                }
            } else {
                // コスト減少 (u->v が接続)
                // u を経由して v に安く行けるようになったかチェック
                Cost newCost = states[u].g + edge.cost;
                if (states[v].rhs > newCost) {
                    states[v].rhs = newCost;
                    states[v].parent = u;
                    UpdateVertex(v);
                }
            }
        }
    }

    // 経路復元
    std::vector<NodeID> GetPath() const override {
        std::vector<NodeID> path;
        if (states[goalNode].g == INF) return path;

        NodeID curr = goalNode;
        path.push_back(curr);

        // 親ポインタを辿るだけなので高速
        // ただし、整合性が崩れている一時的な状態ではループする可能性があるので
        // 安全のためステップ数制限などを入れても良い
        while (curr != startNode) {
            NodeID p = states[curr].parent;
            if (p == -1 || p == curr) break; // 経路なし or エラー
            curr = p;
            path.push_back(curr);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};