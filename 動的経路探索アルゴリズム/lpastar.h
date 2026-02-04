#pragma once
#include "common.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

// 高速化のための定数定義
constexpr double INF_VAL = std::numeric_limits<double>::infinity();

class LPAStar : public ISolver {
private:
    // ----------------------------------------------------------------------
    // 1. 高速化のためのデータ構造
    // ----------------------------------------------------------------------

    // 優先度キー（比較演算をインライン化して高速化）
    struct Key {
        double k1; // min(g, rhs) + h
        double k2; // min(g, rhs)

        // 高速な比較演算子
        bool operator<(const Key& other) const {
            if (std::abs(k1 - other.k1) > EPS) return k1 < other.k1;
            return k2 < other.k2 - EPS; // k1が等しい場合はk2で比較
        }
        
        bool operator>(const Key& other) const {
            if (std::abs(k1 - other.k1) > EPS) return k1 > other.k1;
            return k2 > other.k2 + EPS;
        }

        bool operator<=(const Key& other) const { return !(*this > other); }
        bool operator>=(const Key& other) const { return !(*this < other); }
    };

    // ノード情報（メモリパディングを考慮して配置）
    struct NodeData {
        double g = INF_VAL;
        double rhs = INF_VAL;
        double h = 0.0;
        // 親ポインタの代わりに親のNodeIDを保持（パス復元用）
        // ※LPA*では厳密なparentポインタ管理が難しいが、実用上は更新時に記録する
        int parent = -1; 
        
        // ヒープ内の位置（-1ならヒープに入っていない）
        int heapIndex = -1; 
    };

    // ----------------------------------------------------------------------
    // 2. インデックス付きバイナリヒープ (Indexed Binary Heap)
    // std::set や std::priority_queue よりも更新が圧倒的に速い
    // ----------------------------------------------------------------------
    class IndexedHeap {
    private:
        struct HeapItem {
            Key key;
            int u; // NodeID
        };
        std::vector<HeapItem> heap;
        std::vector<NodeData>* nodesPtr; // 外部のノード配列への参照

        void Swap(int i, int j) {
            std::swap(heap[i], heap[j]);
            (*nodesPtr)[heap[i].u].heapIndex = i;
            (*nodesPtr)[heap[j].u].heapIndex = j;
        }

        void UpHeap(int i) {
            while (i > 0) {
                int p = (i - 1) / 2;
                if (heap[i].key < heap[p].key) { // Min-Heap
                    Swap(i, p);
                    i = p;
                } else {
                    break;
                }
            }
        }

        void DownHeap(int i) {
            int size = (int)heap.size();
            while (true) {
                int left = 2 * i + 1;
                int right = 2 * i + 2;
                int smallest = i;

                if (left < size && heap[left].key < heap[smallest].key) {
                    smallest = left;
                }
                if (right < size && heap[right].key < heap[smallest].key) {
                    smallest = right;
                }

                if (smallest != i) {
                    Swap(i, smallest);
                    i = smallest;
                } else {
                    break;
                }
            }
        }

    public:
        void Initialize(std::vector<NodeData>* nPtr) {
            nodesPtr = nPtr;
            heap.clear();
            heap.reserve(nPtr->size() / 4); // ある程度の容量を確保
        }

        bool Empty() const { return heap.empty(); }

        // Topの取得
        std::pair<Key, int> Top() const {
            return { heap[0].key, heap[0].u };
        }

        // Topの削除
        void Pop() {
            if (heap.empty()) return;
            int last = (int)heap.size() - 1;
            (*nodesPtr)[heap[0].u].heapIndex = -1; // 削除マーク
            
            if (last > 0) {
                heap[0] = heap[last];
                (*nodesPtr)[heap[0].u].heapIndex = 0;
                heap.pop_back();
                DownHeap(0);
            } else {
                heap.pop_back();
            }
        }

        // 挿入または更新 (Insert or Update)
        void InsertOrUpdate(int u, Key k) {
            int idx = (*nodesPtr)[u].heapIndex;
            if (idx == -1) {
                // 新規挿入
                idx = (int)heap.size();
                heap.push_back({k, u});
                (*nodesPtr)[u].heapIndex = idx;
                UpHeap(idx);
            } else {
                // 更新
                Key oldKey = heap[idx].key;
                heap[idx].key = k;
                if (k < oldKey) {
                    UpHeap(idx);
                } else {
                    DownHeap(idx);
                }
            }
        }

        // 削除 (Remove)
        void Remove(int u) {
            int idx = (*nodesPtr)[u].heapIndex;
            if (idx == -1) return;

            int last = (int)heap.size() - 1;
            (*nodesPtr)[u].heapIndex = -1;

            if (idx == last) {
                heap.pop_back();
            } else {
                heap[idx] = heap[last];
                (*nodesPtr)[heap[idx].u].heapIndex = idx;
                heap.pop_back();
                // どちらに動くかわからないので両方試す（あるいは比較する）
                UpHeap(idx);
                DownHeap(idx);
            }
        }
    };

    // ----------------------------------------------------------------------
    // メンバ変数
    // ----------------------------------------------------------------------
    std::vector<NodeData> nodes;
    IndexedHeap openList;
    int startNode = -1;
    int goalNode = -1;

    // ----------------------------------------------------------------------
    // プライベートメソッド
    // ----------------------------------------------------------------------
    
    // Key計算のインライン化
    inline Key CalculateKey(int u) const {
        double min_val = std::min(nodes[u].g, nodes[u].rhs);
        return { min_val + nodes[u].h, min_val };
    }

    // 頂点の更新 (UpdateVertex)
    void UpdateVertex(int u) {
        if (u != startNode) {
            // rhsの再計算: 全プレデセッサを見るのではなく
            // 呼び出し元でコストが判明しているなら、そこから更新するのが最速だが
            // LPA*の定義上、ここでは親を探す必要がある。
            // しかし、GridGraphなら「近傍」を見るだけで良い。
            
            double minRhs = INF_VAL;
            int bestP = -1;
            
            // ここがボトルネックになりがち。
            // Graphの実装によっては GetPredecessors が遅い場合があるので注意。
            // 無向グラフ(Undirected)なら GetSuccessors と同じ。
            for (const auto& edge : graph->GetPredecessors(u)) {
                int pred = edge.target; // Predecessor
                double g_pred = nodes[pred].g;
                if (g_pred == INF_VAL) continue;

                double temp = g_pred + edge.cost;
                if (temp < minRhs) {
                    minRhs = temp;
                    bestP = pred;
                }
            }
            nodes[u].rhs = minRhs;
            nodes[u].parent = bestP;
        }

        // OpenListの操作
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
        
        // メモリ再確保を避ける
        if (nodes.size() != (size_t)nodeCount) {
            nodes.assign(nodeCount, NodeData{});
            for (int i = 0; i < nodeCount; ++i) {
                nodes[i].h = graph->H(i, goal);
            }
        } else {
            // 高速なリセット: 全体をループするより、
            // 前回の探索で汚れた部分だけ直すのが早いが、
            // 初回や大きな変更時は全体初期化が無難。
            // ここでは `std::fill` よりもループで構造体初期化を行う
            for(int i=0; i<nodeCount; ++i) {
                nodes[i].g = INF_VAL;
                nodes[i].rhs = INF_VAL;
                nodes[i].parent = -1;
                nodes[i].heapIndex = -1;
                // hは毎回計算するかキャッシュするか。
                // LPA*はゴールが変わらない前提ならキャッシュ有効
                nodes[i].h = graph->H(i, goal);
            }
        }

        // ヒープの初期化
        openList.Initialize(&nodes);

        // スタート地点の設定
        nodes[startNode].rhs = 0;
        
        // 明示的にUpdateVertexを呼ぶ代わりに直接Insert（少しでも速く）
        openList.InsertOrUpdate(startNode, CalculateKey(startNode));
    }

    bool ComputePath() override {
        Timer timer; timer.Start();
        
        // 頻繁にアクセスする変数をローカルにキャッシュ
        int goal = goalNode;
        
        while (!openList.Empty()) {
            auto [topKey, u] = openList.Top();
            
            // 終了条件チェック (ゴールが整合、かつ TopKey >= GoalKey)
            // キー比較を先に行う（軽い処理優先）
            Key goalKey = CalculateKey(goal);
            if (topKey >= goalKey) {
                if (std::abs(nodes[goal].rhs - nodes[goal].g) < EPS) {
                    break;
                }
            }

            // Pop
            openList.Pop();
            metrics.nodesExpanded++;

            // データのローカルコピー（参照アクセスのコスト削減）
            double u_g = nodes[u].g;
            double u_rhs = nodes[u].rhs;

            if (u_g > u_rhs) {
                // --- Locally Overconsistent (通常のA*的更新) ---
                nodes[u].g = u_rhs; // gを更新
                
                // Successorsの更新
                for (const auto& edge : graph->GetSuccessors(u)) {
                    int v = edge.target;
                    // vのrhs更新ロジックをインライン化して高速化
                    // UpdateVertex(v)を呼ぶとPredecessorsを全走査してしまうため、
                    // ここで「uからの遷移」だけを使って高速に更新を試みる
                    
                    double newRhsCandidate = nodes[u].g + edge.cost;
                    if (newRhsCandidate < nodes[v].rhs) {
                        nodes[v].rhs = newRhsCandidate;
                        nodes[v].parent = u;
                        // 整合性が崩れたのでキュー更新
                        openList.InsertOrUpdate(v, CalculateKey(v));
                    }
                }
            } else {
                // --- Locally Underconsistent (障害物発生などでコスト悪化) ---
                nodes[u].g = INF_VAL;
                
                // 自分自身と近傍を更新
                UpdateVertex(u); 
                
                for (const auto& edge : graph->GetSuccessors(u)) {
                    int v = edge.target;
                    // 明示的にUpdateVertexを呼ぶ必要がある
                    // (uを経由しなくなったため、他のルートを探す必要があるから)
                    UpdateVertex(v);
                }
            }
        }

        metrics.runtimeSeconds = timer.GetElapsedSeconds();
        metrics.pathCost = nodes[goalNode].g;
        return nodes[goalNode].g != INF_VAL;
    }

    // 環境変化時の通知
    void NotifyObstacleChange(int u, bool isBlocked) {
        // ノードuの状態が変わった
        // u自身の更新
        if (isBlocked) {
            nodes[u].rhs = INF_VAL;
            nodes[u].parent = -1;
        } else {
            // 障害物がなくなった -> 周囲から入れるかチェック
            // UpdateVertex(u) と同等だが、neighborsを見る必要がある
            double minRhs = INF_VAL;
            int bestP = -1;
            for (const auto& edge : graph->GetPredecessors(u)) {
                 int pred = edge.target;
                 if (nodes[pred].g == INF_VAL) continue;
                 double t = nodes[pred].g + edge.cost;
                 if (t < minRhs) { minRhs = t; bestP = pred; }
            }
            nodes[u].rhs = minRhs;
            nodes[u].parent = bestP;
        }
        
        // uのキュー状態更新
        if (std::abs(nodes[u].g - nodes[u].rhs) > EPS) {
            openList.InsertOrUpdate(u, CalculateKey(u));
        } else {
            openList.Remove(u);
        }

        // uに接続している周囲のノードへの影響
        for (const auto& edge : graph->GetSuccessors(u)) {
            UpdateVertex(edge.target);
        }
    }

    std::vector<int> GetPath() const override {
        std::vector<int> path;
        if (nodes[goalNode].g == INF_VAL) return path;

        int curr = goalNode;
        path.push_back(curr);

        // 無限ループ防止用のカウンター
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