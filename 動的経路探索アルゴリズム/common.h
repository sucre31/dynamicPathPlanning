#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <chrono> 
#include <iomanip>
#include <tuple> // std::tie用

// 基本型定義
using NodeID = int;
using Cost = double;
constexpr Cost INF = std::numeric_limits<Cost>::infinity();
constexpr double EPS = 1e-9; // 浮動小数点誤差許容値

// 計測用メトリクス
struct SearchMetrics {
    long nodesExpanded = 0;   
    long nodesGenerated = 0;  
    double pathCost = 0.0;
    double runtimeSeconds = 0.0; 

    void Reset() {
        nodesExpanded = 0;
        nodesGenerated = 0;
        pathCost = 0.0;
        runtimeSeconds = 0.0;
    }

    void Print(std::string label) const {
        std::cout << "[" << label << "] " 
                  << "Time: " << std::fixed << std::setprecision(6) << runtimeSeconds * 1000.0 << " ms, "
                  << "Expanded: " << nodesExpanded << ", "
                  << "Generated: " << nodesGenerated << ", "
                  << "Cost: " << std::setprecision(1) << pathCost << std::endl;
    }
};

// グラフインターフェース
class IGraph {
public:
    struct Edge {
        NodeID target; // or source (context dependent)
        Cost cost;
    };
    virtual ~IGraph() = default;
    
    virtual std::vector<Edge> GetSuccessors(NodeID u) const = 0;
    // ★追加: LPA*のrhs計算と経路復元に必要
    virtual std::vector<Edge> GetPredecessors(NodeID u) const = 0; 
    
    virtual Cost H(NodeID from, NodeID to) const = 0;
    virtual int GetNodeCount() const = 0;
};

// ソルバー基底クラス
class ISolver {
protected:
    const IGraph* graph = nullptr;
    SearchMetrics metrics;

public:
    virtual ~ISolver() = default;
    
    void SetGraph(const IGraph* g) { graph = g; }
    void ResetMetrics() { metrics.Reset(); }

    virtual void Initialize(NodeID start, NodeID goal) = 0;
    virtual bool ComputePath() = 0;
    virtual std::vector<NodeID> GetPath() const = 0;
    
    const SearchMetrics& GetMetrics() const { return metrics; }
};

// 簡易タイマークラス
class Timer {
    std::chrono::high_resolution_clock::time_point start_time;
public:
    void Start() { start_time = std::chrono::high_resolution_clock::now(); }
    double GetElapsedSeconds() {
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end_time - start_time;
        return diff.count();
    }
};