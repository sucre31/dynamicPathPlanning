#include "grid_graph.h"
#include "astar.h"
#include "lpastar.h"
#include "mlpastar.h"
#include "scenario.h"
#include <iostream>
#include <iomanip>
#include <vector>

/**
 * マップ表示関数
 * 障害物(#)、パス(*)、スタート(S)、ゴール(G)を表示します。
 * パスが障害物と重なっている場合は 'X' を表示します。
 */
void PrintMap(const GridGraph& graph, const std::vector<NodeID>& path, int width, int height, NodeID start, NodeID goal) {
    if (width > 50 || height > 50) {
        std::cout << "[Map is too large to display]\n";
        return;
    }

    std::cout << "--- Map View ---\n";
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            char c = '.';
            int id = y * width + x;
            
            if (graph.IsObstacle(x, y)) c = '#';
            
            bool isPath = false;
            for(auto pid : path) if(pid == id) isPath = true;
            
            if (isPath) {
                c = (c == '#') ? 'X' : '*';
            }

            if (id == start) c = 'S';
            else if (id == goal) c = 'G';
            
            std::cout << c << " ";
        }
        std::cout << "\n";
    }
}

/**
 * 実験実行用関数
 * A*, LPA*, m-LPA* の3種類を比較実行します。
 */
void RunExperiment(const Scenario& sc) {
    std::cout << "\n========================================\n";
    std::cout << "SCENARIO: " << sc.name << "\n";
    std::cout << "Map Size: " << sc.width << "x" << sc.height << "\n";
    std::cout << "========================================\n";

    GridGraph graph(sc.width, sc.height);
    AStar astar;
    LPAStar lpa;
    MLPAStar mlpa; // 論文「m-LPA*」に基づく実装クラス [cite: 510]

    astar.SetGraph(&graph);
    lpa.SetGraph(&graph);
    mlpa.SetGraph(&graph);

    // --- ステップ 0: 初期設定と探索 ---
    for (const auto& obs : sc.initialObstacles) {
        graph.SetObstacle(obs.x, obs.y, obs.blocked);
    }

    std::cout << "\n--- [Step 0] Initial Search ---\n";
    
    // A* 初期探索
    astar.Initialize(sc.startNode, sc.goalNode);
    astar.ComputePath();
    astar.GetMetrics().Print("A* Initial");

    // LPA* 初期探索
    lpa.Initialize(sc.startNode, sc.goalNode);
    lpa.ComputePath();
    lpa.GetMetrics().Print("LPA* Initial");

    // m-LPA* 初期探索 (Algorithm 2: Bottom-Up Fusionを実行) [cite: 562, 1382]
    mlpa.Initialize(sc.startNode, sc.goalNode);
    mlpa.ComputePath();
    mlpa.GetMetrics().Print("m-LPA* Initial");

    // パス表示
    std::cout << "\n<A* Initial Path>\n";
    PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);
    
    std::cout << "\n<m-LPA* Initial Path>\n";
    PrintMap(graph, mlpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    // --- ステップ 1以降: 動的な環境変化 ---
    for (size_t i = 0; i < sc.steps.size(); ++i) {
        const auto& step = sc.steps[i];
        std::cout << "\n--- [Step " << i + 1 << "] " << step.name << " ---\n";

        // 障害物の更新
        for (const auto& ev : step.events) {
            graph.SetObstacle(ev.x, ev.y, ev.blocked);
        }

        // 1. A* Replan (常にゼロから再計算)
        astar.Initialize(sc.startNode, sc.goalNode);
        astar.ComputePath();
        astar.GetMetrics().Print("A* Replan");

        // 2. LPA* Replan (増分更新)
        lpa.ResetMetrics();
        for (const auto& ev : step.events) {
            int id = ev.y * sc.width + ev.x;
            lpa.NotifyObstacleChange(id, ev.blocked); 
        }
        lpa.ComputePath();
        lpa.GetMetrics().Print("LPA* Replan");

        // 3. m-LPA* Replan (Algorithm 1 & 3: 局所再分割と増分探索) [cite: 510, 696]
        mlpa.ResetMetrics();
        for (const auto& ev : step.events) {
            int id = ev.y * sc.width + ev.x;
            // 変化があった箇所の正方形のみを特定し、ビームレットを再計算 [cite: 601, 677]
            mlpa.NotifyObstacleChange(id, ev.blocked); 
        }
        mlpa.ComputePath();
        mlpa.GetMetrics().Print("m-LPA* Replan");

        // 各アルゴリズムの再計画結果を表示
        std::cout << "\n<A* Replan Path>\n";
        PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<LPA* Replan Path>\n";
        PrintMap(graph, lpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<m-LPA* Replan Path>\n";
        PrintMap(graph, mlpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);
    }
}

int main() {
    // 論文の前提条件通り、Dyadic(2のべき乗)サイズのマップでテスト [cite: 1276]
    // 1. Simple Wall (16x16)
    RunExperiment(Scenario::CreateSimpleWall());
    
    // 2. Trap (32x32)
    RunExperiment(Scenario::CreateTrap());
    
    // 3. Bottleneck (32x32)
    RunExperiment(Scenario::CreateBottleneck());
    
    RunExperiment(Scenario::CreatePaperBenchmark());

    return 0;
}