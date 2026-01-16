#include "grid_graph.h"
#include "astar.h"
#include "lpastar.h"
#include "mastar.h"
#include "scenario.h"
#include <iostream>
#include <iomanip>
#include <vector>

// マップ表示（S: Start, G: Goal を表示）
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
            
            // 1. 基本は障害物か空白
            if (graph.IsObstacle(x, y)) c = '#';
            
            // 2. パスがあれば上書き (* または X)
            bool isPath = false;
            for(auto pid : path) if(pid == id) isPath = true;
            
            if (isPath) {
                c = (c == '#') ? 'X' : '*';
            }

            // 3. スタートとゴールは最優先で上書き
            if (id == start) c = 'S';
            else if (id == goal) c = 'G';
            
            std::cout << c << " ";
        }
        std::cout << "\n";
    }
}

// 1つのシナリオを実行する関数
void RunExperiment(const Scenario& sc) {
    std::cout << "\n========================================\n";
    std::cout << "SCENARIO: " << sc.name << "\n";
    std::cout << "Map Size: " << sc.width << "x" << sc.height << "\n";
    std::cout << "========================================\n";

    GridGraph graph(sc.width, sc.height);
    AStar astar;
    LPAStar lpa;
    MAStar mastar; // m-A*を追加
    astar.SetGraph(&graph);
    lpa.SetGraph(&graph);
    mastar.SetGraph(&graph);

    // 1. 初期配置
    for (const auto& obs : sc.initialObstacles) {
        graph.SetObstacle(obs.x, obs.y, obs.blocked);
    }

    std::cout << "\n--- [Step 0] Initial Search ---\n";
    
    astar.Initialize(sc.startNode, sc.goalNode);
    astar.ComputePath();
    astar.GetMetrics().Print("A* Initial");

    lpa.Initialize(sc.startNode, sc.goalNode);
    lpa.ComputePath();
    lpa.GetMetrics().Print("LPA* Initial");

    mastar.Initialize(sc.startNode, sc.goalNode);
    mastar.ComputePath();
    mastar.GetMetrics().Print("m-A* Initial");

    // 各アルゴリズムの初期パスを表示
    std::cout << "\n<A* Initial Path>\n";
    PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);
    
    std::cout << "\n<LPA* Initial Path>\n";
    PrintMap(graph, lpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    std::cout << "\n<m-A* Initial Path>\n";
    PrintMap(graph, mastar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    // 2. 動的なステップ
    for (size_t i = 0; i < sc.steps.size(); ++i) {
        const auto& step = sc.steps[i];
        std::cout << "\n--- [Step " << i + 1 << "] " << step.name << " ---\n";

        for (const auto& ev : step.events) {
            graph.SetObstacle(ev.x, ev.y, ev.blocked);
        }

        // A* Replan
        astar.Initialize(sc.startNode, sc.goalNode);
        astar.ComputePath();
        astar.GetMetrics().Print("A* Replan");

        // LPA* Replan
        lpa.ResetMetrics();
        for (const auto& ev : step.events) {
            int id = ev.y * sc.width + ev.x;
            lpa.NotifyObstacleChange(id, ev.blocked); 
        }
        lpa.ComputePath();
        lpa.GetMetrics().Print("LPA* Replan");

        // m-A* Replan
        mastar.Initialize(sc.startNode, sc.goalNode);
        mastar.ComputePath();
        mastar.GetMetrics().Print("m-A* Replan");

        // 再計画後のパスを表示
        std::cout << "\n<A* Replan Path>\n";
        PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<LPA* Replan Path>\n";
        PrintMap(graph, lpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<m-A* Replan Path>\n";
        PrintMap(graph, mastar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);
    }
}

int main() {
    // 1. 単純な壁 (10x10)
    RunExperiment(Scenario::CreateSimpleWall());
    
    // 2. 罠 (20x20)
    RunExperiment(Scenario::CreateTrap());
    
    // 3. ボトルネック (20x20)
    RunExperiment(Scenario::CreateBottleneck());

    return 0;
}