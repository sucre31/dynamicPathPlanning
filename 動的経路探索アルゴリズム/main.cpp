#include "grid_graph.h"
#include "astar.h"
#include "lpastar.h"
#include "mastar.h"
#include "mlpastar.h"
#include "hpastar.h"
#include "scenario.h"
#include <iostream>
#include <iomanip>
#include <vector>

/**
 * マップ表示用ユーティリティ
 * 障害物(#)、パス(*)、スタート(S)、ゴール(G)を表示します。
 * パスが障害物上にある場合は 'X' を表示します。
 */
void PrintMap(const GridGraph& graph, const std::vector<NodeID>& path, int width, int height, NodeID start, NodeID goal) {
    if (width > 32 || height > 32) {
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
            for (auto pid : path) if (pid == id) isPath = true;

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
 * シナリオ実行
 * A*, LPA*, m-A*, m-LPA*, HPA* を比較します。
 */
void RunExperiment(const Scenario& sc) {
    std::cout << "\n========================================\n";
    std::cout << "SCENARIO: " << sc.name << "\n";
    std::cout << "Map Size: " << sc.width << "x" << sc.height << "\n";
    std::cout << "========================================\n";

    GridGraph graph(sc.width, sc.height);
    AStar astar;
    LPAStar lpa;
    MAStar mastar;
    MLPAStar mlpa;
    HPAStar hpa;

    astar.SetGraph(&graph);
    lpa.SetGraph(&graph);
    mastar.SetGraph(&graph);
    mlpa.SetGraph(&graph);
    hpa.SetGraph(&graph);

    // --- Step 0: 初期探索 ---
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

    // M-A* 初期探索
    //mastar.Initialize(sc.startNode, sc.goalNode);
    //mastar.ComputePath();
    //mastar.GetMetrics().Print("m-A* Initial");
    //std::cout << "[m-A* Preprocess] Time: " << std::fixed << std::setprecision(6)
    //          << mastar.GetPreprocessSeconds() * 1000.0 << " ms\n";

    // m-LPA* 初期探索
    //mlpa.Initialize(sc.startNode, sc.goalNode);
    //mlpa.ComputePath();
    //mlpa.GetMetrics().Print("m-LPA* Initial");

    // HPA* 初期探索
    hpa.Initialize(sc.startNode, sc.goalNode);
    hpa.ComputePath();
    hpa.GetMetrics().Print("HPA* Initial");

    // パス表示
    std::cout << "\n<A* Initial Path>\n";
    PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    std::cout << "\n<LPA* Initial Path>\n";
    PrintMap(graph, lpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    //std::cout << "\n<m-LPA* Initial Path>\n";
    //PrintMap(graph, mlpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    std::cout << "\n<HPA* Initial Path>\n";
    PrintMap(graph, hpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

    // --- Step 1+: 動的イベント ---
    for (size_t i = 0; i < sc.steps.size(); ++i) {
        const auto& step = sc.steps[i];
        std::cout << "\n--- [Step " << i + 1 << "] " << step.name << " ---\n";

        for (const auto& ev : step.events) {
            graph.SetObstacle(ev.x, ev.y, ev.blocked);
        }

        // 1. A* Replan
        astar.Initialize(sc.startNode, sc.goalNode);
        astar.ComputePath();
        astar.GetMetrics().Print("A* Replan");

        // 2. LPA* Replan (動的更新)
        lpa.ResetMetrics();
        for (const auto& ev : step.events) {
            int id = ev.y * sc.width + ev.x;
            lpa.NotifyObstacleChange(id, ev.blocked);
        }
        lpa.ComputePath();
        lpa.GetMetrics().Print("LPA* Replan");

        // 3. m-A* Replan (static method: re-preprocess)
        //mastar.ResetMetrics();
        //mastar.Initialize(sc.startNode, sc.goalNode);
        //mastar.ComputePath();
        //mastar.GetMetrics().Print("m-A* Replan");
        //std::cout << "[m-A* Preprocess] Time: " << std::fixed << std::setprecision(6)
        //         << mastar.GetPreprocessSeconds() * 1000.0 << " ms\n";

        // 4. m-LPA* Replan
        //mlpa.ResetMetrics();
        //for (const auto& ev : step.events) {
        //    int id = ev.y * sc.width + ev.x;
        //    mlpa.NotifyObstacleChange(id, ev.blocked);
        //}
        //mlpa.ComputePath();
        //mlpa.GetMetrics().Print("m-LPA* Replan");

        // 5. HPA* Replan (static method: rebuild abstraction)
        hpa.ResetMetrics();
        hpa.Initialize(sc.startNode, sc.goalNode);
        hpa.ComputePath();
        hpa.GetMetrics().Print("HPA* Replan");

        std::cout << "\n<A* Replan Path>\n";
        PrintMap(graph, astar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<LPA* Replan Path>\n";
        PrintMap(graph, lpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        //std::cout << "\n<m-A* Replan Path>\n";
        //PrintMap(graph, mastar.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        //
        
        //std::cout << "\n<m-LPA* Replan Path>\n";
        //PrintMap(graph, mlpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);

        std::cout << "\n<HPA* Replan Path>\n";
        PrintMap(graph, hpa.GetPath(), sc.width, sc.height, sc.startNode, sc.goalNode);
    }
}

int main() {
    RunExperiment(Scenario::CreateSmallOscillatingShortcut());
    RunExperiment(Scenario::CreateMidScaleBottleneck64());
    RunExperiment(Scenario::CreateLargeDynamicWall128());
    return 0;
}
