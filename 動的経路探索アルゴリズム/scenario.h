#pragma once
#include "common.h"
#include "grid_graph.h"
#include <vector>
#include <string>
#include <random>

// 1つの変更イベンチE
struct MapEvent {
    int x, y;
    bool blocked;
};

// 1つの実験スチE��チE
struct SimulationStep {
    std::string name;
    std::vector<MapEvent> events;
};

class Scenario {
public:
    std::string name;
    int width, height;
    NodeID startNode, goalNode;
    
    std::vector<MapEvent> initialObstacles;
    std::vector<SimulationStep> steps;

    Scenario(std::string n, int w, int h, NodeID s, NodeID g) 
        : name(n), width(w), height(h), startNode(s), goalNode(g) {}

    // 9. 16x16 螺旋迷路 (Spiral Maze) [完�E修正牁E
    // ロジチE��変更: 壁E�E置ではなく「通路を掘る」方式に変更し、確実にパスを通します、E
    // 外周から中忁E��、ぐるぐると回る一本道を生�Eします、E
    static Scenario CreateSpiralMaze16() {
        int size = 16;
        // スターチE0,0) -> ゴール(中央 7,8)
        Scenario s("16x16 Spiral Maze", size, size, 0, 7 + 8 * size);

        // 1. まず�E埋めする
        for(int y=0; y<size; ++y) {
            for(int x=0; x<size; ++x) {
                s.initialObstacles.push_back({x, y, true});
            }
        }

        // 2. 一筁E��きで通路を掘めE(Carve Path)
        // 座標リストを作り、それを「障害物なぁEfalse)」に設定すめE
        auto carve = [&](int x1, int y1, int x2, int y2) {
            if (x1 == x2) { // 縦掘り
                int min = std::min(y1, y2);
                int max = std::max(y1, y2);
                for(int y=min; y<=max; ++y) s.initialObstacles.push_back({x1, y, false});
            } else { // 横掘り
                int min = std::min(x1, x2);
                int max = std::max(x1, x2);
                for(int x=min; x<=max; ++x) s.initialObstacles.push_back({x, y1, false});
            }
        };

        // 螺旋状に通路を作�E
        // (0,0) -> (15,0) -> (15,15) -> (0,15) -> (0,2) -> (13,2) -> (13,13) -> (2,13) -> (2,4) ...
        // 壁�E厚さを確保するため、Eマス間隔で折り返しまぁE
        
        carve(0, 0, 15, 0);   // 上辺 (右へ)
        carve(15, 0, 15, 15); // 右辺 (下へ)
        carve(15, 15, 0, 15); // 下辺 (左へ)
        carve(0, 15, 0, 2);   // 左辺 (上へ) ※(0,0)までは戻らなぁE

        carve(0, 2, 13, 2);   // 冁E�Eへ (右へ)
        carve(13, 2, 13, 13); // (下へ)
        carve(13, 13, 2, 13); // (左へ)
        carve(2, 13, 2, 4);   // (上へ)

        carve(2, 4, 11, 4);   // さらに冁E�E
        carve(11, 4, 11, 11); 
        carve(11, 11, 4, 11);
        carve(4, 11, 4, 6);

        carve(4, 6, 9, 6);    // ゴール付迁E
        carve(9, 6, 9, 8);
        carve(9, 8, 7, 8);    // ゴール(7,8)へ到遁E

        // 3. 動的イベンチE
        // 中央への入り口付近を一瞬塞ぐ
        SimulationStep step1;
        step1.name = "Block Inner Spiral";
        step1.events.push_back({9, 6, true}); // 通路を�E断
        s.steps.push_back(step1);

        SimulationStep step2;
        step2.name = "Open Inner Spiral";
        step2.events.push_back({9, 6, false}); // 再開
        s.steps.push_back(step2);

        return s;
    }

    // 10. 64x64 疎な森 (Sparse Forest)
    // HPA*の弱点�E�準最適性�E�を突くマップ、E
    // 木、E�E間を「直線的」に抜けるA*に対し、HPA*は「クラスタ墁E��」を経由するためカクカクします、E
    static Scenario CreateForest64() {
        int size = 64;
        Scenario s("64x64 Sparse Forest", size, size, 0, size * size - 1);

        std::mt19937 gen(999);
        std::uniform_real_distribution<> dis(0.0, 1.0);

        // スタート�Eゴール周辺は確実に空ける
        auto isSafe = [&](int x, int y) {
            if (x < 5 && y < 5) return true;
            if (x > size-6 && y > size-6) return true;
            return false;
        };

        // ランダムに木を�E置 (寁E��12%程度がパスを消さずに邪魔する絶妙なライン)
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                if (isSafe(x, y)) continue;
                if (dis(gen) < 0.12) {
                    s.initialObstacles.push_back({x, y, true});
                }
            }
        }

        // 動的イベンチE 中央エリアで植林活勁E(一気に木が増えめE
        SimulationStep step1;
        step1.name = "Sudden Growth";
        for (int i = 0; i < 50; ++i) {
             int rx = 20 + (int)(dis(gen) * 24); // 中央 24x24 エリア
             int ry = 20 + (int)(dis(gen) * 24);
             step1.events.push_back({rx, ry, true});
        }
        s.steps.push_back(step1);

        return s;
    }

    // 11. 32x32 迷宮 (Labyrinth)
    // 典型的な迷路構造。�E岐が多く、ヒューリスチE��チE��が効きにくい、E
    static Scenario CreateLabyrinth32() {
        int size = 32;
        Scenario s("32x32 Labyrinth", size, size, 1 + size, (size-2) + (size-2)*size); // (1,1) -> (30,30)

        // 1. 全て壁にする
        for(int y=0; y<size; ++y) 
            for(int x=0; x<size; ++x) 
                s.initialObstacles.push_back({x, y, true});

        // 2. 棒倒し況E簡昁Eで道を掘る
        // (奁E��座標を通路にする)
        for(int y=1; y<size-1; y+=2) {
            for(int x=1; x<size-1; x+=2) {
                s.initialObstacles.push_back({x, y, false}); // 柱を削めE
                
                // 隣接する壁をランダムに1つ削って連結すめE
                std::mt19937 gen(x + y * size);
                int dir = gen() % 2; // 右か下へ道を伸ばぁE
                if (dir == 0 && x+1 < size-1) s.initialObstacles.push_back({x+1, y, false});
                if (dir == 1 && y+1 < size-1) s.initialObstacles.push_back({x, y+1, false});
            }
        }
        
        // 外周は壁�Eまま維持されてぁE��はぁE

        // 動的イベンチE 壁を破壊してショートカチE��作�E
        SimulationStep step1;
        step1.name = "Break Wall (Shortcut)";
        // 中央付近�E壁を壊す
        step1.events.push_back({16, 15, false});
        step1.events.push_back({16, 16, false});
        step1.events.push_back({16, 17, false});
        s.steps.push_back(step1);

        return s;
    }

    // --- New scenarios for benchmarking ---

    // A* is fast: mostly open field with a few sparse obstacles.
    static Scenario CreateOpenField32() { return CreateOpenField(32); }
    static Scenario CreateOpenField64() { return CreateOpenField(64); }
    static Scenario CreateOpenField128() { return CreateOpenField(128); }

    // HPA* suboptimal: entrances are wide but represented by single transition.
    static Scenario CreateHPAStarSuboptimal32() { return CreateHPAStarSuboptimal(32); }
    static Scenario CreateHPAStarSuboptimal64() { return CreateHPAStarSuboptimal(64); }
    static Scenario CreateHPAStarSuboptimal128() { return CreateHPAStarSuboptimal(128); }

    // LPA* shines: small localized changes that flip a shortcut.
    static Scenario CreateDynamicShortcut32() { return CreateDynamicShortcut(32); }
    static Scenario CreateDynamicShortcut64() { return CreateDynamicShortcut(64); }
    static Scenario CreateDynamicShortcut128() { return CreateDynamicShortcut(128); }

private:
    static Scenario CreateOpenField(int size) {
        Scenario s("OpenField", size, size, 0, size * size - 1);
        // Sparse obstacles away from the main diagonal
        for (int y = 2; y < size - 2; y += 6) {
            for (int x = 2; x < size - 2; x += 9) {
                if ((x + y) % 2 == 0) s.initialObstacles.push_back({x, y, true});
            }
        }
        return s;
    }

    static Scenario CreateHPAStarSuboptimal(int size) {
        Scenario s("HPAStarSuboptimal", size, size, 0, size * size - 1);

        int mid = size / 2;
        // Solid vertical wall except two entrances (each width 5)
        for (int y = 0; y < size; ++y) {
            s.initialObstacles.push_back({mid, y, true});
        }
        // Entrance near top
        for (int y = 2; y <= 6 && y < size; ++y) s.initialObstacles.push_back({mid, y, false});
        // Entrance near bottom
        for (int y = size - 7; y <= size - 3; ++y) s.initialObstacles.push_back({mid, y, false});

        // Add a small blocker around the mid-height to penalize mid transitions
        int blockY = size / 2;
        for (int x = mid + 1; x <= mid + 3 && x < size - 1; ++x) {
            s.initialObstacles.push_back({x, blockY, true});
        }
        return s;
    }

    static Scenario CreateDynamicShortcut(int size) {
        Scenario s("DynamicShortcut", size, size, 0, size * size - 1);

        int mid = size / 2;
        // Two parallel vertical walls with two gaps that toggle
        for (int y = 0; y < size; ++y) {
            s.initialObstacles.push_back({mid - 2, y, true});
            s.initialObstacles.push_back({mid + 2, y, true});
        }
        // Initial gaps
        for (int y = 2; y <= 4 && y < size; ++y) {
            s.initialObstacles.push_back({mid - 2, y, false});
            s.initialObstacles.push_back({mid + 2, y, false});
        }
        for (int y = size - 5; y <= size - 3; ++y) {
            s.initialObstacles.push_back({mid - 2, y, false});
            s.initialObstacles.push_back({mid + 2, y, false});
        }

        SimulationStep step1;
        step1.name = "Close Upper Gap";
        for (int y = 2; y <= 4 && y < size; ++y) {
            step1.events.push_back({mid - 2, y, true});
            step1.events.push_back({mid + 2, y, true});
        }
        s.steps.push_back(step1);

        SimulationStep step2;
        step2.name = "Open Upper Gap / Close Lower Gap";
        for (int y = 2; y <= 4 && y < size; ++y) {
            step2.events.push_back({mid - 2, y, false});
            step2.events.push_back({mid + 2, y, false});
        }
        for (int y = size - 5; y <= size - 3; ++y) {
            step2.events.push_back({mid - 2, y, true});
            step2.events.push_back({mid + 2, y, true});
        }
        s.steps.push_back(step2);

        return s;
    }
};