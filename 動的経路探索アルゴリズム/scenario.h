#pragma once
#include "common.h"
#include "grid_graph.h"
#include <vector>
#include <string>
#include <random>

// 1つの変更イベント
struct MapEvent {
    int x, y;
    bool blocked;
};

// 1つの実験ステップ
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

    // 1. 単純な壁 (16x16: m-A*対応)
    static Scenario CreateSimpleWall() {
        // m-A* のためにサイズを 10 から 16 (2^4) に変更
        int size = 16;
        Scenario s("Simple Wall", size, size, 0, size * size - 1);
        
        // 初期障害物: 中央付近
        s.initialObstacles.push_back({8, 8, true});

        SimulationStep step1;
        step1.name = "Block Center";
        step1.events.push_back({7, 7, true});
        step1.events.push_back({7, 8, true});
        s.steps.push_back(step1);

        SimulationStep step2;
        step2.name = "Open Path";
        step2.events.push_back({7, 7, false});
        step2.events.push_back({7, 8, false});
        s.steps.push_back(step2);

        return s;
    }

    static Scenario CreateSmallOscillatingShortcut() {
        int size = 16;
        Scenario s("16x16 Oscillating Shortcut", size, size, 0, size*size-1);

        // 基本構造：中央に太い壁
        for (int y = 0; y < size; ++y) {
            if (y == 4 || y == 11) continue; // 2つの隙間
            s.initialObstacles.push_back({8, y, true});
        }

        // 変化1：上の隙間を塞ぐ
        SimulationStep step1;
        step1.name = "Close Upper Gap";
        step1.events.push_back({8, 4, true});
        s.steps.push_back(step1);

        // 変化2：下の隙間を塞いで上を開ける
        SimulationStep step2;
        step2.name = "Switch Gap";
        step2.events.push_back({8, 11, true});
        step2.events.push_back({8, 4, false});
        s.steps.push_back(step2);

        return s;
    }

    static Scenario CreateMidScaleBottleneck64() {
    int size = 64;
    Scenario s("64x64 Moving Bottleneck", size, size, 0, size*size-1);

    int wallX = size / 2;

    // 初期：中央に1つだけ gap
    for (int y = 0; y < size; ++y) {
        if (y == size / 2) continue;
        s.initialObstacles.push_back({wallX, y, true});
    }

    SimulationStep step1;
    step1.name = "Move Central Gap";

    // 元の gap を塞ぐ
    step1.events.push_back({wallX, size / 2, true});

    // 新しい gap を同じ壁列に作る
    step1.events.push_back({wallX, size / 2 + 5, false});

    s.steps.push_back(step1);
    return s;
}


    static Scenario CreateLargeDynamicWall128() {
        int size = 128;
        Scenario s("128x128 Growing Wall", size, size, 0, size*size-1);

        // 初期：薄いランダム障害物
        std::mt19937 gen(42);
        std::uniform_real_distribution<> dis(0,1);

        for(int y=0;y<size;y++){
            for(int x=0;x<size;x++){
                if((x<3&&y<3)||(x>size-4&&y>size-4)) continue;
                if(dis(gen)<0.15)
                    s.initialObstacles.push_back({x,y,true});
            }
        }

        // 動的変化：中央に壁が伸びる
        SimulationStep step1;
        step1.name = "Vertical Wall Growth";
        int mx = size/2;
        for(int y=size/4;y<3*size/4;y++)
            step1.events.push_back({mx,y,true});
        s.steps.push_back(step1);

        return s;
    }



    // 2. U字型トラップ (32x32: m-A*対応)
    static Scenario CreateTrap() {
        int size = 32;
        Scenario s("U-Shape Trap", size, size, 0, size * size - 1); 
        
        // U字の壁を生成
        for(int x = 10; x <= 22; ++x) {
            s.initialObstacles.push_back({x, 20, true}); // 底
        }
        for(int y = 10; y <= 20; ++y) {
            s.initialObstacles.push_back({10, y, true});  // 左壁
            s.initialObstacles.push_back({22, y, true}); // 右壁
        }
        
        SimulationStep step1;
        step1.name = "Close Trap Exit";
        step1.events.push_back({16, 10, true}); // 蓋をする
        s.steps.push_back(step1);

        return s;
    }

    // 3. ボトルネック (32x32)
    static Scenario CreateBottleneck() {
        int size = 32;
        Scenario s("Bottleneck", size, size, 0, size * size - 1);
        
        int wallX = 16;
        for(int y = 0; y < size; ++y) {
            if (y != 16) { // 1箇所だけ空ける
                s.initialObstacles.push_back({wallX, y, true});
            }
        }
        
        SimulationStep step1;
        step1.name = "Block the Gap";
        step1.events.push_back({wallX, 16, true});
        step1.events.push_back({wallX, size - 1, false}); // 遠くに迂回路を作成
        s.steps.push_back(step1);

        SimulationStep step2;
        step2.name = "Block the line";
        step2.events.push_back({wallX - 1, 16, true});
        s.steps.push_back(step2); // 不足していた閉じ括弧を修正

        return s;
    }
    

    // 4. 大規模ランダムマップ (2のべき乗指定を推奨)
    static Scenario CreateLargeRandom(int size, double obstacleProb = 0.2) {
        std::string title = "Random Map (" + std::to_string(size) + "x" + std::to_string(size) + ")";
        Scenario s(title, size, size, 0, size * size - 1);

        std::mt19937 gen(1234);
        std::uniform_real_distribution<> dis(0.0, 1.0);
        std::uniform_int_distribution<> posDis(0, size - 1);

        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                if ((x < 5 && y < 5) || (x > size - 6 && y > size - 6)) continue;
                if (dis(gen) < obstacleProb) {
                    s.initialObstacles.push_back({x, y, true});
                }
            }
        }

        SimulationStep step1;
        step1.name = "Random Obstacles Appearance";
        for (int i = 0; i < 5; ++i) {
            int rx = posDis(gen);
            int ry = posDis(gen);
            if ((rx == 0 && ry == 0) || (rx == size-1 && ry == size-1)) continue;
            step1.events.push_back({rx, ry, true});
        }
        s.steps.push_back(step1);

        return s;
    }

    // 5. 論文の実験結果(Figure 9, 10等)を再現するためのランダムシナリオ
    static Scenario CreatePaperBenchmark(int size = 128, double initialDensity = 0.3) {
        std::string title = "Paper Benchmark (" + std::to_string(size) + "x" + std::to_string(size) + ")";
        // スタートとゴールは対角線上に配置
        Scenario s(title, size, size, 0, size * size - 1);

        std::mt19937 gen(42); // 論文の比較再現のためシードを固定
        std::uniform_real_distribution<> dis(0.0, 1.0);
        std::uniform_int_distribution<> posDis(0, size - 1);

        // 初期状態: ランダムに障害物を配置
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                // スタートとゴールの周辺（d-square最小単位）は空けておく
                if ((x < 2 && y < 2) || (x > size - 3 && y > size - 3)) continue;
                if (dis(gen) < initialDensity) {
                    s.initialObstacles.push_back({x, y, true});
                }
            }
        }

        // 論文で強調されている「動的な変化」：既存のパスを遮断するような変化
        // 画面中央に垂直な壁を一気に作る、あるいは消す
        SimulationStep step1;
        step1.name = "Dynamic Obstacle Growth (Mid-Wall)";
        int midX = size / 2;
        for (int y = size / 4; y < (3 * size / 4); ++y) {
            step1.events.push_back({midX, y, true});
        }
        s.steps.push_back(step1);

        return s;
    }
};