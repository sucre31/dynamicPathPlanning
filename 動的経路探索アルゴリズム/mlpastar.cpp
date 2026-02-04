#include "mlpastar.h"

void MLPAStar::Initialize(NodeID start, NodeID goal) {
    metrics.Reset();

    boundary = ExtractBoundary(grid, start, goal);
    beamlets.clear();
    BuildBeamlets(grid, boundary, beamlets);

    absGraph = std::make_unique<AbstractGraph>(boundary, &beamlets);

    lpa.SetGraph(absGraph.get());
    lpa.Initialize(start, goal);
}

bool MLPAStar::ComputePath() {
    if (!lpa.ComputePath()) return false;
    ReconstructPath();
    metrics = lpa.GetMetrics();
    return true;
}

void MLPAStar::ReconstructPath() {
    fullPath.clear();
    auto absPath = lpa.GetPath();
    if (absPath.empty()) return;

    for (size_t i = 0; i + 1 < absPath.size(); ++i) {
        auto it = beamlets.find({absPath[i], absPath[i+1]});
        if (it == beamlets.end()) return;

        const auto& p = it->second.path;
        if (!fullPath.empty())
            fullPath.insert(fullPath.end(), p.begin() + 1, p.end());
        else
            fullPath.insert(fullPath.end(), p.begin(), p.end());
    }
}

static std::vector<NodeID>
ExtractBoundary(const GridGraph* grid, NodeID start, NodeID goal) {
    std::vector<NodeID> b;
    int w = grid->GetWidth();
    int h = grid->GetHeight();

    auto add = [&](int x,int y){
        if (!grid->IsObstacle(x,y))
            b.push_back(y*w+x);
    };

    for(int x=0;x<w;x++){ add(x,0); add(x,h-1); }
    for(int y=1;y<h-1;y++){ add(0,y); add(w-1,y); }

    b.push_back(start);
    b.push_back(goal);

    std::sort(b.begin(), b.end());
    b.erase(std::unique(b.begin(), b.end()), b.end());
    return b;
}
