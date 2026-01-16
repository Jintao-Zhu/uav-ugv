#include <iostream>
#include <vector>

// CoveragePlanner
#include <cgal_definitions.h>
#include <sweep.h>
#include <visibility_graph.h>

using polygon_coverage_planning::visibility_graph::VisibilityGraph;

int main()
{
    std::cout << "[INFO] Cell → sweep test\n";

    Polygon_2 cell;
    cell.push_back(Point_2(0, 0));
    cell.push_back(Point_2(10, 0));
    cell.push_back(Point_2(10, 5));
    cell.push_back(Point_2(0, 5));

    VisibilityGraph graph(cell);

    std::vector<Point_2> waypoints;

    bool ok = polygon_coverage_planning::computeSweep(
        cell,
        graph,
        FT(1.0),           // sweep spacing
        Direction_2(1, 0),  // x direction
        true,
        &waypoints
    );

    if (!ok) {
        std::cerr << "[SWEEP] Failed\n";
        return -1;
    }

    std::cout << "[SWEEP] Waypoints:\n";
    for (auto& p : waypoints) {
        std::cout << p.x() << " " << p.y() << "\n";
    }

    return 0;
}
