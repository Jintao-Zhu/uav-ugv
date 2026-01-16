#include <iostream>
#include <vector>
#include <string>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// CoveragePlanner
#include <cgal_definitions.h>
#include <decomposition.h>
#include <visibility_graph.h>
#include <sweep.h>
// #include <coverage_planner.h>

// CGAL
#include <CGAL/convex_hull_2.h>

// ===============================
// 手动指定 RMF 地图路径 & level
// ===============================
static const std::string RMF_YAML_PATH =
    "/home/suda/rmf_ws_2/src/rmf_demos/rmf_demos_maps/maps/airport_terminal/airport_terminal.building.yaml";

static const std::string RMF_LEVEL_NAME =
    "L1";

// ===============================
// 读取 RMF vertices
// ===============================
std::vector<Point_2> load_vertices_from_rmf_yaml(
    const std::string& yaml_path,
    const std::string& level_name)
{
    std::vector<Point_2> points;

    YAML::Node map = YAML::LoadFile(yaml_path);

    auto vertices = map["levels"][level_name]["vertices"];
    if (!vertices || !vertices.IsSequence()) {
        throw std::runtime_error("vertices not found or not a sequence");
    }

    for (const auto& v : vertices) {
        if (!v.IsSequence() || v.size() < 2) {
            continue;
        }

        double x = v[0].as<double>();
        double y = v[1].as<double>();

        points.emplace_back(x, y);
    }

    std::cout << "[RMF] Loaded vertices: "
              << points.size() << std::endl;

    return points;
}

// ===============================
// vertices → PolygonWithHoles
// ===============================
PolygonWithHoles build_polygon_from_vertices(
    const std::vector<Point_2>& points)
{
    if (points.size() < 3) {
        throw std::runtime_error("Not enough points");
    }

    Polygon_2 outer;

    CGAL::convex_hull_2(
        points.begin(),
        points.end(),
        std::back_inserter(outer)
    );

    if (!outer.is_simple()) {
        throw std::runtime_error("Convex hull is not simple");
    }

    std::cout << "[CGAL] Convex hull vertices: "
              << outer.size() << std::endl;

    return PolygonWithHoles(outer);
}

// ===============================
// main
// ===============================
int main()
{
    try {
        std::cout << "[INFO] Loading RMF map:\n  "
                  << RMF_YAML_PATH
                  << "\n  level = "
                  << RMF_LEVEL_NAME << std::endl;

        auto points = load_vertices_from_rmf_yaml(
            RMF_YAML_PATH, RMF_LEVEL_NAME);

        auto pwh = build_polygon_from_vertices(points);

        std::vector<Polygon_2> cells;

        bool ok =
            polygon_coverage_planning::computeBestTCDFromPolygonWithHoles(
                pwh, &cells);

        if (!ok) {
            std::cerr << "[TCD] Decomposition failed\n";
            return -1;
        }

        std::cout << "[TCD] Cells generated: "
                  << cells.size() << std::endl;

        // std::vector<Point_2> verts;
        // verts.reserve(cells[6].size());

        // for (auto it = cells[6].begin(); it != cells[6].end(); ++it) {
        //     verts.push_back(*it);
        // }

        // std::cout << "[DEBUG] cell 0 has " << verts.size() << " vertices\n";

 
        // for (size_t i = 0; i < cells.size(); ++i) {
        //     const auto& cell = cells[i];

        //     std::cout << "\n[CHECK] Cell " << i << std::endl;
        //     std::cout << "  size = " << cell.size() << std::endl;
        //     std::cout << "  is_simple = " << cell.is_simple() << std::endl;
        //     std::cout << "  is_empty = " << cell.is_empty() << std::endl;

        //     if (cell.size() < 3 || !cell.is_simple()) {
        //         std::cout << "  ❌ invalid polygon" << std::endl;
        //     } else {
        //         std::cout << "  ✅ valid polygon" << std::endl;
        //     }
        // }

        // for (const auto& cell : cells)
        // {
        //     auto box = cell.bbox();
        //     std::cout << box.xmin() << " " << box.ymin() << std::endl;
        // }
    }
    
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
