#include <iostream>
#include <vector>
#include <string>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// CGAL + CoveragePlanner
#include <cgal_definitions.h>
#include <decomposition.h>
#include <visibility_graph.h>
#include <sweep.h>

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
// 读取 RMF vertices（保持原样）
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
// （保持你现在的 convex hull 逻辑）
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

        // 1️⃣ RMF → vertices
        auto points = load_vertices_from_rmf_yaml(
            RMF_YAML_PATH, RMF_LEVEL_NAME);

        // 2️⃣ vertices → PolygonWithHoles
        auto pwh = build_polygon_from_vertices(points);

        // 3️⃣ TCD decomposition
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

        // 4️⃣ 对每个 cell 生成 sweep
        int sweep_step = 5;  // 间距，先小一点保证成功

        for (size_t i = 0; i < cells.size(); ++i) {

            std::cout << "\n[CELL " << i << "]\n";
            std::cout << "  vertices = "
                      << cells[i].size() << std::endl;

            if (cells[i].size() < 3) {
                std::cout << "  [SKIP] degenerate cell\n";
                continue;
            }

            // 4.1 选择 sweep 方向
            Direction_2 sweep_dir;
            polygon_coverage_planning::findBestSweepDir(
                cells[i], &sweep_dir);

            // 4.2 构建 visibility graph
            polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(
                cells[i]);

            // 4.3 计算 sweep
            std::vector<Point_2> waypoints;
            bool sweep_ok =
                polygon_coverage_planning::computeSweep(
                    cells[i],
                    vis_graph,
                    sweep_step,
                    sweep_dir,
                    true,
                    &waypoints);

            if (!sweep_ok) {
                std::cerr << "  [SWEEP] failed\n";
                continue;
            }

            // 4.4 输出 sweep 点
            std::cout << "  [SWEEP] waypoints = "
                      << waypoints.size() << std::endl;

            for (const auto& p : waypoints) {
                std::cout << "    "
                          << CGAL::to_double(p.x()) << " "
                          << CGAL::to_double(p.y()) << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
