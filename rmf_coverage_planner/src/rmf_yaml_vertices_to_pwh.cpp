#include <iostream>
#include <vector>
#include <string>
#include <fstream>

// CGAL + CoveragePlanner
#include <cgal_definitions.h>
#include <decomposition.h>
#include <visibility_graph.h>
#include <sweep.h>

// CGAL
#include <CGAL/Polygon_2.h>

// ===============================
// Gazebo 中确认过的矩形地图（单位：米）
//
// 左上角:  (0,   0)
// 左下角:  (0,  -64)
// 右上角:  (282, 0)
// 右下角:  (282,-64)
// ===============================
static constexpr double MAP_MIN_X = 0.0;
static constexpr double MAP_MAX_X = 282.0;
static constexpr double MAP_MIN_Y = -64.0;
static constexpr double MAP_MAX_Y = 0.0;

static const double GAZEBO_ORIGIN_X = 40.0;
static const double GAZEBO_ORIGIN_Y = -40.0;

// 输出 waypoint 文件
static const std::string OUTPUT_WAYPOINT_FILE =
    "/home/suda/drone_ugv_ws/src/rmf_coverage_planner/data/waypoints.csv";

// 固定飞行高度（PX4 NED，向上为负）
static constexpr double FLIGHT_HEIGHT = -7.0;

// sweep 间距（米）
static constexpr int SWEEP_STEP = 10;

// ===============================
// 构造矩形 PolygonWithHoles
// ===============================
PolygonWithHoles build_rectangle_polygon()
{
    Polygon_2 outer;

    // ⚠️ CGAL 要求逆时针（CCW）
    outer.push_back(Point_2(MAP_MIN_X, MAP_MAX_Y)); // 左上
    outer.push_back(Point_2(MAP_MAX_X, MAP_MAX_Y)); // 右上
    outer.push_back(Point_2(MAP_MAX_X, MAP_MIN_Y)); // 右下
    outer.push_back(Point_2(MAP_MIN_X, MAP_MIN_Y)); // 左下

    if (!outer.is_simple()) {
        throw std::runtime_error("Rectangle polygon is not simple");
    }

    std::cout << "[MAP] Using hard-coded rectangle\n";
    std::cout << "      X: [" << MAP_MIN_X << ", " << MAP_MAX_X << "]\n";
    std::cout << "      Y: [" << MAP_MIN_Y << ", " << MAP_MAX_Y << "]\n";

    return PolygonWithHoles(outer);
}

// ===============================
// main
// ===============================
int main()
{
    try {
        std::cout << "[INFO] Generating coverage waypoints from rectangle map\n";

        // 1️⃣ 构造 PolygonWithHoles（矩形）
        auto pwh = build_rectangle_polygon();

        // 2️⃣ TCD 分解
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

        std::vector<Point_2> all_waypoints;

        // 3️⃣ 对每个 cell 生成 sweep
        for (size_t i = 0; i < cells.size(); ++i) {

            std::cout << "\n[CELL " << i << "]\n";
            std::cout << "  vertices = "
                      << cells[i].size() << std::endl;

            if (cells[i].size() < 3) {
                std::cout << "  [SKIP] degenerate cell\n";
                continue;
            }

            // 3.1 选择 sweep 方向
            Direction_2 sweep_dir(0.0,-1.0);
            // polygon_coverage_planning::findBestSweepDir(
            //     cells[i], &sweep_dir);

            // 3.2 visibility graph
            polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(
                cells[i]);

            // 3.3 计算 sweep
            std::vector<Point_2> waypoints;
            bool sweep_ok =
                polygon_coverage_planning::computeSweep(
                    cells[i],
                    vis_graph,
                    SWEEP_STEP,
                    sweep_dir,
                    true,
                    &waypoints);

            if (!sweep_ok) {
                std::cerr << "  [SWEEP] failed\n";
                continue;
            }

            std::cout << "  [SWEEP] waypoints = "
                      << waypoints.size() << std::endl;

            all_waypoints.insert(
                all_waypoints.end(),
                waypoints.begin(),
                waypoints.end());
        }

        // 4️⃣ 保存 CSV（PX4 可直接读取）
        std::ofstream ofs(OUTPUT_WAYPOINT_FILE);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to open output file");
        }

        ofs << "# x, y, z (PX4 NED, meters)\n";

        // PX4与Gazebo存在坐标映射，不能直接写入，需进行变换
        for (const auto& p : all_waypoints) {
            ofs << CGAL::to_double(p.y()) + GAZEBO_ORIGIN_X << ", "
                << CGAL::to_double(p.x()) + GAZEBO_ORIGIN_Y << ", "
                << FLIGHT_HEIGHT << "\n";
        }

        ofs.close();

        std::cout << "\n[OK] Waypoints saved to:\n"
                  << OUTPUT_WAYPOINT_FILE << std::endl;

        std::cout << "[OK] Total waypoints: "
                  << all_waypoints.size() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
