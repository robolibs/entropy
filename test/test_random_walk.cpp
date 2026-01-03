#include <cmath>
#include <doctest/doctest.h>
#include <entropy/entropy.hpp>

TEST_CASE("RandomWalk basic construction") {
    SUBCASE("Default config constructor") {
        entropy::path::RandomWalk walker(100);
        CHECK_NOTHROW(walker.generate());
        CHECK(walker.get_total_steps() == 100);
    }

    SUBCASE("Constructor with seed") {
        entropy::path::RandomWalk walker(100, 42);
        walker.generate();
        CHECK(walker.get_path().size() == 101); // steps + 1 for start point
    }

    SUBCASE("Constructor with config") {
        entropy::path::WalkConfig config;
        config.seed = 1337;
        config.min_speed = 2.0;
        config.max_speed = 5.0;

        entropy::path::RandomWalk walker(50, config);
        walker.generate();

        CHECK(walker.get_speed() >= 2.0);
        CHECK(walker.get_speed() <= 5.0 * 1.025); // max + 2.5% superhuman
    }

    SUBCASE("Invalid total_steps throws") {
        CHECK_THROWS_AS(entropy::path::RandomWalk(0), std::invalid_argument);
        CHECK_THROWS_AS(entropy::path::RandomWalk(-10), std::invalid_argument);
    }
}

TEST_CASE("RandomWalk path generation") {
    entropy::path::WalkConfig config;
    config.seed = 42;
    entropy::path::RandomWalk walker(100, config);
    walker.generate();

    SUBCASE("Path has correct size") {
        CHECK(walker.get_path().size() == 101); // total_steps + 1
    }

    SUBCASE("Start and end points are accessible") {
        auto start = walker.get_start_point();
        auto end = walker.get_end_point();

        CHECK(std::isfinite(start.x));
        CHECK(std::isfinite(start.y));
        CHECK(std::isfinite(end.x));
        CHECK(std::isfinite(end.y));
    }

    SUBCASE("Path points are connected by walker speed") {
        double speed = walker.get_speed();
        const auto &path = walker.get_path();

        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            double dx = std::abs(path.waypoints[i].point.x - path.waypoints[i - 1].point.x);
            double dy = std::abs(path.waypoints[i].point.y - path.waypoints[i - 1].point.y);

            // Each step should move by 0 or speed in each direction
            CHECK((dx < 0.001 || std::abs(dx - speed) < 0.001));
            CHECK((dy < 0.001 || std::abs(dy - speed) < 0.001));
        }
    }
}

TEST_CASE("RandomWalk deterministic behavior") {
    SUBCASE("Same seed produces same path") {
        entropy::path::WalkConfig config;
        config.seed = 12345;

        entropy::path::RandomWalk walker1(50, config);
        entropy::path::RandomWalk walker2(50, config);

        walker1.generate();
        walker2.generate();

        CHECK(walker1.get_speed() == walker2.get_speed());
        CHECK(walker1.get_path().size() == walker2.get_path().size());

        const auto &path1 = walker1.get_path();
        const auto &path2 = walker2.get_path();

        for (size_t i = 0; i < path1.waypoints.size(); ++i) {
            CHECK(path1.waypoints[i].point.x == path2.waypoints[i].point.x);
            CHECK(path1.waypoints[i].point.y == path2.waypoints[i].point.y);
        }
    }

    SUBCASE("Different seeds produce different paths") {
        entropy::path::RandomWalk walker1(50, 111);
        entropy::path::RandomWalk walker2(50, 222);

        walker1.generate();
        walker2.generate();

        // Very unlikely to have same speed and path
        bool different = (walker1.get_speed() != walker2.get_speed()) ||
                         (walker1.get_end_point().x != walker2.get_end_point().x) ||
                         (walker1.get_end_point().y != walker2.get_end_point().y);
        CHECK(different);
    }
}

TEST_CASE("RandomWalk move patterns") {
    SUBCASE("Moore pattern allows 8 directions") {
        entropy::path::WalkConfig config;
        config.seed = 999;
        config.move_pattern = entropy::path::MovePattern::Moore;
        config.random_start = false;

        entropy::path::RandomWalk walker(1000, config);
        walker.generate();

        double speed = walker.get_speed();
        const auto &path = walker.get_path();

        bool has_diagonal = false;
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            double dx = std::abs(path.waypoints[i].point.x - path.waypoints[i - 1].point.x);
            double dy = std::abs(path.waypoints[i].point.y - path.waypoints[i - 1].point.y);

            // Diagonal move: both dx and dy are non-zero
            if (dx > 0.001 && dy > 0.001) {
                has_diagonal = true;
                break;
            }
        }
        CHECK(has_diagonal);
    }

    SUBCASE("Neumann pattern only allows 4 directions") {
        entropy::path::WalkConfig config;
        config.seed = 888;
        config.move_pattern = entropy::path::MovePattern::Neumann;
        config.random_start = false;

        entropy::path::RandomWalk walker(100, config);
        walker.generate();

        double speed = walker.get_speed();
        const auto &path = walker.get_path();

        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            double dx = std::abs(path.waypoints[i].point.x - path.waypoints[i - 1].point.x);
            double dy = std::abs(path.waypoints[i].point.y - path.waypoints[i - 1].point.y);

            // Neumann: only one of dx or dy should be non-zero (no diagonals)
            bool is_cardinal = (dx < 0.001 && dy > 0.001) || (dx > 0.001 && dy < 0.001);
            CHECK(is_cardinal);
        }
    }
}

TEST_CASE("RandomWalk random start") {
    SUBCASE("Random start enabled") {
        entropy::path::WalkConfig config;
        config.seed = 777;
        config.random_start = true;

        entropy::path::RandomWalk walker(100, config);
        walker.generate();

        auto start = walker.get_start_point();
        // With random start, unlikely to be exactly at origin
        bool not_origin = (std::abs(start.x) > 0.001 || std::abs(start.y) > 0.001);
        CHECK(not_origin);
    }

    SUBCASE("Random start disabled") {
        entropy::path::WalkConfig config;
        config.seed = 666;
        config.random_start = false;

        entropy::path::RandomWalk walker(100, config);
        walker.generate();

        auto start = walker.get_start_point();
        CHECK(std::abs(start.x) < 0.001);
        CHECK(std::abs(start.y) < 0.001);
    }
}

TEST_CASE("RandomWalk walker types") {
    SUBCASE("Walker type classification") {
        // Test multiple walkers to cover different speed ranges
        int slow_count = 0, normal_count = 0, fast_count = 0, superhuman_count = 0;

        for (int seed = 0; seed < 100; ++seed) {
            entropy::path::RandomWalk walker(10, seed);

            switch (walker.get_walker_type()) {
            case entropy::path::WalkerType::Slow:
                slow_count++;
                break;
            case entropy::path::WalkerType::Normal:
                normal_count++;
                break;
            case entropy::path::WalkerType::Fast:
                fast_count++;
                break;
            case entropy::path::WalkerType::Superhuman:
                superhuman_count++;
                break;
            }
        }

        // Should have some distribution across types
        CHECK(slow_count > 0);
        CHECK(normal_count > 0);
        CHECK(fast_count > 0);
        // Superhuman is rare (2.5% chance), might be 0
    }

    SUBCASE("Walker type name strings") {
        CHECK(std::string(entropy::path::RandomWalk::walker_type_name(entropy::path::WalkerType::Slow)) ==
              "Slow Walker");
        CHECK(std::string(entropy::path::RandomWalk::walker_type_name(entropy::path::WalkerType::Normal)) ==
              "Normal Walker");
        CHECK(std::string(entropy::path::RandomWalk::walker_type_name(entropy::path::WalkerType::Fast)) ==
              "Fast Walker");
        CHECK(std::string(entropy::path::RandomWalk::walker_type_name(entropy::path::WalkerType::Superhuman)) ==
              "Superhuman");
    }
}

TEST_CASE("RandomWalk speed range") {
    SUBCASE("Speed within configured range") {
        entropy::path::WalkConfig config;
        config.seed = 555;
        config.min_speed = 5.0;
        config.max_speed = 10.0;

        entropy::path::RandomWalk walker(50, config);

        double speed = walker.get_speed();
        CHECK(speed >= 5.0);
        CHECK(speed <= 10.0 * 1.025); // max + 2.5% superhuman bonus
    }

    SUBCASE("Default speed range") {
        entropy::path::RandomWalk walker(50, 444);

        double speed = walker.get_speed();
        CHECK(speed >= 1.0);         // default min
        CHECK(speed <= 3.0 * 1.025); // default max + 2.5%
    }
}

TEST_CASE("WalkSimulation basic functionality") {
    SUBCASE("Construction with multiple walkers") {
        entropy::path::WalkSimulation sim(100, 5);
        CHECK(sim.num_walkers() == 5);
    }

    SUBCASE("Invalid parameters throw") {
        CHECK_THROWS_AS(entropy::path::WalkSimulation(0, 5), std::invalid_argument);
        CHECK_THROWS_AS(entropy::path::WalkSimulation(100, 0), std::invalid_argument);
        CHECK_THROWS_AS(entropy::path::WalkSimulation(-10, 5), std::invalid_argument);
        CHECK_THROWS_AS(entropy::path::WalkSimulation(100, -3), std::invalid_argument);
    }

    SUBCASE("Generate all walkers") {
        entropy::path::WalkSimulation sim(50, 3);
        sim.generate();

        for (size_t i = 0; i < sim.num_walkers(); ++i) {
            CHECK(sim.get_walker(i).get_path().size() == 51);
        }
    }

    SUBCASE("Each walker has different seed") {
        entropy::path::WalkConfig config;
        config.seed = 1000;

        entropy::path::WalkSimulation sim(50, 3, config);
        sim.generate();

        // Different seeds should produce different speeds (very likely)
        double speed0 = sim.get_walker(0).get_speed();
        double speed1 = sim.get_walker(1).get_speed();
        double speed2 = sim.get_walker(2).get_speed();

        bool all_different = (speed0 != speed1) || (speed1 != speed2) || (speed0 != speed2);
        CHECK(all_different);
    }
}

TEST_CASE("WalkSimulation bounds") {
    entropy::path::WalkConfig config;
    config.seed = 123;
    config.random_start = false;

    entropy::path::WalkSimulation sim(100, 5, config);
    sim.generate();

    SUBCASE("Bounds are computed") {
        auto bounds = sim.get_bounds();
        // Just verify bounds are set (size > 0)
        CHECK(bounds.size.x > 0);
        CHECK(bounds.size.y > 0);
    }
}

TEST_CASE("WalkSimulation walker access") {
    entropy::path::WalkSimulation sim(50, 3);

    SUBCASE("Valid index access") {
        CHECK_NOTHROW(sim.get_walker(0));
        CHECK_NOTHROW(sim.get_walker(1));
        CHECK_NOTHROW(sim.get_walker(2));
    }

    SUBCASE("Invalid index throws") {
        CHECK_THROWS_AS(sim.get_walker(3), std::out_of_range);
        CHECK_THROWS_AS(sim.get_walker(100), std::out_of_range);
    }

    SUBCASE("Get all walkers") {
        const auto &walkers = sim.get_walkers();
        CHECK(walkers.size() == 3);
    }
}

TEST_CASE("RandomWalk configuration setters") {
    entropy::path::RandomWalk walker(50, 111);

    SUBCASE("set_seed changes behavior") {
        walker.set_seed(222);
        double speed1 = walker.get_speed();

        walker.set_seed(333);
        double speed2 = walker.get_speed();

        CHECK(speed1 != speed2);
    }

    SUBCASE("set_speed_range") {
        walker.set_speed_range(10.0, 20.0);
        double speed = walker.get_speed();

        CHECK(speed >= 10.0);
        CHECK(speed <= 20.0 * 1.025);
    }

    SUBCASE("set_move_pattern") {
        walker.set_move_pattern(entropy::path::MovePattern::Neumann);
        walker.generate();

        const auto &path = walker.get_path();
        double speed = walker.get_speed();

        // Verify no diagonal moves
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            double dx = std::abs(path.waypoints[i].point.x - path.waypoints[i - 1].point.x);
            double dy = std::abs(path.waypoints[i].point.y - path.waypoints[i - 1].point.y);

            bool is_cardinal = (dx < 0.001 && dy > 0.001) || (dx > 0.001 && dy < 0.001);
            CHECK(is_cardinal);
        }
    }

    SUBCASE("set_random_start") {
        walker.set_random_start(false);
        walker.generate();

        auto start = walker.get_start_point();
        CHECK(std::abs(start.x) < 0.001);
        CHECK(std::abs(start.y) < 0.001);
    }
}
