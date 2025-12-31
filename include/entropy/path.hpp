// Random walk path generator
// Based on random-walk simulation concepts

#pragma once

#include <cmath>
#include <datapod/datapod.hpp>
#include <random>

namespace entropy {
    namespace path {

        // Movement pattern types
        enum class MovePattern {
            Neumann, // 4 directions (N, S, E, W)
            Moore    // 8 directions (includes diagonals)
        };

        // Direction enum for movement
        enum class Direction { North, Northeast, East, Southeast, South, Southwest, West, Northwest };

        // Walker type based on speed
        enum class WalkerType { Slow, Normal, Fast, Superhuman };

        // Configuration for random walk generation
        struct WalkConfig {
            int seed = 1337;
            double min_speed = 1.0;
            double max_speed = 3.0;
            MovePattern move_pattern = MovePattern::Moore;
            bool random_start = true;
            double start_range_factor = 1.0; // multiplied by sqrt(steps) for start range

            WalkConfig() = default;
            WalkConfig(int seed_) : seed(seed_) {}
        };

        // Random Walk Generator class
        class RandomWalk {
          public:
            RandomWalk(int total_steps, const WalkConfig &config = WalkConfig());
            RandomWalk(int total_steps, int seed);

            // Generate the random walk path
            void generate();

            // Get the generated path
            const datapod::Path &get_path() const;
            datapod::Path &get_path();

            // Get walker properties
            double get_speed() const;
            WalkerType get_walker_type() const;
            datapod::Point get_start_point() const;
            datapod::Point get_end_point() const;

            // Configuration setters (must be called before generate())
            void set_seed(int seed);
            void set_speed_range(double min_speed, double max_speed);
            void set_move_pattern(MovePattern pattern);
            void set_random_start(bool random_start);
            void set_start_range_factor(double factor);

            // Get total steps
            int get_total_steps() const;

            // Static helper to get string name of walker type
            static const char *walker_type_name(WalkerType type);

          private:
            int total_steps_;
            WalkConfig config_;
            double walker_speed_;
            datapod::Path path_;
            std::mt19937 rng_;

            void init_speed();
            double get_random_speed();
            datapod::Point get_random_startpoint();
            Direction get_random_direction();
            void plan_next_step(Direction direction, const datapod::Point &current);
        };

        // Multi-walker simulation
        class WalkSimulation {
          public:
            WalkSimulation(int total_steps, int num_walkers, const WalkConfig &config = WalkConfig());

            // Generate all walks
            void generate();

            // Get all generated paths
            const std::vector<RandomWalk> &get_walkers() const;
            std::vector<RandomWalk> &get_walkers();

            // Get a specific walker
            const RandomWalk &get_walker(size_t index) const;
            RandomWalk &get_walker(size_t index);

            // Get number of walkers
            size_t num_walkers() const;

            // Get bounding box of all paths
            datapod::Box get_bounds() const;

          private:
            int total_steps_;
            int num_walkers_;
            WalkConfig config_;
            std::vector<RandomWalk> walkers_;
        };

    } // namespace path
} // namespace entropy
