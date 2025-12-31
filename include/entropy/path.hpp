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

        // ============ IMPLEMENTATION ============

        // RandomWalk implementation

        inline RandomWalk::RandomWalk(int total_steps, const WalkConfig &config)
            : total_steps_(total_steps), config_(config), walker_speed_(0.0), rng_(config.seed) {
            if (total_steps <= 0) {
                throw std::invalid_argument("total_steps must be positive");
            }
            init_speed();
        }

        inline RandomWalk::RandomWalk(int total_steps, int seed) : RandomWalk(total_steps, WalkConfig(seed)) {}

        inline void RandomWalk::init_speed() { walker_speed_ = get_random_speed(); }

        inline double RandomWalk::get_random_speed() {
            // Speed range with 2.5% superhuman bonus possibility
            double superhuman_bonus = config_.max_speed * 0.025;
            std::uniform_real_distribution<double> dist(config_.min_speed, config_.max_speed + superhuman_bonus);
            return dist(rng_);
        }

        inline datapod::Point RandomWalk::get_random_startpoint() {
            double range = std::sqrt(static_cast<double>(total_steps_)) * config_.start_range_factor;
            std::uniform_real_distribution<double> dist(-range, range);
            return datapod::Point{dist(rng_), dist(rng_), 0.0};
        }

        inline Direction RandomWalk::get_random_direction() {
            if (config_.move_pattern == MovePattern::Moore) {
                // 8 directions
                std::uniform_int_distribution<int> dist(0, 7);
                return static_cast<Direction>(dist(rng_));
            } else {
                // 4 directions (Neumann): N, E, S, W (indices 0, 2, 4, 6)
                std::uniform_int_distribution<int> dist(0, 3);
                int directions[] = {0, 2, 4, 6}; // North, East, South, West
                return static_cast<Direction>(directions[dist(rng_)]);
            }
        }

        inline void RandomWalk::plan_next_step(Direction direction, const datapod::Point &current) {
            double dx = 0.0, dy = 0.0;

            switch (direction) {
            case Direction::North:
                dy = walker_speed_;
                break;
            case Direction::Northeast:
                dx = walker_speed_;
                dy = walker_speed_;
                break;
            case Direction::East:
                dx = walker_speed_;
                break;
            case Direction::Southeast:
                dx = walker_speed_;
                dy = -walker_speed_;
                break;
            case Direction::South:
                dy = -walker_speed_;
                break;
            case Direction::Southwest:
                dx = -walker_speed_;
                dy = -walker_speed_;
                break;
            case Direction::West:
                dx = -walker_speed_;
                break;
            case Direction::Northwest:
                dx = -walker_speed_;
                dy = walker_speed_;
                break;
            }

            datapod::Point new_point{current.x + dx, current.y + dy, current.z};
            path_.waypoints.push_back(datapod::Pose{new_point, datapod::Quaternion{}});
        }

        inline void RandomWalk::generate() {
            path_.waypoints.clear();

            // Set starting point
            datapod::Point start_point;
            if (config_.random_start) {
                start_point = get_random_startpoint();
            } else {
                start_point = datapod::Point{0.0, 0.0, 0.0};
            }

            // Add starting pose
            path_.waypoints.push_back(datapod::Pose{start_point, datapod::Quaternion{}});

            // Generate path
            for (int step = 1; step <= total_steps_; ++step) {
                Direction dir = get_random_direction();
                plan_next_step(dir, path_.waypoints[path_.waypoints.size() - 1].point);
            }
        }

        inline const datapod::Path &RandomWalk::get_path() const { return path_; }

        inline datapod::Path &RandomWalk::get_path() { return path_; }

        inline double RandomWalk::get_speed() const { return walker_speed_; }

        inline WalkerType RandomWalk::get_walker_type() const {
            double range = config_.max_speed - config_.min_speed;
            double threshold = range * 0.25;

            if (walker_speed_ < config_.min_speed + threshold) {
                return WalkerType::Slow;
            } else if (walker_speed_ >= config_.min_speed + threshold &&
                       walker_speed_ < config_.max_speed - threshold) {
                return WalkerType::Normal;
            } else if (walker_speed_ >= config_.max_speed - threshold && walker_speed_ <= config_.max_speed) {
                return WalkerType::Fast;
            } else {
                return WalkerType::Superhuman;
            }
        }

        inline datapod::Point RandomWalk::get_start_point() const {
            if (path_.waypoints.empty()) {
                return datapod::Point{};
            }
            return path_.waypoints[0].point;
        }

        inline datapod::Point RandomWalk::get_end_point() const {
            if (path_.waypoints.empty()) {
                return datapod::Point{};
            }
            return path_.waypoints[path_.waypoints.size() - 1].point;
        }

        inline void RandomWalk::set_seed(int seed) {
            config_.seed = seed;
            rng_.seed(seed);
            init_speed();
        }

        inline void RandomWalk::set_speed_range(double min_speed, double max_speed) {
            config_.min_speed = min_speed;
            config_.max_speed = max_speed;
            init_speed();
        }

        inline void RandomWalk::set_move_pattern(MovePattern pattern) { config_.move_pattern = pattern; }

        inline void RandomWalk::set_random_start(bool random_start) { config_.random_start = random_start; }

        inline void RandomWalk::set_start_range_factor(double factor) { config_.start_range_factor = factor; }

        inline int RandomWalk::get_total_steps() const { return total_steps_; }

        const char *RandomWalk::walker_type_name(WalkerType type) {
            switch (type) {
            case WalkerType::Slow:
                return "Slow Walker";
            case WalkerType::Normal:
                return "Normal Walker";
            case WalkerType::Fast:
                return "Fast Walker";
            case WalkerType::Superhuman:
                return "Superhuman";
            default:
                return "Unknown";
            }
        }

        // WalkSimulation implementation

        inline WalkSimulation::WalkSimulation(int total_steps, int num_walkers, const WalkConfig &config)
            : total_steps_(total_steps), num_walkers_(num_walkers), config_(config) {
            if (total_steps <= 0) {
                throw std::invalid_argument("total_steps must be positive");
            }
            if (num_walkers <= 0) {
                throw std::invalid_argument("num_walkers must be positive");
            }

            // Create walkers with different seeds
            walkers_.reserve(num_walkers);
            for (int i = 0; i < num_walkers; ++i) {
                WalkConfig walker_config = config;
                walker_config.seed = config.seed + i; // Different seed for each walker
                walkers_.emplace_back(total_steps, walker_config);
            }
        }

        inline void WalkSimulation::generate() {
            for (auto &walker : walkers_) {
                walker.generate();
            }
        }

        inline const std::vector<RandomWalk> &WalkSimulation::get_walkers() const { return walkers_; }

        inline std::vector<RandomWalk> &WalkSimulation::get_walkers() { return walkers_; }

        inline const RandomWalk &WalkSimulation::get_walker(size_t index) const {
            if (index >= walkers_.size()) {
                throw std::out_of_range("walker index out of range");
            }
            return walkers_[index];
        }

        inline RandomWalk &WalkSimulation::get_walker(size_t index) {
            if (index >= walkers_.size()) {
                throw std::out_of_range("walker index out of range");
            }
            return walkers_[index];
        }

        inline size_t WalkSimulation::num_walkers() const { return walkers_.size(); }

        inline datapod::Box WalkSimulation::get_bounds() const {
            if (walkers_.empty()) {
                return datapod::Box{};
            }

            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();

            for (const auto &walker : walkers_) {
                const auto &path = walker.get_path();
                for (const auto &pose : path.waypoints) {
                    min_x = std::min(min_x, pose.point.x);
                    max_x = std::max(max_x, pose.point.x);
                    min_y = std::min(min_y, pose.point.y);
                    max_y = std::max(max_y, pose.point.y);
                }
            }

            // Create box using Pose (at center) and Size (dimensions)
            double center_x = (min_x + max_x) / 2.0;
            double center_y = (min_y + max_y) / 2.0;
            datapod::Pose center_pose{datapod::Point{center_x, center_y, 0.0}, datapod::Quaternion{}};
            datapod::Size dimensions{max_x - min_x, max_y - min_y, 0.0};
            return datapod::Box{center_pose, dimensions};
        }

    } // namespace path
} // namespace entropy
