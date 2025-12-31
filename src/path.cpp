// Random walk path generator implementation

#include "entropy/path.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace entropy {
    namespace path {

        // RandomWalk implementation

        RandomWalk::RandomWalk(int total_steps, const WalkConfig &config)
            : total_steps_(total_steps), config_(config), walker_speed_(0.0), rng_(config.seed) {
            if (total_steps <= 0) {
                throw std::invalid_argument("total_steps must be positive");
            }
            init_speed();
        }

        RandomWalk::RandomWalk(int total_steps, int seed) : RandomWalk(total_steps, WalkConfig(seed)) {}

        void RandomWalk::init_speed() { walker_speed_ = get_random_speed(); }

        double RandomWalk::get_random_speed() {
            // Speed range with 2.5% superhuman bonus possibility
            double superhuman_bonus = config_.max_speed * 0.025;
            std::uniform_real_distribution<double> dist(config_.min_speed, config_.max_speed + superhuman_bonus);
            return dist(rng_);
        }

        datapod::Point RandomWalk::get_random_startpoint() {
            double range = std::sqrt(static_cast<double>(total_steps_)) * config_.start_range_factor;
            std::uniform_real_distribution<double> dist(-range, range);
            return datapod::Point{dist(rng_), dist(rng_), 0.0};
        }

        Direction RandomWalk::get_random_direction() {
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

        void RandomWalk::plan_next_step(Direction direction, const datapod::Point &current) {
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

        void RandomWalk::generate() {
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

        const datapod::Path &RandomWalk::get_path() const { return path_; }

        datapod::Path &RandomWalk::get_path() { return path_; }

        double RandomWalk::get_speed() const { return walker_speed_; }

        WalkerType RandomWalk::get_walker_type() const {
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

        datapod::Point RandomWalk::get_start_point() const {
            if (path_.waypoints.empty()) {
                return datapod::Point{};
            }
            return path_.waypoints[0].point;
        }

        datapod::Point RandomWalk::get_end_point() const {
            if (path_.waypoints.empty()) {
                return datapod::Point{};
            }
            return path_.waypoints[path_.waypoints.size() - 1].point;
        }

        void RandomWalk::set_seed(int seed) {
            config_.seed = seed;
            rng_.seed(seed);
            init_speed();
        }

        void RandomWalk::set_speed_range(double min_speed, double max_speed) {
            config_.min_speed = min_speed;
            config_.max_speed = max_speed;
            init_speed();
        }

        void RandomWalk::set_move_pattern(MovePattern pattern) { config_.move_pattern = pattern; }

        void RandomWalk::set_random_start(bool random_start) { config_.random_start = random_start; }

        void RandomWalk::set_start_range_factor(double factor) { config_.start_range_factor = factor; }

        int RandomWalk::get_total_steps() const { return total_steps_; }

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

        WalkSimulation::WalkSimulation(int total_steps, int num_walkers, const WalkConfig &config)
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

        void WalkSimulation::generate() {
            for (auto &walker : walkers_) {
                walker.generate();
            }
        }

        const std::vector<RandomWalk> &WalkSimulation::get_walkers() const { return walkers_; }

        std::vector<RandomWalk> &WalkSimulation::get_walkers() { return walkers_; }

        const RandomWalk &WalkSimulation::get_walker(size_t index) const {
            if (index >= walkers_.size()) {
                throw std::out_of_range("walker index out of range");
            }
            return walkers_[index];
        }

        RandomWalk &WalkSimulation::get_walker(size_t index) {
            if (index >= walkers_.size()) {
                throw std::out_of_range("walker index out of range");
            }
            return walkers_[index];
        }

        size_t WalkSimulation::num_walkers() const { return walkers_.size(); }

        datapod::Box WalkSimulation::get_bounds() const {
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
