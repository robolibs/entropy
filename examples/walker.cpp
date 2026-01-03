#include <entropy/entropy.hpp>
#include <iostream>

int main() {
    // Single random walk
    entropy::path::WalkConfig config;
    config.seed = 1337;
    config.move_pattern = entropy::path::MovePattern::Moore;

    entropy::path::RandomWalk walker(100, config);
    walker.generate();

    std::cout << "Random Walk:" << std::endl;
    std::cout << "  Type: " << entropy::path::RandomWalk::walker_type_name(walker.get_walker_type()) << std::endl;
    std::cout << "  Speed: " << walker.get_speed() << std::endl;
    std::cout << "  Start: (" << walker.get_start_point().x << ", " << walker.get_start_point().y << ")" << std::endl;
    std::cout << "  End: (" << walker.get_end_point().x << ", " << walker.get_end_point().y << ")" << std::endl;
    std::cout << "  Path length: " << walker.get_path().size() << " poses" << std::endl;

    // Multi-walker simulation
    entropy::path::WalkSimulation sim(50, 3);
    sim.generate();

    std::cout << "\nSimulation with " << sim.num_walkers() << " walkers:" << std::endl;
    for (size_t i = 0; i < sim.num_walkers(); ++i) {
        const auto &w = sim.get_walker(i);
        std::cout << "  Walker " << i << ": " << entropy::path::RandomWalk::walker_type_name(w.get_walker_type())
                  << " (speed=" << w.get_speed() << ")" << std::endl;
    }

    return 0;
}
