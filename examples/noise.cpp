#include <entropy/entropy.hpp>
#include <iostream>

int main() {
    entropy::noise::NoiseGen noise(42);
    noise.SetFrequency(0.05f);
    noise.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Perlin);

    std::cout << "Perlin Noise samples:" << std::endl;
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 10; ++x) {
            float val = noise.GetNoise(x * 10.0f, y * 10.0f);
            std::cout << (val > 0 ? "+" : "-");
        }
        std::cout << std::endl;
    }

    return 0;
}
