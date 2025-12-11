#include <algorithm>
#include <cmath>
#include <doctest/doctest.h>
#include <entropy/entropy.hpp>
#include <vector>

TEST_CASE("Noise smoothness and continuity") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFrequency(0.1f);

    SUBCASE("2D noise continuity") {
        // Test that nearby points have similar values (smoothness)
        std::vector<float> values;
        float step = 0.01f;

        for (int i = 0; i < 10; ++i) {
            float x = i * step;
            float y = i * step;
            values.push_back(gen.GetNoise(x, y));
        }

        // Check that consecutive values don't differ too dramatically
        for (size_t i = 1; i < values.size(); ++i) {
            float diff = std::abs(values[i] - values[i - 1]);
            CHECK(diff < 0.5f); // Reasonable smoothness threshold
        }
    }

    SUBCASE("3D noise continuity") {
        std::vector<float> values;
        float step = 0.01f;

        for (int i = 0; i < 10; ++i) {
            float x = i * step;
            float y = i * step;
            float z = i * step;
            values.push_back(gen.GetNoise(x, y, z));
        }

        // Check smoothness
        for (size_t i = 1; i < values.size(); ++i) {
            float diff = std::abs(values[i] - values[i - 1]);
            CHECK(diff < 0.5f);
        }
    }
}

TEST_CASE("Noise distribution properties") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFrequency(0.1f);

    SUBCASE("2D noise value distribution") {
        std::vector<float> values;

        // Sample noise at many points
        for (int x = 0; x < 20; ++x) {
            for (int y = 0; y < 20; ++y) {
                values.push_back(gen.GetNoise(x * 0.1f, y * 0.1f));
            }
        }

        // All values should be in range
        for (float val : values) {
            CHECK(val >= -1.0f);
            CHECK(val <= 1.0f);
            CHECK(std::isfinite(val));
        }

        // Should have some variety in values
        float min_val = *std::min_element(values.begin(), values.end());
        float max_val = *std::max_element(values.begin(), values.end());
        CHECK(max_val - min_val > 0.1f); // Should have reasonable range
    }

    SUBCASE("3D noise value distribution") {
        std::vector<float> values;

        // Sample noise at many points
        for (int x = 0; x < 10; ++x) {
            for (int y = 0; y < 10; ++y) {
                for (int z = 0; z < 10; ++z) {
                    values.push_back(gen.GetNoise(x * 0.1f, y * 0.1f, z * 0.1f));
                }
            }
        }

        // All values should be in range
        for (float val : values) {
            CHECK(val >= -1.0f);
            CHECK(val <= 1.0f);
            CHECK(std::isfinite(val));
        }

        // Should have variety
        float min_val = *std::min_element(values.begin(), values.end());
        float max_val = *std::max_element(values.begin(), values.end());
        CHECK(max_val - min_val > 0.1f);
    }
}

TEST_CASE("Performance and stress testing") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("Large number of 2D samples") {
        // Test generating many samples doesn't crash or produce invalid values
        for (int i = 0; i < 1000; ++i) {
            float x = (i % 100) * 0.1f;
            float y = (i / 100) * 0.1f;
            float noise = gen.GetNoise(x, y);

            CHECK(std::isfinite(noise));
            CHECK(noise >= -1.0f);
            CHECK(noise <= 1.0f);
        }
    }

    SUBCASE("Large number of 3D samples") {
        // Test generating many samples doesn't crash or produce invalid values
        for (int i = 0; i < 500; ++i) {
            float x = (i % 10) * 0.2f;
            float y = ((i / 10) % 10) * 0.2f;
            float z = (i / 100) * 0.2f;
            float noise = gen.GetNoise(x, y, z);

            CHECK(std::isfinite(noise));
            CHECK(noise >= -1.0f);
            CHECK(noise <= 1.0f);
        }
    }
}

TEST_CASE("Noise with different data types") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("Double precision coordinates") {
        double x = 1.23456789;
        double y = 9.87654321;
        double z = 5.55555555;

        float noise2d = gen.GetNoise(x, y);
        float noise3d = gen.GetNoise(x, y, z);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Integer coordinates") {
        int x = 5, y = 10, z = 15;

        float noise2d = gen.GetNoise(static_cast<float>(x), static_cast<float>(y));
        float noise3d = gen.GetNoise(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }
}

TEST_CASE("Noise symmetry and patterns") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFrequency(0.1f);

    SUBCASE("Origin-centered sampling") {
        // Test noise around origin to check for obvious asymmetries
        float center = gen.GetNoise(0.0f, 0.0f);
        float right = gen.GetNoise(1.0f, 0.0f);
        float left = gen.GetNoise(-1.0f, 0.0f);
        float up = gen.GetNoise(0.0f, 1.0f);
        float down = gen.GetNoise(0.0f, -1.0f);

        // All should be valid
        CHECK(std::isfinite(center));
        CHECK(std::isfinite(right));
        CHECK(std::isfinite(left));
        CHECK(std::isfinite(up));
        CHECK(std::isfinite(down));

        // They should generally be different (not perfectly symmetric)
        CHECK(right != left);
        CHECK(up != down);
    }

    SUBCASE("Grid pattern detection") {
        // Check that noise doesn't have obvious grid artifacts
        std::vector<float> grid_values;

        for (int x = 0; x < 5; ++x) {
            for (int y = 0; y < 5; ++y) {
                grid_values.push_back(gen.GetNoise(x * 1.0f, y * 1.0f));
            }
        }

        // Should have variety (not all the same value)
        float first = grid_values[0];
        bool has_variety = false;
        for (float val : grid_values) {
            if (std::abs(val - first) > 0.01f) {
                has_variety = true;
                break;
            }
        }
        CHECK(has_variety);
    }
}

TEST_CASE("Noise configuration combinations") {
    entropy::noise::NoiseGen gen(999);

    SUBCASE("Perlin + FBm combination") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Perlin);
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);
        gen.SetFractalOctaves(4);
        gen.SetFrequency(0.05f);

        float noise = gen.GetNoise(10.0f, 20.0f, 30.0f);

        CHECK(std::isfinite(noise));
        CHECK(noise >= -2.0f); // FBm can exceed [-1,1]
        CHECK(noise <= 2.0f);
    }

    SUBCASE("Cellular + custom settings") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Cellular);
        gen.SetCellularDistanceFunction(entropy::noise::NoiseGen::CellularDistanceFunction_Manhattan);
        gen.SetCellularReturnType(entropy::noise::NoiseGen::CellularReturnType_Distance2);
        gen.SetCellularJitter(0.8f);
        gen.SetFrequency(0.02f);

        float noise = gen.GetNoise(5.0f, 15.0f);

        CHECK(std::isfinite(noise));
    }

    SUBCASE("Value + Ridged combination") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Value);
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_Ridged);
        gen.SetFractalOctaves(6);
        gen.SetFractalGain(0.3f);
        gen.SetFractalLacunarity(2.2f);

        float noise = gen.GetNoise(7.5f, 12.3f);

        CHECK(std::isfinite(noise));
        CHECK(noise >= -2.0f);
        CHECK(noise <= 2.0f);
    }
}

TEST_CASE("Edge case coordinate values") {
    entropy::noise::NoiseGen gen(777);

    SUBCASE("Very large positive coordinates") {
        float noise2d = gen.GetNoise(999999.0f, 888888.0f);
        float noise3d = gen.GetNoise(999999.0f, 888888.0f, 777777.0f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
    }

    SUBCASE("Very large negative coordinates") {
        float noise2d = gen.GetNoise(-999999.0f, -888888.0f);
        float noise3d = gen.GetNoise(-999999.0f, -888888.0f, -777777.0f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
    }

    SUBCASE("Mixed large positive and negative coordinates") {
        float noise2d = gen.GetNoise(500000.0f, -500000.0f);
        float noise3d = gen.GetNoise(500000.0f, -500000.0f, 250000.0f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
    }

    SUBCASE("Very small coordinates close to zero") {
        float noise2d = gen.GetNoise(0.0000001f, -0.0000001f);
        float noise3d = gen.GetNoise(0.0000001f, -0.0000001f, 0.0000002f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
    }
}
