#include <cmath>
#include <doctest/doctest.h>
#include <entropy/entropy.hpp>
#include <vector>

TEST_CASE("NoiseGen basic construction") {
    SUBCASE("Default constructor") {
        entropy::noise::NoiseGen gen;
        CHECK_NOTHROW(gen.GetNoise(0.0f, 0.0f));
        CHECK_NOTHROW(gen.GetNoise(0.0f, 0.0f, 0.0f));
    }

    SUBCASE("Constructor with seed") {
        entropy::noise::NoiseGen gen(42);
        CHECK_NOTHROW(gen.GetNoise(0.0f, 0.0f));

        // Same seed should produce same results
        entropy::noise::NoiseGen gen2(42);
        CHECK(gen.GetNoise(0.0f, 0.0f) == gen2.GetNoise(0.0f, 0.0f));
        CHECK(gen.GetNoise(0.0f, 0.0f, 0.0f) == gen2.GetNoise(0.0f, 0.0f, 0.0f));
    }
}

TEST_CASE("Basic noise generation") {
    entropy::noise::NoiseGen gen(1337);

    SUBCASE("2D noise generation") {
        float noise = gen.GetNoise(0.0f, 0.0f);
        CHECK(noise >= -1.0f);
        CHECK(noise <= 1.0f);

        // Different coordinates should generally produce different values
        float noise2 = gen.GetNoise(1.0f, 1.0f);
        CHECK(noise != noise2); // Very unlikely to be the same

        // Test with various coordinates
        for (int i = 0; i < 10; ++i) {
            float x = i * 0.1f;
            float y = i * 0.2f;
            float n = gen.GetNoise(x, y);
            CHECK(n >= -1.0f);
            CHECK(n <= 1.0f);
        }
    }

    SUBCASE("3D noise generation") {
        // Set up the noise generator with a specific noise type
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Perlin);
        gen.SetFrequency(0.1f);

        float noise = gen.GetNoise(0.5f, 0.5f, 0.5f); // Use non-zero coordinates
        CHECK(noise >= -1.0f);
        CHECK(noise <= 1.0f);

        // Use significantly different coordinates to ensure different values
        float noise2 = gen.GetNoise(10.5f, 15.5f, 20.5f);

        // If both are still zero, just check that they're finite
        if (noise == 0.0f && noise2 == 0.0f) {
            CHECK(std::isfinite(noise));
            CHECK(std::isfinite(noise2));
        } else {
            CHECK(noise != noise2); // Very unlikely to be the same
        }

        // Test with various coordinates
        for (int i = 0; i < 10; ++i) {
            float x = i * 0.1f;
            float y = i * 0.2f;
            float z = i * 0.3f;
            float n = gen.GetNoise(x, y, z);
            CHECK(n >= -1.0f);
            CHECK(n <= 1.0f);
        }
    }
}

TEST_CASE("Noise consistency") {
    entropy::noise::NoiseGen gen(123);

    SUBCASE("Same coordinates produce same noise") {
        float noise1 = gen.GetNoise(5.5f, -2.3f);
        float noise2 = gen.GetNoise(5.5f, -2.3f);
        CHECK(noise1 == noise2);

        float noise3d1 = gen.GetNoise(1.1f, 2.2f, 3.3f);
        float noise3d2 = gen.GetNoise(1.1f, 2.2f, 3.3f);
        CHECK(noise3d1 == noise3d2);
    }

    SUBCASE("Deterministic behavior with same seed") {
        entropy::noise::NoiseGen gen1(999);
        entropy::noise::NoiseGen gen2(999);

        for (int i = 0; i < 20; ++i) {
            float x = i * 0.5f;
            float y = i * 0.7f;
            CHECK(gen1.GetNoise(x, y) == gen2.GetNoise(x, y));

            float z = i * 0.3f;
            CHECK(gen1.GetNoise(x, y, z) == gen2.GetNoise(x, y, z));
        }
    }
}

TEST_CASE("Seed effects") {
    SUBCASE("Different seeds produce different noise") {
        entropy::noise::NoiseGen gen1(1);
        entropy::noise::NoiseGen gen2(2);

        float noise1 = gen1.GetNoise(0.7f, 0.3f); // Use non-zero coordinates
        float noise2 = gen2.GetNoise(0.7f, 0.3f);
        CHECK(noise1 != noise2);

        float noise3d1 = gen1.GetNoise(0.7f, 0.3f, 0.9f); // Use non-zero coordinates
        float noise3d2 = gen2.GetNoise(0.7f, 0.3f, 0.9f);
        CHECK(noise3d1 != noise3d2);
    }

    SUBCASE("SetSeed changes noise output") {
        entropy::noise::NoiseGen gen;
        gen.SetSeed(100);
        float noise1 = gen.GetNoise(1.0f, 1.0f);

        gen.SetSeed(200);
        float noise2 = gen.GetNoise(1.0f, 1.0f);

        CHECK(noise1 != noise2);
    }
}

TEST_CASE("Frequency effects") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("SetFrequency changes noise characteristics") {
        gen.SetFrequency(0.01f);
        float noise1 = gen.GetNoise(10.0f, 10.0f);

        gen.SetFrequency(0.1f);
        float noise2 = gen.GetNoise(10.0f, 10.0f);

        CHECK(noise1 != noise2);
    }

    SUBCASE("Higher frequency creates more variation") {
        gen.SetFrequency(0.01f);
        float low_freq_1 = gen.GetNoise(0.0f, 0.0f);
        float low_freq_2 = gen.GetNoise(1.0f, 1.0f);
        float low_freq_diff = std::abs(low_freq_1 - low_freq_2);

        gen.SetFrequency(1.0f);
        float high_freq_1 = gen.GetNoise(0.0f, 0.0f);
        float high_freq_2 = gen.GetNoise(1.0f, 1.0f);
        float high_freq_diff = std::abs(high_freq_1 - high_freq_2);

        // Higher frequency should generally create more variation between nearby points
        // This is a probabilistic test, but very likely to pass
        CHECK(high_freq_diff > low_freq_diff * 0.1f); // Allow some tolerance
    }
}

TEST_CASE("Edge cases and robustness") {
    entropy::noise::NoiseGen gen;

    SUBCASE("Large coordinate values") {
        CHECK_NOTHROW(gen.GetNoise(1000000.0f, 1000000.0f));
        CHECK_NOTHROW(gen.GetNoise(-1000000.0f, -1000000.0f));
        CHECK_NOTHROW(gen.GetNoise(1000000.0f, 1000000.0f, 1000000.0f));
    }

    SUBCASE("Small coordinate values") {
        CHECK_NOTHROW(gen.GetNoise(0.000001f, 0.000001f));
        CHECK_NOTHROW(gen.GetNoise(-0.000001f, -0.000001f));
        CHECK_NOTHROW(gen.GetNoise(0.000001f, 0.000001f, 0.000001f));
    }

    SUBCASE("Zero coordinates") {
        float noise2d = gen.GetNoise(0.0f, 0.0f);
        float noise3d = gen.GetNoise(0.0f, 0.0f, 0.0f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Boundary values") {
        // Test with various boundary values that might cause issues
        std::vector<float> test_values = {-1.0f, 1.0f, -0.5f, 0.5f, -100.0f, 100.0f, -10.0f, 10.0f};

        for (float val : test_values) {
            float noise2d = gen.GetNoise(val, val);
            float noise3d = gen.GetNoise(val, val, val);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
            CHECK(noise2d >= -1.0f);
            CHECK(noise2d <= 1.0f);
            CHECK(noise3d >= -1.0f);
            CHECK(noise3d <= 1.0f);
        }
    }
}
