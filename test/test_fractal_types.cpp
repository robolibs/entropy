#include <cmath>
#include <doctest/doctest.h>
#include <entropy/entropy.hpp>
#include <vector>

TEST_CASE("Fractal type configurations") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("FractalType_None (default behavior)") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_None);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("FractalType_FBm") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("FractalType_Ridged") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_Ridged);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("FractalType_PingPong") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_PingPong);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Different fractal types produce different results") {
        float x = 3.5f, y = 4.2f;

        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_None);
        float none_noise = gen.GetNoise(x, y);

        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);
        float fbm_noise = gen.GetNoise(x, y);

        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_Ridged);
        float ridged_noise = gen.GetNoise(x, y);

        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_PingPong);
        float pingpong_noise = gen.GetNoise(x, y);

        // Different fractal types should produce different results
        CHECK(none_noise != fbm_noise);
        CHECK(fbm_noise != ridged_noise);
        CHECK(ridged_noise != pingpong_noise);
    }
}

TEST_CASE("Fractal octaves configuration") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);

    SUBCASE("Valid octave counts") {
        std::vector<int> octave_counts = {1, 2, 3, 4, 5, 8, 10};

        for (int octaves : octave_counts) {
            gen.SetFractalOctaves(octaves);

            float noise2d = gen.GetNoise(2.0f, 3.0f);
            float noise3d = gen.GetNoise(2.0f, 3.0f, 4.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
            CHECK(noise2d >= -2.0f); // Fractal noise might exceed [-1,1] slightly
            CHECK(noise2d <= 2.0f);
            CHECK(noise3d >= -2.0f);
            CHECK(noise3d <= 2.0f);
        }
    }

    SUBCASE("Different octave counts produce different results") {
        float x = 1.0f, y = 1.0f;

        gen.SetFractalOctaves(1);
        float octaves_1 = gen.GetNoise(x, y);

        gen.SetFractalOctaves(3);
        float octaves_3 = gen.GetNoise(x, y);

        gen.SetFractalOctaves(6);
        float octaves_6 = gen.GetNoise(x, y);

        // More octaves generally create more detailed noise
        CHECK(octaves_1 != octaves_3);
        CHECK(octaves_3 != octaves_6);
    }
}

TEST_CASE("Fractal lacunarity configuration") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);

    SUBCASE("Valid lacunarity values") {
        std::vector<float> lacunarity_values = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f};

        for (float lacunarity : lacunarity_values) {
            gen.SetFractalLacunarity(lacunarity);

            float noise2d = gen.GetNoise(1.5f, 2.5f);
            float noise3d = gen.GetNoise(1.5f, 2.5f, 3.5f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Different lacunarity values produce different results") {
        float x = 2.0f, y = 3.0f;

        gen.SetFractalLacunarity(1.5f);
        float lac_15 = gen.GetNoise(x, y);

        gen.SetFractalLacunarity(2.0f);
        float lac_20 = gen.GetNoise(x, y);

        gen.SetFractalLacunarity(3.0f);
        float lac_30 = gen.GetNoise(x, y);

        // Different lacunarity should produce different results
        CHECK(lac_15 != lac_20);
        CHECK(lac_20 != lac_30);
    }
}

TEST_CASE("Fractal gain configuration") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);

    SUBCASE("Valid gain values") {
        std::vector<float> gain_values = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f};

        for (float gain : gain_values) {
            gen.SetFractalGain(gain);

            float noise2d = gen.GetNoise(4.0f, 5.0f);
            float noise3d = gen.GetNoise(4.0f, 5.0f, 6.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Different gain values produce different results") {
        float x = 1.0f, y = 2.0f;

        gen.SetFractalGain(0.2f);
        float gain_02 = gen.GetNoise(x, y);

        gen.SetFractalGain(0.5f);
        float gain_05 = gen.GetNoise(x, y);

        gen.SetFractalGain(0.8f);
        float gain_08 = gen.GetNoise(x, y);

        // Different gain should produce different results
        CHECK(gain_02 != gain_05);
        CHECK(gain_05 != gain_08);
    }
}

TEST_CASE("Fractal weighted strength") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);

    SUBCASE("Valid weighted strength values") {
        std::vector<float> strength_values = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};

        for (float strength : strength_values) {
            gen.SetFractalWeightedStrength(strength);

            float noise2d = gen.GetNoise(3.0f, 4.0f);
            float noise3d = gen.GetNoise(3.0f, 4.0f, 5.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Different weighted strength values") {
        float x = 5.0f, y = 6.0f;

        gen.SetFractalWeightedStrength(0.0f);
        float strength_0 = gen.GetNoise(x, y);

        gen.SetFractalWeightedStrength(0.5f);
        float strength_5 = gen.GetNoise(x, y);

        gen.SetFractalWeightedStrength(1.0f);
        float strength_10 = gen.GetNoise(x, y);

        // All should be valid, may or may not be different
        CHECK(std::isfinite(strength_0));
        CHECK(std::isfinite(strength_5));
        CHECK(std::isfinite(strength_10));
    }
}

TEST_CASE("Fractal ping pong strength") {
    entropy::noise::NoiseGen gen(42);
    gen.SetFractalType(entropy::noise::NoiseGen::FractalType_PingPong);

    SUBCASE("Valid ping pong strength values") {
        std::vector<float> strength_values = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f};

        for (float strength : strength_values) {
            gen.SetFractalPingPongStrength(strength);

            float noise2d = gen.GetNoise(1.0f, 2.0f);
            float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Different ping pong strength values produce different results") {
        float x = 2.0f, y = 3.0f;

        gen.SetFractalPingPongStrength(1.0f);
        float strength_1 = gen.GetNoise(x, y);

        gen.SetFractalPingPongStrength(2.0f);
        float strength_2 = gen.GetNoise(x, y);

        gen.SetFractalPingPongStrength(3.0f);
        float strength_3 = gen.GetNoise(x, y);

        // Different ping pong strength should produce different results
        CHECK(strength_1 != strength_2);
        CHECK(strength_2 != strength_3);
    }
}

TEST_CASE("Complex fractal configurations") {
    entropy::noise::NoiseGen gen(123);

    SUBCASE("FBm with custom parameters") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_FBm);
        gen.SetFractalOctaves(4);
        gen.SetFractalLacunarity(2.1f);
        gen.SetFractalGain(0.6f);
        gen.SetFractalWeightedStrength(0.3f);

        float noise2d = gen.GetNoise(10.0f, 15.0f);
        float noise3d = gen.GetNoise(10.0f, 15.0f, 20.0f);

        CHECK(std::isfinite(noise2d));
        CHECK(std::isfinite(noise3d));
        CHECK(noise2d >= -2.0f);
        CHECK(noise2d <= 2.0f);
        CHECK(noise3d >= -2.0f);
        CHECK(noise3d <= 2.0f);
    }

    SUBCASE("Ridged fractal with custom parameters") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_Ridged);
        gen.SetFractalOctaves(6);
        gen.SetFractalLacunarity(2.5f);
        gen.SetFractalGain(0.4f);

        float noise = gen.GetNoise(5.0f, 7.0f, 9.0f);

        CHECK(std::isfinite(noise));
        CHECK(noise >= -2.0f);
        CHECK(noise <= 2.0f);
    }

    SUBCASE("PingPong fractal with custom parameters") {
        gen.SetFractalType(entropy::noise::NoiseGen::FractalType_PingPong);
        gen.SetFractalOctaves(3);
        gen.SetFractalPingPongStrength(2.5f);
        gen.SetFractalLacunarity(1.8f);
        gen.SetFractalGain(0.7f);

        float noise = gen.GetNoise(8.0f, 12.0f);

        CHECK(std::isfinite(noise));
        CHECK(noise >= -2.0f);
        CHECK(noise <= 2.0f);
    }
}
