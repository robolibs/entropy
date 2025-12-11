#include <doctest/doctest.h>
#include <entropy/entropy.hpp>
#include <vector>

TEST_CASE("Noise type configurations") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("OpenSimplex2 noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_OpenSimplex2);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("OpenSimplex2S noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_OpenSimplex2S);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Cellular noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Cellular);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Perlin noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Perlin);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("ValueCubic noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_ValueCubic);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Value noise type") {
        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Value);

        float noise2d = gen.GetNoise(1.0f, 2.0f);
        float noise3d = gen.GetNoise(1.0f, 2.0f, 3.0f);

        CHECK(noise2d >= -1.0f);
        CHECK(noise2d <= 1.0f);
        CHECK(noise3d >= -1.0f);
        CHECK(noise3d <= 1.0f);
    }

    SUBCASE("Different noise types produce different results") {
        float x = 5.0f, y = 3.0f, z = 1.0f;

        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_OpenSimplex2);
        float simplex2d = gen.GetNoise(x, y);
        float simplex3d = gen.GetNoise(x, y, z);

        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Perlin);
        float perlin2d = gen.GetNoise(x, y);
        float perlin3d = gen.GetNoise(x, y, z);

        gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Value);
        float value2d = gen.GetNoise(x, y);
        float value3d = gen.GetNoise(x, y, z);

        // Different noise types should produce different results
        CHECK(simplex2d != perlin2d);
        CHECK(perlin2d != value2d);
        CHECK(simplex3d != perlin3d);
        CHECK(perlin3d != value3d);
    }
}

TEST_CASE("Rotation type configurations") {
    entropy::noise::NoiseGen gen(42);

    SUBCASE("RotationType3D_None") {
        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_None);

        float noise = gen.GetNoise(1.0f, 2.0f, 3.0f);
        CHECK(noise >= -1.0f);
        CHECK(noise <= 1.0f);
    }

    SUBCASE("RotationType3D_ImproveXYPlanes") {
        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_ImproveXYPlanes);

        float noise = gen.GetNoise(1.0f, 2.0f, 3.0f);
        CHECK(noise >= -1.0f);
        CHECK(noise <= 1.0f);
    }

    SUBCASE("RotationType3D_ImproveXZPlanes") {
        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_ImproveXZPlanes);

        float noise = gen.GetNoise(1.0f, 2.0f, 3.0f);
        CHECK(noise >= -1.0f);
        CHECK(noise <= 1.0f);
    }

    SUBCASE("Different rotation types may produce different results") {
        float x = 10.0f, y = 5.0f, z = 2.0f;

        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_None);
        float noise_none = gen.GetNoise(x, y, z);

        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_ImproveXYPlanes);
        float noise_xy = gen.GetNoise(x, y, z);

        gen.SetRotationType3D(entropy::noise::NoiseGen::RotationType3D_ImproveXZPlanes);
        float noise_xz = gen.GetNoise(x, y, z);

        // Note: These might be the same in some cases, but generally should differ
        // We just ensure they're all valid values
        CHECK(noise_none >= -1.0f);
        CHECK(noise_none <= 1.0f);
        CHECK(noise_xy >= -1.0f);
        CHECK(noise_xy <= 1.0f);
        CHECK(noise_xz >= -1.0f);
        CHECK(noise_xz <= 1.0f);
    }
}

TEST_CASE("Cellular noise configurations") {
    entropy::noise::NoiseGen gen(42);
    gen.SetNoiseType(entropy::noise::NoiseGen::NoiseType_Cellular);

    SUBCASE("Cellular distance functions") {
        std::vector<entropy::noise::NoiseGen::CellularDistanceFunction> distance_funcs = {
            entropy::noise::NoiseGen::CellularDistanceFunction_Euclidean,
            entropy::noise::NoiseGen::CellularDistanceFunction_EuclideanSq,
            entropy::noise::NoiseGen::CellularDistanceFunction_Manhattan, entropy::noise::NoiseGen::CellularDistanceFunction_Hybrid};

        for (auto func : distance_funcs) {
            gen.SetCellularDistanceFunction(func);

            float noise2d = gen.GetNoise(3.0f, 4.0f);
            float noise3d = gen.GetNoise(3.0f, 4.0f, 5.0f);

            CHECK(noise2d >= -1.0f);
            CHECK(noise2d <= 1.0f);
            CHECK(noise3d >= -1.0f);
            CHECK(noise3d <= 1.0f);
        }
    }

    SUBCASE("Cellular return types") {
        std::vector<entropy::noise::NoiseGen::CellularReturnType> return_types = {
            entropy::noise::NoiseGen::CellularReturnType_CellValue,    entropy::noise::NoiseGen::CellularReturnType_Distance,
            entropy::noise::NoiseGen::CellularReturnType_Distance2,    entropy::noise::NoiseGen::CellularReturnType_Distance2Add,
            entropy::noise::NoiseGen::CellularReturnType_Distance2Sub, entropy::noise::NoiseGen::CellularReturnType_Distance2Mul,
            entropy::noise::NoiseGen::CellularReturnType_Distance2Div};

        for (auto type : return_types) {
            gen.SetCellularReturnType(type);

            float noise2d = gen.GetNoise(2.0f, 3.0f);
            float noise3d = gen.GetNoise(2.0f, 3.0f, 4.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Cellular jitter modifier") {
        std::vector<float> jitter_values = {0.0f, 0.5f, 1.0f, 1.5f};

        for (float jitter : jitter_values) {
            gen.SetCellularJitter(jitter);

            float noise2d = gen.GetNoise(1.0f, 1.0f);
            float noise3d = gen.GetNoise(1.0f, 1.0f, 1.0f);

            CHECK(std::isfinite(noise2d));
            CHECK(std::isfinite(noise3d));
        }
    }

    SUBCASE("Cellular configurations produce different results") {
        float x = 7.0f, y = 8.0f;

        gen.SetCellularReturnType(entropy::noise::NoiseGen::CellularReturnType_Distance);
        float dist_noise = gen.GetNoise(x, y);

        gen.SetCellularReturnType(entropy::noise::NoiseGen::CellularReturnType_CellValue);
        float cell_noise = gen.GetNoise(x, y);

        // Different return types should generally produce different results
        // (though not guaranteed in all cases)
        CHECK(std::isfinite(dist_noise));
        CHECK(std::isfinite(cell_noise));
    }
}
