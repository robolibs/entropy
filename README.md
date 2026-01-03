
<img align="right" width="26%" src="./misc/logo.png">

Entropy
===

A high-performance C++ library for generating various types of noise data, featuring multiple noise algorithms, fractal types, domain warping, and cellular noise generation.

---

## Features

### Noise Types
- **OpenSimplex2**: High-quality gradient noise with improved visual characteristics
- **OpenSimplex2S**: Smoother variant of OpenSimplex2
- **Perlin**: Classic Perlin noise algorithm
- **Value**: Simple value noise interpolation
- **ValueCubic**: Cubic interpolated value noise for smoother results
- **Cellular**: Voronoi/Worley noise for organic textures

### Fractal Types
- **FBm** (Fractional Brownian Motion): Layered noise for natural terrain
- **Ridged**: Creates sharp ridges and valleys
- **PingPong**: Oscillating fractal patterns
- **Domain Warp**: Progressive and Independent fractal domain warping

### Domain Warping
- **OpenSimplex2**: Standard domain warping
- **OpenSimplex2Reduced**: Optimized variant with reduced computation
- **BasicGrid**: Simple grid-based warping

### 3D Rotation Types
- **None**: No rotation applied
- **ImproveXYPlanes**: Optimized for sampling XY planes in 3D
- **ImproveXZPlanes**: Optimized for sampling XZ planes in 3D

### Cellular Noise Features
- **Distance Functions**: Euclidean, EuclideanSq, Manhattan, Hybrid
- **Return Types**: CellValue, Distance, Distance2, Distance2Add, Distance2Sub, Distance2Mul, Distance2Div
- **Jitter Control**: Adjustable randomness in cell positions

---

## Installation

### CMake

```cmake
FetchContent_Declare(
  entropy
  GIT_REPOSITORY https://github.com/bresilla/entropy.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(entropy)

target_link_libraries(<lib/bin> PRIVATE entropy::entropy)
```

---

## Quick Start

```cpp
#include <entropy/entropy.hpp>

// Create noise generator with optional seed
entropy::NoiseGen noise(1337);

// Configure basic settings
noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
noise.SetFrequency(0.01f);

// Generate 2D noise
float value2D = noise.GetNoise(x, y);

// Generate 3D noise  
float value3D = noise.GetNoise(x, y, z);
```

## Basic Configuration

### Noise Type Selection
```cpp
entropy::NoiseGen gen;

// Set different noise algorithms
gen.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
gen.SetNoiseType(entropy::NoiseGen::NoiseType_Perlin);
gen.SetNoiseType(entropy::NoiseGen::NoiseType_Cellular);
gen.SetNoiseType(entropy::NoiseGen::NoiseType_Value);
gen.SetNoiseType(entropy::NoiseGen::NoiseType_ValueCubic);
```

### Basic Parameters
```cpp
// Set random seed for deterministic results
gen.SetSeed(42);

// Control noise frequency (scale)
gen.SetFrequency(0.01f);  // Lower = larger features
```

## Fractal Noise

Add complexity with layered octaves:

```cpp
// Enable fractal noise
gen.SetFractalType(entropy::NoiseGen::FractalType_FBm);

// Configure fractal parameters
gen.SetFractalOctaves(4);        // Number of layers
gen.SetFractalLacunarity(2.0f);  // Frequency multiplier per octave
gen.SetFractalGain(0.5f);        // Amplitude multiplier per octave
gen.SetFractalWeightedStrength(0.0f);  // Octave weighting (0-1)

// For PingPong fractals
gen.SetFractalType(entropy::NoiseGen::FractalType_PingPong);
gen.SetFractalPingPongStrength(2.0f);
```

## Cellular Noise

Generate Voronoi/Worley patterns:

```cpp
gen.SetNoiseType(entropy::NoiseGen::NoiseType_Cellular);

// Configure distance calculation
gen.SetCellularDistanceFunction(entropy::NoiseGen::CellularDistanceFunction_Euclidean);
gen.SetCellularDistanceFunction(entropy::NoiseGen::CellularDistanceFunction_Manhattan);

// Set return value type
gen.SetCellularReturnType(entropy::NoiseGen::CellularReturnType_Distance);
gen.SetCellularReturnType(entropy::NoiseGen::CellularReturnType_CellValue);

// Control cell randomness
gen.SetCellularJitter(1.0f);  // 0-1, higher = more random
```

## Domain Warping

Distort coordinate space for interesting effects:

```cpp
// Set warp algorithm
gen.SetDomainWarpType(entropy::NoiseGen::DomainWarpType_OpenSimplex2);
gen.SetDomainWarpAmp(1.0f);  // Warp strength

// Apply fractal warping
gen.SetFractalType(entropy::NoiseGen::FractalType_DomainWarpProgressive);

// Usage: warp coordinates first, then sample noise
float x = 10.0f, y = 20.0f;
gen.DomainWarp(x, y);  // Modifies x, y
float noise = gen.GetNoise(x, y);
```

## 3D Optimizations

Reduce directional artifacts when sampling 2D slices of 3D noise:

```cpp
// Optimize for XY plane sampling
gen.SetRotationType3D(entropy::NoiseGen::RotationType3D_ImproveXYPlanes);

// Optimize for XZ plane sampling  
gen.SetRotationType3D(entropy::NoiseGen::RotationType3D_ImproveXZPlanes);
```

## Advanced Examples

### Terrain Generation
```cpp
entropy::NoiseGen terrain(1234);
terrain.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
terrain.SetFractalType(entropy::NoiseGen::FractalType_FBm);
terrain.SetFractalOctaves(6);
terrain.SetFrequency(0.005f);
terrain.SetFractalGain(0.6f);

for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
        float elevation = terrain.GetNoise(x, y);
        // elevation is between -1 and 1
    }
}
```

### Cave Systems
```cpp
entropy::NoiseGen caves(5678);
caves.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
caves.SetFractalType(entropy::NoiseGen::FractalType_Ridged);
caves.SetFractalOctaves(4);
caves.SetFrequency(0.02f);

for (int z = 0; z < depth; z++) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float density = caves.GetNoise(x, y, z);
            bool isCave = density > 0.1f; // Threshold for cave vs solid
        }
    }
}
```

### Warped Marble Texture
```cpp
entropy::NoiseGen marble(9999);
marble.SetNoiseType(entropy::NoiseGen::NoiseType_Perlin);
marble.SetDomainWarpType(entropy::NoiseGen::DomainWarpType_OpenSimplex2);
marble.SetDomainWarpAmp(0.5f);
marble.SetFrequency(0.02f);

float x = coord_x, y = coord_y;
marble.DomainWarp(x, y);  // Warp the sampling coordinates
float pattern = marble.GetNoise(x, y);
```

### Cellular Stone Pattern
```cpp
entropy::NoiseGen stones(4321);
stones.SetNoiseType(entropy::NoiseGen::NoiseType_Cellular);
stones.SetCellularDistanceFunction(entropy::NoiseGen::CellularDistanceFunction_Euclidean);
stones.SetCellularReturnType(entropy::NoiseGen::CellularReturnType_Distance);
stones.SetCellularJitter(0.8f);
stones.SetFrequency(0.05f);

float pattern = stones.GetNoise(x, y);
```

## Performance Notes

- **2D vs 3D**: 2D noise is faster than 3D
- **Noise Types**: Value < ValueCubic < Perlin < OpenSimplex2 < Cellular (speed)
- **Fractals**: More octaves = slower but more detailed
- **Domain Warp**: Adds computational cost but creates unique effects

## Output Range

- Basic noise: `-1.0` to `1.0`
- Fractal noise: May slightly exceed `[-1, 1]` depending on settings
- Cellular noise: Varies by return type and distance function

---

## Testing

The library includes comprehensive tests covering all functionality:

```bash
# Build with tests enabled
cmake -B build -DENTROPY_ENABLE_TESTS=ON
make -C build

# Run tests
make -C build test
```

---

## Credits

Based on [FastNoiseLite](https://github.com/Auburn/FastNoiseLite) by [Auburn](https://github.com/Auburn).
