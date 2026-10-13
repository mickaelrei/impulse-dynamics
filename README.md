# impulse-dynamics

Impulse-based collision dynamics — simulador C++20 do TCC que compara resolvedores de colisão por impulso, com preview 3D via [tmig](https://github.com/mickaelrei/tmig) (git submodule em `external/tmig`).

```bash
git submodule update --init --recursive
cmake -B build
cmake --build build -j
```

## Benchmark de desempenho (headless)

Compara tempo médio de `World::step` por cenário e resolvedor (sem OpenGL):

```bash
./scripts/run-physics-benchmark.sh
```

Gera `benchmark/physics_step_summary.csv`.
