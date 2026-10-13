#include "bench/bench_layouts.hpp"
#include "sim/scenarios.hpp"
#include "sim/world.hpp"
#include "solvers/global_solver.hpp"
#include "solvers/iterative_solver.hpp"
#include "solvers/sequential_solver.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

constexpr float kDt = 1.f / 60.f;

enum class BenchScenario : int {
    HeadOn = 0,
    NewtonsCradle,
    SimultaneousHit,
    Count
};

struct BenchScenarioSpec {
    const char* key;
    const char* label;
    Scenario scenario;
};

const BenchScenarioSpec& spec(BenchScenario s) {
    static const BenchScenarioSpec table[] = {
        {"head_on", "Head-on collision", Scenario::HeadOnCollision},
        {"newton_cradle", "Newton's cradle", Scenario::NewtonsCradle},
        {"simultaneous_hit", "Simultaneous hit", Scenario::SimultaneousHit},
    };
    return table[static_cast<int>(s)];
}

enum class SolverKind { Sequential, Global, Iterative };

struct SolverSpec {
    const char* key;
    const char* label;
    SolverKind kind;
    size_t iterativeM;
};

void initWorld(
    World& world,
    const BenchScenarioSpec& scenarioSpec,
    size_t replicas,
    size_t newtonChainLength
) {
    const ScenarioDef& def = getScenarioDef(scenarioSpec.scenario);
    bench::buildStressLayout(world.bodies(), scenarioSpec.scenario, replicas, newtonChainLength);
    def.configureWorld(world);
    world.setPenetrationCorrectionEnabled(true);
}

double measureAverageStepMsSteady(
    World& world,
    CollisionSolver& solver,
    int warmupSteps,
    int measureSteps
) {
    for (int i = 0; i < warmupSteps; ++i) {
        world.step(kDt, solver);
    }

    const auto t0 = Clock::now();
    for (int i = 0; i < measureSteps; ++i) {
        world.step(kDt, solver);
    }
    const auto t1 = Clock::now();

    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return totalMs / static_cast<double>(measureSteps);
}

double measureAverageStepMsReset(
    World& world,
    CollisionSolver& solver,
    const BenchScenarioSpec& scenarioSpec,
    size_t replicas,
    size_t newtonChainLength,
    int measureSteps
) {
    double totalMs = 0.0;
    for (int i = 0; i < measureSteps; ++i) {
        initWorld(world, scenarioSpec, replicas, newtonChainLength);
        const auto t0 = Clock::now();
        world.step(kDt, solver);
        const auto t1 = Clock::now();
        totalMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    return totalMs / static_cast<double>(measureSteps);
}

struct Aggregate {
    double mean = 0.0;
    double stddev = 0.0;
};

Aggregate aggregate(const std::vector<double>& samples) {
    Aggregate out{};
    if (samples.empty()) {
        return out;
    }
    double sum = 0.0;
    for (double v : samples) {
        sum += v;
    }
    out.mean = sum / static_cast<double>(samples.size());
    if (samples.size() < 2) {
        return out;
    }
    double var = 0.0;
    for (double v : samples) {
        const double d = v - out.mean;
        var += d * d;
    }
    var /= static_cast<double>(samples.size() - 1);
    out.stddev = std::sqrt(var);
    return out;
}

std::filesystem::path defaultOutputPath() {
    return std::filesystem::path("benchmark") / "physics_step_results.csv";
}

std::filesystem::path defaultSummaryPath() {
    return std::filesystem::path("benchmark") / "physics_step_summary.csv";
}

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "  --output PATH       Detailed CSV (default: benchmark/physics_step_results.csv)\n"
        << "  --summary PATH      Aggregated CSV (default: benchmark/physics_step_summary.csv)\n"
        << "  --repetitions N     Runs per (scenario, solver) (default: 5)\n"
        << "  --replicas N        Parallel copies of each scenario (default: 48)\n"
        << "  --newton-balls N    Balls per Newton chain when replicas > 1 (default: 5)\n"
        << "  --warmup STEPS      Warmup steps before timing (default: 120)\n"
        << "  --steps STEPS       Timed steps per run (default: 3000)\n"
        << "  --iterative-m M     Gauss-Seidel iterations (default: 10)\n"
        << "  --steady-state      Continuous simulation (default: reset scene each timed step)\n";
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path outputPath = defaultOutputPath();
    std::filesystem::path summaryPath = defaultSummaryPath();
    int repetitions = 5;
    int warmupSteps = 120;
    int measureSteps = 3000;
    size_t iterativeM = 10;
    size_t replicas = 48;
    size_t newtonChainLength = 5;
    bool steadyState = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto needValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--output") {
            outputPath = needValue("--output");
        } else if (arg == "--summary") {
            summaryPath = needValue("--summary");
        } else if (arg == "--repetitions") {
            repetitions = std::stoi(needValue("--repetitions"));
        } else if (arg == "--replicas") {
            replicas = static_cast<size_t>(std::stoul(needValue("--replicas")));
        } else if (arg == "--newton-balls") {
            newtonChainLength = static_cast<size_t>(std::stoul(needValue("--newton-balls")));
        } else if (arg == "--warmup") {
            warmupSteps = std::stoi(needValue("--warmup"));
        } else if (arg == "--steps") {
            measureSteps = std::stoi(needValue("--steps"));
        } else if (arg == "--iterative-m") {
            iterativeM = static_cast<size_t>(std::stoul(needValue("--iterative-m")));
        } else if (arg == "--steady-state") {
            steadyState = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return 2;
        }
    }

    if (repetitions < 1 || warmupSteps < 0 || measureSteps < 1 || replicas < 1) {
        std::cerr << "Invalid repetition, replica or step counts.\n";
        return 2;
    }

    std::filesystem::create_directories(outputPath.parent_path());
    std::filesystem::create_directories(summaryPath.parent_path());

    std::ofstream detailOut(outputPath);
    if (!detailOut) {
        std::cerr << "Failed to open " << outputPath << "\n";
        return 1;
    }

    detailOut << "scenario_key,scenario_label,solver_key,solver_label,iterative_m,replicas,"
                 "body_count,reset_each_step,repetition,warmup_steps,measure_steps,avg_step_ms\n";

    std::ofstream summaryOut(summaryPath);
    if (!summaryOut) {
        std::cerr << "Failed to open " << summaryPath << "\n";
        return 1;
    }

    summaryOut << "scenario_key,solver_key,iterative_m,replicas,body_count,reset_each_step,"
                  "repetitions,mean_ms,stddev_ms,warmup_steps,measure_steps,dt\n";

    SequentialSolver sequential;
    GlobalImpulseSolver global;
    IterativeSolver iterative(iterativeM);

    const std::vector<SolverSpec> solvers = {
        {"sequential", "Sequential", SolverKind::Sequential, 0},
        {"global", "Global Gauss-Jordan", SolverKind::Global, 0},
        {"iterative", "Iterative Gauss-Seidel", SolverKind::Iterative, iterativeM},
    };

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "physics-scenarios-bench  dt=" << kDt << "  reps=" << repetitions
              << "  replicas=" << replicas << "  warmup=" << warmupSteps
              << "  steps=" << measureSteps
              << "  mode=" << (steadyState ? "steady" : "reset-each-step") << "\n\n";

    for (int si = 0; si < static_cast<int>(BenchScenario::Count); ++si) {
        const BenchScenarioSpec& sc = spec(static_cast<BenchScenario>(si));
        const size_t bodyCount = bench::bodyCountFor(sc.scenario, replicas, newtonChainLength);
        const bool scenarioSteady =
            steadyState || sc.scenario == Scenario::NewtonsCradle;
        std::cout << sc.label << "  (N=" << bodyCount << ", replicas=" << replicas
                  << ", mode=" << (scenarioSteady ? "steady" : "reset") << "):\n";

        for (const SolverSpec& sol : solvers) {
            std::vector<double> samples;
            samples.reserve(static_cast<size_t>(repetitions));

            for (int rep = 0; rep < repetitions; ++rep) {
                World world;
                initWorld(world, sc, replicas, newtonChainLength);

                CollisionSolver* solver = nullptr;
                switch (sol.kind) {
                case SolverKind::Sequential:
                    solver = &sequential;
                    break;
                case SolverKind::Global:
                    solver = &global;
                    break;
                case SolverKind::Iterative:
                    iterative.setMaxIterations(iterativeM);
                    solver = &iterative;
                    break;
                }

                double avgMs = 0.0;
                initWorld(world, sc, replicas, newtonChainLength);
                if (scenarioSteady) {
                    avgMs = measureAverageStepMsSteady(world, *solver, warmupSteps, measureSteps);
                } else {
                    avgMs = measureAverageStepMsReset(
                        world,
                        *solver,
                        sc,
                        replicas,
                        newtonChainLength,
                        measureSteps
                    );
                }
                samples.push_back(avgMs);

                detailOut << sc.key << ',' << '"' << sc.label << '"' << ','
                          << sol.key << ',' << '"' << sol.label << '"' << ','
                          << sol.iterativeM << ',' << replicas << ',' << bodyCount << ','
                          << (scenarioSteady ? 0 : 1) << ','
                          << rep << ',' << warmupSteps << ',' << measureSteps << ','
                          << avgMs << '\n';
            }

            const Aggregate stats = aggregate(samples);
            summaryOut << sc.key << ',' << sol.key << ',' << sol.iterativeM << ','
                       << replicas << ',' << bodyCount << ','
                       << (scenarioSteady ? 0 : 1) << ','
                       << repetitions << ',' << stats.mean << ',' << stats.stddev << ','
                       << warmupSteps << ',' << measureSteps << ',' << kDt << '\n';

            std::cout << "  " << std::setw(22) << std::left << sol.label
                      << " mean=" << std::setprecision(4) << stats.mean
                      << " ms  stddev=" << stats.stddev << " ms\n";
        }
        std::cout << '\n';
    }

    std::cout << "Wrote " << outputPath << "\n";
    std::cout << "Wrote " << summaryPath << "\n";
    return 0;
}
