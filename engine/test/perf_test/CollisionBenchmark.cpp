#include <benchmark/benchmark.h>

static void bm_test(benchmark::State& state) {
	for (auto _ : state) {
			
	}
}

BENCHMARK(bm_test);
BENCHMARK_MAIN();