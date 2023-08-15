include(FetchContent)
FetchContent_Declare(gtest
  QUIET
  URL https://github.com/google/googletest/archive/release-1.10.0.tar.gz
)

FetchContent_Declare(googlebenchmark
  QUIET
   GIT_REPOSITORY https://github.com/google/benchmark.git
   GIT_TAG v1.8.1) # need master for benchmark::benchmark


# configure build of googletest
option(INSTALL_GTEST "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)" OFF)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_TESTING off)    # to suppress benchmark internal tests

FetchContent_MakeAvailable(gtest)
FetchContent_MakeAvailable(googlebenchmark)

include(GoogleTest)

set_target_properties(gtest_main PROPERTIES FOLDER "Vendor")
set_target_properties(gtest PROPERTIES FOLDER "Vendor")
set_target_properties(benchmark PROPERTIES FOLDER "Vendor")
function(buildTest TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PUBLIC Engine PRIVATE gtest_main)
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "Tests")
    gtest_discover_tests(${TEST_NAME})
endfunction()
get_property(BENCHMARK_DLL TARGET benchmark::benchmark PROPERTY IMPORTED_LOCATION_RELEASE)
function(buildBenchmark BENCH_NAME BENCH_SOURCE)
	add_executable(${BENCH_NAME} ${BENCH_SOURCE})
	target_link_libraries(${BENCH_NAME} PUBLIC Engine PRIVATE benchmark::benchmark)
	set_target_properties(${BENCH_NAME} PROPERTIES FOLDER "Tests")
endfunction()

buildTest(BoxRigidBodyTest unit_test/BoxRigidBodyTest.cpp)
buildTest(CollisionTest unit_test/CollisionTest.cpp)
buildTest(MathTest unit_test/MathTest.cpp)
buildTest(JointTest unit_test/JointTest.cpp)
buildTest(CircleRigidBodyTest unit_test/CircleRigidBodyTest.cpp)
buildTest(AABBTest unit_test/AABBTest.cpp)
buildTest(MachTest unit_test/MachTest.cpp)
buildBenchmark(CollisionBenchmark perf_test/CollisionBenchmark.cpp)