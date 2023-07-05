include(FetchContent)
FetchContent_Declare(gtest
  QUIET
  URL https://github.com/google/googletest/archive/release-1.10.0.tar.gz
)
# configure build of googletest
option(INSTALL_GTEST "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)" OFF)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(gtest)

include(GoogleTest)

set_target_properties(gtest_main PROPERTIES FOLDER "Vendor")
set_target_properties(gtest PROPERTIES FOLDER "Vendor")

function(buildTest TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PUBLIC Engine PRIVATE gtest_main)
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "Tests")
    gtest_discover_tests(${TEST_NAME})
endfunction()

buildTest(BoxRigidBodyTest BoxRigidBodyTest.cpp)
buildTest(CollisionTest CollisionTest.cpp)
buildTest(MathTest MathTest.cpp)

