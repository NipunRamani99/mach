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


add_executable(
    BoxRigidBodyTest
    BoxRigidBodyTest.cpp)
add_executable(CollisionTest
                CollisionTest.cpp)

target_link_libraries(BoxRigidBodyTest
    PUBLIC
    Engine 
    PRIVATE
    gtest_main
  )
target_link_libraries(CollisionTest
    PUBLIC
    Engine 
    PRIVATE
    gtest_main
  )

include(GoogleTest)
gtest_discover_tests(BoxRigidBodyTest)
gtest_discover_tests(CollisionTest)