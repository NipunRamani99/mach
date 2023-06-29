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

function(buildTest TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PUBLIC Engine PRIVATE gtest_main)
    gtest_discover_tests(${TEST_NAME})
endfunction()

buildTest(BoxRigidBodyTest BoxRigidBodyTest.cpp)
buildTest(CollisionTest CollisionTest.cpp)
buildTest(MathTest MathTest.cpp)

#add_executable(BoxRigidBodyTest BoxRigidBodyTest.cpp)
#add_executable(CollisionTest CollisionTest.cpp)
#add_executable(MathTest MathTest.cpp)
#[[
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

target_link_libraries(MathTest
    PUBLIC
    Engine 
    PRIVATE
    gtest_main
  )

gtest_discover_tests(BoxRigidBodyTest)
gtest_discover_tests(CollisionTest)
]]