cmake_minimum_required(VERSION 3.9)

include(FetchContent)

project(Sanbox)

set(PROJECT_NAME Sandbox)

#Get all the dependenicies
set(Engine_DIR "./../engine/out/install/x64-Debug/lib/cmake/Engine")
find_package(Engine)

#Fetch GLM for vector math
#FetchContent_Declare(
#	glm
#	GIT_REPOSITORY https://github.com/g-truc/glm.git
#	GIT_TAG		   0.9.9.8
#)
#FetchContent_GetProperties(glm)
#set(GLM_TEST_ENABLE OFF CACHE BOOL "" FORCE)

#FetchContent_MakeAvailable(glm)
#add_subdirectory(${glm_SOURCE_DIR} ${glm_BINARY_DIR})


# Add the source files
file(GLOB SOURCE_FILES
	src/**
)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Engine)
target_include_directories(${PROJECT_NAME} PUBLIC ${glm_SOURCE_DIR})
# Install executable
install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION .)