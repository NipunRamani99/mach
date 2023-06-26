cmake_minimum_required(VERSION 3.9)

project(Sanbox)

set(Engine_DIR "./../engine/out/install/x64-Debug/lib/cmake/Engine")
find_package(Engine)
# Add the source files
file(GLOB SOURCE_FILES
	src/**
)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Engine )
# Install executable
install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION .)