cmake_minimum_required(VERSION 3.9)

include(FetchContent)

project(Sanbox)

set(PROJECT_NAME Sandbox)

#Get all the dependenicies
set(Engine_DIR "./../engine/out/install/x64-Debug/lib/cmake/Engine")
find_package(Engine)

add_subdirectory(dependencies) # add this

# Add the source files
file(GLOB SOURCE_FILES
	src/**
)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE Engine sfml-graphics ImGui-SFML::ImGui-SFML)
target_include_directories(${PROJECT_NAME} PUBLIC ${glm_SOURCE_DIR})
# Install executable
install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION .)

# Copy DLLs needed for runtime on Windows
if(WIN32)
  if (BUILD_SHARED_LIBS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:ImGui-SFML::ImGui-SFML>
          $<TARGET_FILE:sfml-graphics>
          $<TARGET_FILE:sfml-window>
          $<TARGET_FILE:sfml-system>
          $<TARGET_FILE_DIR:${PROJECT_NAME}>)
  endif()
endif()
