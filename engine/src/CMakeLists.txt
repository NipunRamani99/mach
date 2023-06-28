cmake_minimum_required(VERSION 3.14)

project(Engine)
#set(NAMESPACE_NAME "Engine")
option(BUILD_SHARED_LIBS "Build the shared library" ON)

set(ENGINE_VERSION_MAJOR 0)
set(ENGINE_VERSION_MINOR 1)
set(ENGINE_VERSION_PATCH 0)
set(ENGINE_VERSION_STRING ${ENGINE_VERSION_MAJOR}.${ENGINE_VERSION_MINOR}.${ENGINE_VERSION_PATCH})

# adjust this path depending where cmake searches for the target files

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(FetchContent)

#Fetch GLM for vector math
FetchContent_Declare(
	glm
	GIT_REPOSITORY https://github.com/g-truc/glm.git
	GIT_TAG		   0.9.9.8
)
FetchContent_GetProperties(glm)
set(GLM_TEST_ENABLE OFF CACHE BOOL "" FORCE)

FetchContent_Populate(glm)
FetchContent_MakeAvailable(glm)
add_subdirectory(${glm_SOURCE_DIR} ${glm_BINARY_DIR})

file(GLOB SOURCE_FILES
	Engine/*.cpp
    Engine/include/*.hpp)

# Get the publicc header files
file(GLOB PUBLIC_HEADER_FILES
	"Engine/include/*.h"
	"Engine/include/*.hpp"

)
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Engine/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${glm_SOURCE_DIR}>
      
    )

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${ENGINE_VERSION_STRING}
    SOVERSION ${ENGINE_VERSION_MAJOR}
    PUBLIC_HEADER "${PUBLIC_HEADER_FILES}")

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Library
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Library
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  COMPONENT Library # This is for Windows
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} COMPONENT Development
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

# adjust this path depending where cmake searches for the target files
set(ConfigPackageLocation "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

install(EXPORT ${PROJECT_NAME}Targets
  FILE
    ${PROJECT_NAME}Targets.cmake
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Development
)

configure_package_config_file(
    ${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION "${ConfigPackageLocation}"
    PATH_VARS CMAKE_INSTALL_PREFIX
    )

write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${ENGINE_VERSION_STRING}
    COMPATIBILITY AnyNewerVersion
    )

install(TARGETS Engine DESTINATION bin)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Development
)
