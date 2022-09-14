cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(USE_FOLDERS ON)
set(CMAKE_BUILD_PARALLEL_LEVEL 12)
enable_language(C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(ATP_TE)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(LINUX True)
	message(STATUS "Compiling For Linux")

	find_package(X11 REQUIRED)
	find_package(TBB REQUIRED)

	list(APPEND EXTRA_LIBRARIES ${X11_LIBRARIES})
	list(APPEND EXTRA_INCLUDES ${X11_INCLUDE_DIRS})

    add_compile_options(-msse2)
else()
	set(LINUX False)
endif()

file(GLOB_RECURSE ATP_TE_SRC "include/*/*.cpp" "include/*.cpp")

add_library(ATP_TE STATIC ${ATP_NN_SRC})

if(MSVC)
	source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include/lungo FILES ${ATP_NN_SRC})
	target_compile_options(ATP_TE PRIVATE "/MP")
endif()

# set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML COMPONENTS network audio graphics window system REQUIRED)

target_include_directories(
	ATP_NN PUBLIC
	"${CMAKE_SOURCE_DIR}/dependencies/stb/include"
	"${CMAKE_SOURCE_DIR}/dependencies/glm"
	# ${SFML_INCLUDE_DIR}
)

target_link_libraries(
	ATP_NN PUBLIC
	sfml-network
	sfml-audio
	sfml-graphics
	sfml-window
	sfml-system
	# glm
	# ${SFML_LIBRARIES}
	# ${SFML_DEPENDENCIES}
)