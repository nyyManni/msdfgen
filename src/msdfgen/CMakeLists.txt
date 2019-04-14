find_package(Freetype REQUIRED)

include_directories(
  ../msdf/src/
  ../include/
  ../libs/lodepng/
  ../libs/tinyxml2/
)
include_directories(${FREETYPE_INCLUDE_DIRS})
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../msdf)

# Define an executable
add_executable(msdfgen src/main.cpp)

# Define the libraries this project depends upon
target_link_libraries(msdfgen msdf)
