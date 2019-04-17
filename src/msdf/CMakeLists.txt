# Define library. Only source files here!
project(msdf VERSION 1.6 LANGUAGES CXX)

find_package(Freetype REQUIRED)

file(GLOB_RECURSE msdf_SOURCES
  "src/*.cpp"
  "../libs/lodepng/*.cpp"
  "../libs/tinyxml2/*.cpp"
)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    add_compile_definitions(MSDFGEN_USE_OPENMP)
endif()
add_library(msdf SHARED ${msdf_SOURCES})

include_directories(${FREETYPE_INCLUDE_DIRS})
target_include_directories(msdf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    src
    ../libs/lodepng
    ../libs/tinyxml2)

target_link_libraries(msdf ${FREETYPE_LIBRARIES})

install(TARGETS msdf EXPORT MsdfConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
