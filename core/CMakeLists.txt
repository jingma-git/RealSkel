message("-- Configure Milk......")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"                   ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"                 ON)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"                    ON)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ON)

find_package(LIBIGL REQUIRED)
if(LIBIGL_FOUND)
   message("-- Found LIBIGL ${LIBIGL_INCLUDE_DIR}")
   include_directories(${LIBIGL_INCLUDE_DIR})
else()
   message("-- Not Found LIBIGL")
endif()


# OpenCV
if(MSVC)
set(OpenCV_DIR C:/thirdParty/opencv/build/install/x64/vc15/lib)
endif()

if(UNIX)
   # Eigen3
   find_package(Eigen3 3.1.0)
   if(Eigen3_FOUND)
      message("-- Found Eigen3")
      message("-- EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")
   else()
      message("-- Not Found Eigen3")
   endif()
endif()


# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
   message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
   message("-- OpenCV_LIBS: ${OpenCV_LIBS}")
   include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# On Windows: need to download boost and set BOOST_INCLUDEDIR, BOOST_LIBRARYDIR C:\local\boost_1_71_0\lib64-msvc-14.1
find_package(Boost COMPONENTS filesystem REQUIRED)
if(Boost_FOUND)
    message("-- Boost_INCLUDE ${Boost_INCLUDE_DIRS}")
    message("-- Boost_LIB ${Boost_LIBRARIES}")
endif()

# add the library that runs
file(GLOB MILK_SRC "./*.h" "./*.cpp" "./*.c")
# ## include
add_library(milk STATIC ${MILK_SRC})
# target_link_libraries(milk Threads::Threads triangle OpenMP::OpenMP_CXX)
# target_link_libraries(milk Threads::Threads triangle igl::tetgen)
target_link_libraries(milk PUBLIC igl::core igl::cgal igl::triangle igl::tetgen ${OpenCV_LIBS} ${Boost_LIBRARIES})

# state that anybody linking to us needs to include the current source dir
# to find *.h, while we don't.
target_include_directories(milk
                           INTERFACE
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                            $<INSTALL_INTERFACE:include>
                           )
