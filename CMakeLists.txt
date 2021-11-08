
cmake_minimum_required(VERSION 3.1)
project(RealSkel)

#################### Set CMake Configure Variables ################################
set(CMAKE_BUILD_TYPE RELEASE) # DEBUG
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined")
set(CMAKE_CXX_STANDARD 14)

if(MSVC) # Windows
  set(QT_INSTALL_PATH E:/Qt/Qt5.12/5.12.2/msvc2017_64) # !!! set "your own QT install path"
  set(Qt5Core_DIR ${QT_INSTALL_PATH}/lib/cmake/Qt5Core) # !!! set "your own path"
  set(Qt5Widgets_DIR ${QT_INSTALL_PATH}/lib/cmake/Qt5Widgets) # !!! set "your own path"
  set(Qt5Gui_DIR ${QT_INSTALL_PATH}/lib/cmake/Qt5Gui)  # !!! set "your own path"
  set(Qt5OpenGL_DIR ${QT_INSTALL_PATH}/lib/cmake/Qt5OpenGL) # !!! set "your own path"

  set(OpenCV_DIR E:/thirdparty/opencv-4.2.0/build/install) # !!! set "your own path"

  set(Boost_INCLUDE_DIR E:/thirdparty/boost_1_74_0) # !!! set "your own path"
  set(BOOST_LIBRARYDIR E:/thirdparty/boost_1_74_0/lib64-msvc-14.2) # !!! set "your own path"
else() # Unix
  set(QT_INSTALL_PATH /opt/Qt5.12.2/5.10.0/gcc_64) # !!! set "your own QT install path"
  # Change Qt Dir if there is multiple Qt versions in your system
  # https://unix.stackexchange.com/questions/414904/anaconda-qt-vs-system-qt
  file( GLOB_RECURSE sres ${QT_INSTALL_PATH}/*/Qt5Config.cmake )
  message("-- Qt resouce ${sres}")
  get_filename_component( Qt5_DIR ${sres} DIRECTORY )
  message("-- Qt5_DIR resouce ${Qt5_DIR}")
  set(QT_QMAKE_EXECUTABLE ${QT_INSTALL_PATH}/bin/qmake)

  cmake_policy(SET CMP0072 OLD) # OpenGL https://github.com/PointCloudLibrary/pcl/issues/3065
endif()

# Qt
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)



#################### Find-Package ##################################################
# Find the Qt Related libraries
find_package(OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})


add_subdirectory(core) # my own library milk
add_subdirectory(viewer)
include_directories(./)


#################### Link & Compile ###############################################
file(GLOB SRC "viewer/*.h" "./viewer/*.cpp" ./viewer/sketchapp.qrc)
add_executable(main main.cpp ${SRC})
if(UNIX)
  target_link_libraries(
    main
    PUBLIC
      milk
      GL
      Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
      ${OpenCV_LIBS}
  )
endif()

if(MSVC)
  target_link_libraries(
    main
    PUBLIC
      milk
      Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
      ${OpenCV_LIBS}
      OpenGL32.lib
  )
endif()