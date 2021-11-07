
cmake_minimum_required(VERSION 3.1...3.15)
project(shadow_play)

# Set Build Type as Release
set(CMAKE_BUILD_TYPE RELEASE)
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined")
set(CMAKE_CXX_STANDARD 14)

#################### Qt ################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# CMAKE_MODULE_PATH
# CMAKE_SOURCE_DIR

# Change Qt Dir if there is multiple Qt versions in your system
# https://unix.stackexchange.com/questions/414904/anaconda-qt-vs-system-qt
#set(QT_INSTALL_PATH D:/Programs/Qt/Qt5.12.2/5.12.2/msvc2017_64) # !!! set "your own QT install path"
set(QT_INSTALL_PATH /opt/Qt5.12.2/5.10.0/gcc_64) # !!! set "your own QT install path"
file( GLOB_RECURSE sres ${QT_INSTALL_PATH}/*/Qt5Config.cmake )
message("-- Qt resouce ${sres}")
get_filename_component( Qt5_DIR ${sres} DIRECTORY )
message("-- Qt5_DIR resouce ${Qt5_DIR}")
set(QT_QMAKE_EXECUTABLE ${QT_INSTALL_PATH}/bin/qmake)


#################### Find-Package ################################
# OpenGL
cmake_policy(SET CMP0072 OLD) # https://github.com/PointCloudLibrary/pcl/issues/3065
# Find the QtWidgets library
find_package(OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})



# OpenCV
if(MSVC)
set(OpenCV_INCLUDE_DIRS C:/thirdParty/opencv/build/install/include) # !!! set "your own OpenCV path"
endif()
if(UNIX)
    find_package(OpenCV REQUIRED)
    if(OpenCV_FOUND)
      message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
      message("-- OpenCV_LIBS: ${OpenCV_LIBS}")
    endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})


# add_subdirectory(external/triangle)
add_subdirectory(core) # my own library milk
add_subdirectory(viewer)
include_directories(./)


#################### Link & Compile ################################
file(GLOB SRC "viewer/*.h" "./viewer/*.cpp" ./viewer/sketchapp.qrc)
add_executable(main main.cpp ${SRC})

# Use the Widgets module from Qt 5.
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