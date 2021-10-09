#################### Qt ################################
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

#################### OpenCV ################################
if(MSVC)
set(OpenCV_INCLUDE_DIRS C:/thirdParty/opencv/build/install/include)
endif()
if(UNIX)
    find_package(OpenCV REQUIRED)
    if(OpenCV_FOUND)
      message("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
      message("-- OpenCV_LIBS: ${OpenCV_LIBS}")
    endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})