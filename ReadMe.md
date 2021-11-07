# Real-time Skeletonization for Sketch-based Modeling (SMI:2021)
<img src="data/banner_thumb.png" alt="ModelingCat" width="250"/>

## Demo
We provide an executable software under directory "demo_exe/".

<img src="data/ModelingCat.gif" alt="ModelingCat" width="500"/>

## Tested Environment
1. Ubuntu20.04 LTS, gcc-7 compiler
2. Windows10, Visual Studio Community 2017/2019 x86_amd64 compiler

## Prerequisites 
### Ubuntu20.04 LTS
1. `cmake`: https://cmake.org/
2. `VS Code` (Optional but Highly Recommended): https://code.visualstudio.com/docs/cpp/cmake-linux

### Windows10
1. `Visual Studio 2017/2019`
2. `git`: https://git-scm.com/
3. `cmake`: https://cmake.org/
4. `VS Code` (Optional but Highly Recommended): https://code.visualstudio.com/docs/cpp/cmake-linux

## Dependencies
Qt, OpenCV, Boost, Libigl(Tetgen, Triangle, CGAL, Eigen)
Please install CMake, Qt and OpenCV first, and modify the CMakeLists.txt according to your library's install path.
### Ubuntu20.04 LTS
#### (1) Install Qt 5.12.2 
1. Download `qt-opensource-linux-x64-5.12.2.run` from https://download.qt.io/archive/qt/5.12/5.12.2/, click `.run` to install
#### (2) Install OpenCV
```
sudo apt-get install libopencv-dev
```
#### (3) Install Boost
```
sudo apt-get install libboost-all-dev
```

### Windows10
#### (1) Install Qt 5.12.2 
1. Download `qt-opensource-windows-x86-5.12.2.exe` from https://download.qt.io/archive/qt/5.12/5.12.2/, click `.exe` to install, then select `MSVC 2017 64-bit`
#### (2) Install OpenCV
1. Download OpenCV source code from https://github.com/opencv/opencv/archive/4.2.0.zip
2. CMake Configure:

<img src="data/configure_opencv.png" alt="configure_opencv.png" width="800"/>

3. Build and Install: 1. open `OpenCV.sln` under `build` directory with VS2017/VS2019. 2. Select `build type` as `Release` and `Build solution`. 3.  goto `CMakeTargets/INSTALL` and `Build`

<img src="data/build_install_opencv.png" alt="build_install_opencv.png" width="800"/>


#### (3) Install Boost
https://sourceforge.net/projects/boost/files/boost-binaries/1.74.0/, select suitable versions to download (`boost_1_74_0-msvc-14.2-64.exe` for VS2019, `boost_1_74_0-msvc-14.1-64.exe` for VS2017), click `.exe` to install

## Compile & Run

### For Linux Users, type in the following commands:
```
mkdir build
cd build
cmake ..
make -j8
./main
```
### For Windows Users
It's highly recommended using 'visual studio code' IDE and "cmake tools from 'VS code' market place" to compile the source code.

<img src="data/compile.png" alt="compile in VS Code" width="800"/>

1. Select the `build type` as `Realse`
2. Select a compiler:  `Ctrl+Shift+P`, type in `CMake: Select a Kit`, select a compiler (eg. `Visual Studio Community 2017 x86_amd64` or ``Visual Studio Community 2019 x86_amd64`)
3. CMake Configuration: `Ctrl+Shift+P`, type in `CMake Configure`. The configuration process will automatically download all dependencies of libigl. Wait until `CMake Configuration` successfully finishes.

<img src="data/configure_done.png" alt="configure_done" width="800"/>

4. Build the solution:  you will find `RealSkel.sln` under directory `build/`，
open `.sln` with Visual Studio 2017/2019 and `Build Solution`.  

<img src="data/build_realskel.png" alt="build" width="800"/>

5. Run the program: copy all `*.dll` and directory `platforms/` from `demo_exe` to your `build/Release/` directory. Click `main.exe`, then you can run the program.

<img src="data/copy_dll_before.png" alt="copy_dll_before" width="800"/>
<img src="data/copy_dll_after.png" alt="copy_dll_after" width="800"/>

## Other Issues
### Adapt to CGAL
goto `external\libigl\external\cgal\Surface_mesh\include\CGAL\Surface_mesh\Surface_mesh.h`, add the following code in `Class SM_Index`
```cpp
size_type idx()const{return idx_;}
```
You should get something like this
```cpp
class SM_Index
{
// ... 
    size_type idx()const{return idx_;}
// ...
};
```