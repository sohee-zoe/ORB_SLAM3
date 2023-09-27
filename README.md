# ORB_SLAM3
Ubuntu 20.04 / C++14 / ROS Noetic


## ROS Noetic

[noetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

```bash
echo "export ROS_IP=localhost" >> ~/.zshrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.zshrc

source ~/.zshrc
```

- ROS 패키지 ddynamic_reconfigure 설치
    
    ```bash
    sudo apt-get install ros-$ROS_DISTRO-ddynamic-reconfigure
    
    # sudo apt install ros-$ROS_DISTR-rosbash
    ```
    

## Realsense SDK (librealsense2)

- https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

```bash
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update

sudo apt-get install librealsense2-dkms 
sudo apt-get install librealsense2-utils 
sudo apt-get install librealsense2-dev 
sudo apt-get install librealsense2-dbg

# sudo apt install ros-$ROS_DISTRO-librealsense2*

## 설치 확인
realsense-viewer

## 버전 확인
# modinfo uvcvideo | grep "version:"
dpkg -l | grep realsense
```

```bash
## 삭제
dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
```

## RealSense ROS Wrapper

- https://github.com/leggedrobotics/realsense-ros-rsl

```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```

## ORB-SLAM3

### dependencies

```bash
sudo apt install libpython2.7-dev
sudo apt-get install libboost-dev
sudo apt install libssl-dev
sudo apt install libboost-serialization-dev 
# 또는
# sudo apt install libboost-all-dev
```

- OpenCV v4.4 이상
    
    ```bash
    cd ~/opt/opencv
    git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib
    git checkout 4.5.2
    cd ../opencv
    git checkout 4.5.2
    mkdir build && cd build
    
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D CUDA_ARCH_BIN=7.5 \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF .. \
    -D PYTHON3_INCLUDE_DIR=/usr/include/python3.8 \
    -D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include/ \
    -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
    -D PYTHON3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so ../
    
    make -j8 
    # time make -j$(nproc)
    sudo make install
    
    ## 확인
    pkg-config --modversion opencv
    ```
    
- Eigen3 v3.1.0 이상
    
    ```bash
    sudo apt-get install libeigen3-dev
    
    pkg-config --modversion eigen3
    ```
    

### Thirdparty

- Pangolin
    
    ```bash
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
    cd ORB_SLAM3
    
    cd Thirdparty/
    
    ### Pangolin 빌드
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    ./scripts/install_prerequisites.sh recommended
    
    # sudo apt install ninja-build
    cmake -B build -GNinja 
    cmake --build build 
    ```
    

### Source Code 수정

- (Optional) `Examples/Monocular/mono_euroc.cc` 뷰어 true 수정
    
    ```bash
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    ```
    
- `src/Optimizer.cc` 수정
    - 해당 관련 이슈
    - https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/289
        
        ```cpp
        Not preintegrated measurement
        Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)
        ```
        
    - 수정
    
    ```cpp
    if(pKFi->mPrevKF && pKFi->mnId<=maxKFid && pKFi->mpImuPreintegrated) // 수정
    {
    	if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
    		continue;
    ```
    
    ```cpp
    if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
    {
    	if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
    		continue;
    	if(!pKFi->mpImuPreintegrated)
    	{
    		std::cout << "Not preintegrated measurement" << std::endl;
    		continue; // 수정
    	}
    ```


### CMakeLists.txt 수정

- `ORB_SLAM3/CMakeLists.txt`
    
    ```bash
    # # Check C++11 or C++0x support
    # include(CheckCXXCompilerFlag)
    # CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    # CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    # if(COMPILER_SUPPORTS_CXX11)
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    #    add_definitions(-DCOMPILEDWITHC11)
    #    message(STATUS "Using flag -std=c++11.")
    # elseif(COMPILER_SUPPORTS_CXX0X)
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    #    add_definitions(-DCOMPILEDWITHC0X)
    #    message(STATUS "Using flag -std=c++0x.")
    # else()
    #    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    # endif()
    
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_EXTENSIONS OFF)
    add_definitions(-DCOMPILEDWITHC11)
    ```
    
- (Optional) `ORB_SLAM3/Thirdparty/Sophus/CMakeLists.txt`
    
    ```bash
    # # Set compiler-specific settings (FixMe: Should not cmake do this for us automatically?)
    # IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    #    SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
    #    SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
    #    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra  -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
    # ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    #    SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
    #    SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
    #    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++11 -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
    #    SET(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
    #    SET(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
    #    SET(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
    # ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
    #    ADD_DEFINITIONS("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
    # ENDIF()
    
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_EXTENSIONS OFF)
    add_definitions(-DCOMPILEDWITHC11)
    ```
    

### Build

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

### 실행

```bash
cd ORB_SLAM3

# [실행하고자 하는 Mode] [장소 인식에 필요한 Vocabulary.txt] [Dataset에 맞는 파라미터를 넣어주기 위한 yaml파일] [데이터셋 경로] [데이터셋에 맞는 Timestamp.txt]

## Mono
./Examples/Monocular/mono_euroc \
./Vocabulary/ORBvoc.txt \
./Examples/Monocular/EuRoC.yaml \
.**/DataSets/EuRoC/MH01/** \
./Examples/Monocular/EuRoC_TimeStamps/MH01.txt

## Mono-Inertial
./Examples/Monocular-Inertial/mono_inertial_euroc \
./Vocabulary/ORBvoc.txt \
./Examples/Monocular-Inertial/EuRoC.yaml \
./DataSets/EuRoC/MH01/ \
./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt
```

## ORB-SLAM3-ROS

- Examples_Old에 존재하는 ROS 폴더를 Examples로 복사
    
    `cp -r Examples_old/ROS Examples`
    
- zshrc 설정
    
    ```bash
    sudo nano ~/.zshrc
    
    # 추가
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$HOME/project/ORB_SLAM3/Examples/ROS/ORB_SLAM3
    
    source ~/.zshrc
    ```
    
- CMakeLists.txt 수정
    
    ```bash
    #cmake_minimum_required(VERSION 2.4.6)
    project(ORB_SLAM3)
    #include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
    
    # CXX11 -> CXX14 주석 처리
    # # Check C++11 or C++0x support
    # include(CheckCXXCompilerFlag)
    # CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    # CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    # if(COMPILER_SUPPORTS_CXX11)
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    #    add_definitions(-DCOMPILEDWITHC11)
    #    message(STATUS "Using flag -std=c++11.")
    # elseif(COMPILER_SUPPORTS_CXX0X)
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    #    add_definitions(-DCOMPILEDWITHC0X)
    #    message(STATUS "Using flag -std=c++0x.")
    # else()
    #    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    # endif()
    
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_EXTENSIONS OFF)
    add_definitions(-DCOMPILEDWITHC11)
    
    ## OpenCV 버전 수정
    find_package(OpenCV 4.0 QUIET)
    
    include_directories(
    # ...
    ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
    # ...
    )
    
    ## MonoAR 주석처리
    ## Node for monocular camera (Augmented Reality Demo)
    # rosbuild_add_executable(MonoAR
    # src/AR/ros_mono_ar.cc
    # src/AR/ViewerAR.h
    # src/AR/ViewerAR.cc
    # )
    
    # target_link_libraries(MonoAR
    # ${LIBS}
    # )
    ```
    
- 빌드
    
    ```bash
    cd ORB_SLAM3
    chmod +x build_ros.sh
    ./build_ros.sh
    ```
    
- zshrc 추가
    
    ```bash
    sudo nano ~/.zshrc
    
    # 추가
    source $HOME/project/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build/devel/setup.zsh
    
    source ~/.zshrc
    ```
    

- `RealSense_D455.yaml`
    
    ```yaml
    %YAML:1.0
    
    #--------------------------------------------------------------------------------------------
    # Camera Parameters. Adjust them!
    #--------------------------------------------------------------------------------------------
    File.version: "1.0"
    
    Camera.type: "PinHole"
    
    # Right Camera calibration and distortion parameters (OpenCV)
    Camera1.fx: 381.124917
    Camera1.fy: 381.124917
    Camera1.cx: 320.000000
    Camera1.cy: 240.000000
    
    # dist1ortion parameters
    Camera1.k1: -0.062358
    Camera1.k2: 0.053711
    Camera1.k3: 0.0
    Camera1.p1: -0.002285
    Camera1.p2: -0.000221
    
    # Camera resolution
    Camera.width: 640
    Camera.height: 480
    
    # Camera frames per second 
    Camera.fps: 30
    
    # Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
    Camera.RGB: 1
    
    # Transformation from body-frame (imu) to left camera
    IMU.T_b_c1: !!opencv-matrix
       rows: 4
       cols: 4
       dt: f
       data: [1,0,0,-0.005,
             0,1,0,-0.005,
             0,0,1,0.0117,
             0.0, 0.0, 0.0, 1.0]
    
    # Do not insert KFs when recently lost
    IMU.InsertKFsWhenLost: 0
    
    # IMU noise (Use those from VINS-mono)
    IMU.NoiseGyro: 1e-3 # 2.44e-4 #1e-3 # rad/s^0.5
    IMU.NoiseAcc: 1e-2 # 1.47e-3 #1e-2 # m/s^1.5
    IMU.GyroWalk: 1e-6 # rad/s^1.5
    IMU.AccWalk: 1e-4 # m/s^2.5
    IMU.Frequency: 200.0
    
    #--------------------------------------------------------------------------------------------
    # ORB Parameters
    #--------------------------------------------------------------------------------------------
    # ORB Extractor: Number of features per image
    ORBextractor.nFeatures: 1250
    
    # ORB Extractor: Scale factor between levels in the scale pyramid 	
    ORBextractor.scaleFactor: 1.2
    
    # ORB Extractor: Number of levels in the scale pyramid	
    ORBextractor.nLevels: 8
    
    # ORB Extractor: Fast threshold
    # Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
    # Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
    # You can lower these values if your images have low contrast			
    ORBextractor.iniThFAST: 20
    ORBextractor.minThFAST: 7
    
    #--------------------------------------------------------------------------------------------
    # Viewer Parameters
    #--------------------------------------------------------------------------------------------
    Viewer.KeyFrameSize: 0.05
    Viewer.KeyFrameLineWidth: 1.0
    Viewer.GraphLineWidth: 0.9
    Viewer.PointSize: 2.0
    Viewer.CameraSize: 0.08
    Viewer.CameraLineWidth: 3.0
    Viewer.ViewpointX: 0.0
    Viewer.ViewpointY: -0.7
    Viewer.ViewpointZ: -3.5
    Viewer.ViewpointF: 500.0
    ```
    

- 실행
    
    ```bash
    # roscore
    # roslaunch 하면서 자동 실행함
    ```
    
    ```bash
    roslaunch realsense2_camera rs_camera.launch \
    enable_color:=true enable_depth:=false \
    color_width:=640 color_height:=480 color_fps:=30 \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true enable_accel:=true \
    initial_reset:=true enable_sync:=true 
    ```
    
    ```bash
    cd ORB_SLAM3
    
    rosrun ORB_SLAM3 Mono_Inertial \ 
    Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/RealSense_D455.yaml
    ```
    
    ```bash
    rosrun ORB_SLAM3 Mono_Inertial \
    ./Vocabulary/ORBvoc.txt ./Examples/RealSense_D455.yaml
    ```
