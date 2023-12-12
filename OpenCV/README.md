# OpenCV Installation Guide

This project relies on a statically linked version of OpenCV world, it is configured to rely on version 4.8.0, but there is no strict reliance on this version. It is assumed that you wish to build OpenCV for Microsoft HoloLens 2 on a x86-64 CPU on Windows. This guide builds a very comprehensive version of OpenCV with every module that is capable of running on HoloLens2. This is not necessary for the IR Tracking, so feel free to adjust the built modules if you wish to have a smaller static library.

#### Building OpenCV using this guide is confirmed working on Windows 11 22H2 with Visual Studio Community 2022, but it should also work on other versions of Windows 11 or 10.


## Step-by-Step Guide
1. Install VS 2022 Community with all components required for cross-compiling for ARM64
1. Open the CMakeToolchain.txt and adjust the paths for your specific MSVC version and VS installation directory
1. Clone the OpenCV repository from GitHub (https://github.com/opencv/opencv/tree/4.8.0)
1. Checkout the version 4.8.0 tag on the git repository (git checkout 4.8.0)
1. Open "modules\dnn\src\layers\cpu_kernels\conv_block.simd.hpp" using a text editor of choice and modify lines 455-461 as follows:
    - Original content:
    ```
    c0 += vld1q_f32(c);
    c1 += vld1q_f32(c + 4);
    c2 += vld1q_f32(c + 8);
    c3 += vld1q_f32(c + 12);
    c4 += vld1q_f32(c + 16);
    c5 += vld1q_f32(c + 20);
    c6 += vld1q_f32(c + 24);
    ```
    - New content:
    ```
    c0 = vaddq_f32(c0, vld1q_f32(c));
    c1 = vaddq_f32(c1, vld1q_f32(c + 4));
    c2 = vaddq_f32(c2, vld1q_f32(c + 8));
    c3 = vaddq_f32(c3, vld1q_f32(c + 12));
    c4 = vaddq_f32(c4, vld1q_f32(c + 16));
    c5 = vaddq_f32(c5, vld1q_f32(c + 20));
    c6 = vaddq_f32(c6, vld1q_f32(c + 24));
    ```
    - Thanks to GitHub user zixianweei for this fix provided here https://github.com/opencv/opencv/issues/24006
    - OpenCV 4.x already has a fix for this, so if you compile version 4.8.2 or greater this step should be unnecessary (As of writing, 4.8.1 is the newest version, which does not include a fix)
1. Configure using CMAKE GUI as follows:
    - Set source code directory to the cloned repository
    - Set build directory
    - Press "Configure"
    - On configure:
      - Generator to Visual Studio 17 2022
      - Set platform to ARM64
      - Check "Specify toolchain file for cross-compiling"
      - Press "Next"
      - Select the CMakeToolchain.txt file from the folder that contains this README
      - Press "Finish"
    - Settings:
      - Enable BUILD_opencv_world
      - Disable BUILD_shared_libs
      - Disable BUILD_TESTS
    - Press "Generate"
    - Press "Open Project"
1. Within Visual Studio
    - Make sure the Solution Configuration is "Release" on "ARM64"
    - In solution explorer, select the following projects and open their properties:
        - all projects within 3rdparty
        - opencv_world
        - carotene
        - carotene_objs
        - jsimd
        - tegra_hal
    - In properties, go to Configuration Properties > C/C++ > Code Generation
    - Change the setting "Runtime Library" from /MT to /MD
    - Build opencv_world
1. Copy "opencv_world480.lib" from "<your_opencv_build_folder>\lib\Release" to "HoloLens2-IRTracking\OpenCV\lib\ARM64\Release/"
1. Copy all ".lib" files from "<your_opencv_build_folder>\3rdparty\lib\Release" to "HoloLens2-IRTracking\OpenCV\3rdparty\lib\ARM64\Release"
