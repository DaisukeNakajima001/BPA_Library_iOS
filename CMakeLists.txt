# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(BPA_Library)

# C++標準の設定
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# インクルードディレクトリの追加
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ../pcl/build-ios/install-ios/include/pcl-1.14
    ../flann/build-iOS/install-iOS/include
    ../lz4/build-iOS/install-iOS/include
    ../eigen/eigen3
    ../boost-iosx/frameworks/Headers
)

# ライブラリディレクトリの追加
link_directories(
    ../pcl/build-ios/install-ios/lib
    ../lz4/build-iOS/install-iOS/lib
    ../flann/build-iOS/install-iOS/lib
    ../boost-iosx/boost/stage/ios/lib
)

# ライブラリターゲットの追加
add_library(BPA_Library STATIC
    src/BPA.cpp
    src/IntersectionTest.cpp
)

# 必要なライブラリとのリンク
target_link_libraries(BPA_Library
    pcl_common
    pcl_io
    flann
    lz4
    boost_system
    # 必要に応じて他のPCLやBoostのライブラリを追加
)


# ライブラリのインストール設定（オプション）
install(TARGETS BPA_Library
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/
        DESTINATION include)
