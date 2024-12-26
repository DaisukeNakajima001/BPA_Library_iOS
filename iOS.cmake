# iOS用のCMakeツールチェーンファイル
set(CMAKE_SYSTEM_NAME iOS)

# iOSデプロイメントターゲット
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.4")

# iOS SDKパス
set(CMAKE_OSX_SYSROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS.sdk")

# 対応するアーキテクチャ
set(CMAKE_OSX_ARCHITECTURES "arm64")

# コンパイラの設定
set(CMAKE_C_COMPILER "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang")
set(CMAKE_CXX_COMPILER "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang++")

# ポジション独立コードの有効化
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Simulator用アーキテクチャを無効化
set(CMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH YES)
