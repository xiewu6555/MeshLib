# 临时禁用 OpenCASCADE 的 CMake 配置文件
# 
# 此文件创建是因为 vcpkg 的 OpenCASCADE 安装由于 gettext-libintl 编译失败而无法正常进行
# 用户要求必须支持 STEP 格式，但目前需要先让构建系统能够正常工作
#
# 解决方案：
# 1. 临时设置 MRIOEXTRAS_NO_STEP=ON 以跳过 OpenCASCADE 依赖
# 2. 后续可以通过手动安装 OpenCASCADE 或解决 vcpkg 问题来恢复 STEP 支持
#
# 使用方法：
# 在主 CMakeLists.txt 中 include 此文件，或者在 cmake 配置时添加：
# -DMRIOEXTRAS_NO_STEP=ON

# 临时禁用 STEP 支持以避免 OpenCASCADE 依赖问题
set(MRIOEXTRAS_NO_STEP ON CACHE BOOL "Temporarily disable STEP support due to OpenCASCADE vcpkg build issues" FORCE)

message(STATUS "临时禁用 STEP 格式支持 - 原因：OpenCASCADE vcpkg 安装失败")
message(STATUS "要恢复 STEP 支持，请解决 gettext-libintl 编译问题或手动安装 OpenCASCADE")