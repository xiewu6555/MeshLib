# 使用手动安装的 OpenCASCADE 7.9.1 配置文件
#
# 此文件配置 CMake 使用手动下载并安装的 OpenCASCADE 7.9.1 预编译版本
# 用于替代 vcpkg 的 OpenCASCADE 包，以支持 STEP 格式
#
# 使用方法：
# 在主 CMakeLists.txt 中 include 此文件，或者在 cmake 配置时添加：
# -DUSE_MANUAL_OPENCASCADE=ON

if(USE_MANUAL_OPENCASCADE OR DEFINED OPENCASCADE_MANUAL_ROOT)
    # 设置 OpenCASCADE 手动安装路径
    if(NOT DEFINED OPENCASCADE_MANUAL_ROOT)
        set(OPENCASCADE_MANUAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencascade/occt_vc14-64" CACHE PATH "OpenCASCADE 手动安装根目录")
    endif()
    
    message(STATUS "使用手动安装的 OpenCASCADE: ${OPENCASCADE_MANUAL_ROOT}")
    
    # 检查手动安装的 OpenCASCADE 是否存在
    if(NOT EXISTS "${OPENCASCADE_MANUAL_ROOT}/cmake/OpenCASCADEConfig.cmake")
        message(FATAL_ERROR "在 ${OPENCASCADE_MANUAL_ROOT} 找不到手动安装的 OpenCASCADE")
    endif()
    
    # 设置 OpenCASCADE_DIR 以便 find_package 能够找到
    set(OpenCASCADE_DIR "${OPENCASCADE_MANUAL_ROOT}/cmake" CACHE PATH "OpenCASCADE CMake 配置目录" FORCE)
    
    # 设置库和包含目录
    set(OpenCASCADE_INCLUDE_DIR "${OPENCASCADE_MANUAL_ROOT}/inc" CACHE PATH "OpenCASCADE 头文件目录" FORCE)
    
    # 为了兼容期望头文件在 opencascade/ 子目录的代码，我们需要确保包含路径正确
    # 创建一个符号链接或直接使用父目录作为包含路径
    if(NOT EXISTS "${OPENCASCADE_MANUAL_ROOT}/inc/opencascade")
        file(MAKE_DIRECTORY "${OPENCASCADE_MANUAL_ROOT}/inc/opencascade")
        # 注意：在 Windows 上我们可能需要复制文件而不是创建符号链接
    endif()
    set(OpenCASCADE_LIBRARY_DIR "${OPENCASCADE_MANUAL_ROOT}/win64/vc14/lib" CACHE PATH "OpenCASCADE 库文件目录" FORCE)
    set(OpenCASCADE_BINARY_DIR "${OPENCASCADE_MANUAL_ROOT}/win64/vc14/bin" CACHE PATH "OpenCASCADE 二进制文件目录" FORCE)
    
    # 确保 STEP 支持未被禁用
    set(MRIOEXTRAS_NO_STEP OFF CACHE BOOL "启用 STEP 格式支持" FORCE)
    
    # 设置第三方依赖路径
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencascade/3rdparty-vc14-64")
        set(OpenCASCADE_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencascade/3rdparty-vc14-64" CACHE PATH "OpenCASCADE 第三方依赖目录" FORCE)
    endif()
    
    message(STATUS "OpenCASCADE 配置:")
    message(STATUS "  版本: 7.9.1")
    message(STATUS "  根目录: ${OPENCASCADE_MANUAL_ROOT}")
    message(STATUS "  CMake 目录: ${OpenCASCADE_DIR}")
    message(STATUS "  头文件目录: ${OpenCASCADE_INCLUDE_DIR}")
    message(STATUS "  库目录: ${OpenCASCADE_LIBRARY_DIR}")
    message(STATUS "  二进制目录: ${OpenCASCADE_BINARY_DIR}")
    message(STATUS "  STEP 支持: 已启用")

else()
    message(STATUS "使用默认的 OpenCASCADE 配置（vcpkg 或系统安装）")
endif()