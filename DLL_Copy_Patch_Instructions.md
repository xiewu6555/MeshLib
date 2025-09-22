# MeshViewer DLL 自动拷贝配置说明

## 版本信息
- 版本：v1.0
- 日期：2025-09-12
- 作者：Claude Code

## 概述
本补丁为 MeshViewer.exe 添加自动 DLL 拷贝功能，确保运行时能找到所有必需的依赖库。

## 已创建的文件

### 1. CopyRuntimeDependencies.cmake 模块
位置：`F:/Code/OpenProject/MeshLib/cmake/Modules/CopyRuntimeDependencies.cmake`

该模块包含以下功能：
- `copy_vcpkg_dlls()` - 自动拷贝 vcpkg 安装的 DLL 文件
- `copy_manual_opencascade_dlls()` - 拷贝手动安装的 OpenCASCADE DLL 文件
- `copy_runtime_dependencies()` - 主函数，统一调用所有拷贝功能

## 需要手动修改的文件

### 1. 修改 MRViewerApp/CMakeLists.txt
文件：`F:/Code/OpenProject/MeshLib/source/MRViewerApp/CMakeLists.txt`

#### 修改点 1：添加模块包含
在 `project(MeshViewer CXX)` 行之后添加：
```cmake
# 包含运行时依赖拷贝模块
include(CopyRuntimeDependencies)
```

#### 修改点 2：添加 DLL 拷贝配置
在 `ENDIF() # MR_EMSCRIPTEN` 行之后添加：
```cmake
# 配置运行时依赖 DLL 拷贝（仅限 Windows）
IF(WIN32 AND NOT MR_EMSCRIPTEN)
  copy_runtime_dependencies(${PROJECT_NAME})
ENDIF()
```

## DLL 拷贝范围

### 1. vcpkg 库 DLL
从 `./vcpkg/installed/x64-windows-meshlib/bin/` 目录拷贝所有 DLL：
- boost 相关 DLL
- fmt.dll
- spdlog.dll  
- tbb12.dll
- freetype.dll
- 图像处理库 DLL（png, jpeg, tiff 等）
- GDCM 相关 DLL
- OpenVDB 相关 DLL
- 其他 vcpkg 依赖

### 2. 手动 OpenCASCADE DLL
从 `./thirdparty/opencascade/occt_vc14-64/win64/vc14/bin/` 拷贝：
- 所有 TK*.dll 文件（约 80+ 个）
- OpenCASCADE 核心库

### 3. 第三方依赖 DLL
从 `./thirdparty/opencascade/3rdparty-vc14-64/bin/` 拷贝（如果存在）：
- OpenCASCADE 的第三方依赖库

## 工作原理

1. **构建时拷贝**：使用 `add_custom_command(TARGET ... POST_BUILD)` 在构建完成后自动拷贝
2. **只拷贝已更改的文件**：使用 `copy_if_different` 避免不必要的拷贝
3. **条件执行**：仅在 Windows 且非 Emscripten 环境下执行
4. **详细日志**：提供详细的拷贝过程信息

## 预期效果

配置完成后：
1. 每次构建 MeshViewer 时，会自动拷贝所有必需的 DLL 到 `build_final/bin/` 目录
2. MeshViewer.exe 可以独立运行，无需设置 PATH 环境变量
3. 支持 Debug 和 Release 两种配置

## 测试步骤

1. 应用上述修改
2. 重新配置并构建项目：
   ```bash
   cd build_final
   cmake .. -G "Visual Studio 17 2022" -A x64
   cmake --build . --config Release
   ```
3. 检查 `build_final/bin/` 目录是否包含所有 DLL 文件
4. 直接运行 `MeshViewer.exe` 测试是否正常启动

## 注意事项

1. 确保 `USE_MANUAL_OPENCASCADE=ON` 或正确配置了 OpenCASCADE 路径
2. 如果 vcpkg triplet 不是 `x64-windows-meshlib`，需要相应调整模块中的路径
3. 首次运行可能需要较长时间拷贝所有 DLL 文件
4. 如果发现缺少 DLL，可以在模块中添加额外的拷贝逻辑

## 故障排除

### 问题：找不到 vcpkg DLL
- 检查 `CMAKE_TOOLCHAIN_FILE` 是否正确指向 vcpkg.cmake
- 验证 `VCPKG_TARGET_TRIPLET` 设置

### 问题：找不到 OpenCASCADE DLL  
- 确认 `USE_MANUAL_OPENCASCADE=ON`
- 检查 OpenCASCADE 安装路径

### 问题：MeshViewer.exe 仍然找不到 DLL
- 使用 `ldd MeshViewer.exe` 或 Dependency Walker 检查缺失的 DLL
- 手动添加特定的 DLL 拷贝逻辑到模块中

## 完整的修改后的 MRViewerApp/CMakeLists.txt 内容

```cmake
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CXX_STANDARD ${MR_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MeshViewer CXX)

# 包含运行时依赖拷贝模块
include(CopyRuntimeDependencies)

# We make this a `MACOSX_BUNDLE` even though we don't package it, solely to unify
#   the `MR_LOCAL_RESOURCES=1` search path between this and our other apps that actually need to be bundles.
add_executable(${PROJECT_NAME} WIN32 MACOSX_BUNDLE MRViewerApp.cpp)

file(GLOB PNGS "*.png")
file(COPY ${PNGS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

IF(MR_EMSCRIPTEN)
  file(GLOB LOCAL_WASM_FILES "../../wasm/*.*")
  file(COPY ${LOCAL_WASM_FILES} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  target_link_libraries(${PROJECT_NAME} PRIVATE
    zip
    gtest
    -Wl,--whole-archive
    MRCommonPlugins
    MRMesh
    MRIOExtras
    MRViewer
    -Wl,--no-whole-archive
  )
ELSE() # NOT MR_EMSCRIPTEN
  IF(APPLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _GNU_SOURCE) #for Boost.Stacktrace
  ENDIF()
  target_link_libraries(${PROJECT_NAME} PRIVATE
    MRViewer
    MRMesh
  )
ENDIF() # MR_EMSCRIPTEN

# 配置运行时依赖 DLL 拷贝（仅限 Windows）
IF(WIN32 AND NOT MR_EMSCRIPTEN)
  copy_runtime_dependencies(${PROJECT_NAME})
ENDIF()

install(TARGETS ${PROJECT_NAME} DESTINATION "${MR_BIN_DIR}")
install(FILES ${LIB_LIST} DESTINATION "${MR_RESOURCES_DIR}")
install(FILES ${PNGS} DESTINATION "${MR_RESOURCES_DIR}")

IF(MR_EMSCRIPTEN)
  install(FILES ${LOCAL_WASM_FILES} DESTINATION "${MR_RESOURCES_DIR}/wasm")
ENDIF()

IF(MR_PCH)
  TARGET_PRECOMPILE_HEADERS(${PROJECT_NAME} REUSE_FROM MRPch)
ENDIF()
```