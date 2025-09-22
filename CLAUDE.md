# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作提供指导。版本：v2.0

## 项目概览

MeshLib 是一个用 C++ 编写的开源 3D 数据处理 SDK，支持 C、C# 和 Python 绑定。它提供高级的网格处理、布尔运算、孔洞填补、简化等算法。该库跨平台（Windows、macOS、Linux、WebAssembly）并支持 CUDA GPU 加速。

### 最新功能支持 (v2.0 - 2025-09-13)
- ✅ **STEP 格式完整支持**：通过手动安装的 OpenCASCADE 7.9.1
- ✅ **体素数据处理**：通过 GDCM 3.0.24 支持医学图像格式  
- ✅ **E57 点云格式**：完整支持点云数据导入/导出
- ✅ **自动化 DLL 管理**：134 个运行时依赖自动拷贝
- ✅ **现代 CMake 架构**：模块化配置和依赖管理

### 快速构建（Windows）
```bash
# 1. 克隆项目后运行
setup-vcpkg.bat                    # 初始化 vcpkg
build_with_manual_occt.bat         # 构建项目

# 2. 运行主界面
cd build_final/bin
MeshViewer.exe                     # 启动 3D 查看器
```

## 构建系统

本项目使用现代 CMake 作为主要构建系统。关键组件：

- **主 CMakeLists.txt**：根配置文件，包含构建选项
- **构建脚本**：位于 `scripts/` 目录
  - `scripts/build_thirdparty.sh`：构建第三方依赖
  - `scripts/build_source.sh`：构建主项目
- **CMake 模块**：位于 `cmake/Modules/` 目录，包含平台特定配置

### vcpkg 项目独立配置

项目采用**完全独立的 vcpkg 配置**，避免环境污染：

**配置文件：**
- `vcpkg.json` - 主要依赖清单
- `vcpkg-configuration.json` - vcpkg 配置
- `.vcpkg` - 默认 triplet 设置
- `setup-vcpkg.bat/sh` - 一键初始化脚本

**快速开始：**
```bash
# 一键初始化（推荐）
setup-vcpkg.bat        # Windows
./setup-vcpkg.sh       # Linux/macOS

# 构建项目
mkdir build && cd build
cmake ..               # 自动使用项目本地 vcpkg
cmake --build . --config Release
```

**可选特性：**
```bash
# 启用特定特性
cmake .. -DVCPKG_MANIFEST_FEATURES="opencascade;python;threading"
```

### Windows 特定配置

```cmake
# Windows SDK 版本设置
set(CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION "10.0.22621.0")

# CUDA 工具包路径（可选）
set(CUDAToolkit_ROOT "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.4/")

# 默认使用自定义 triplet
set(VCPKG_TARGET_TRIPLET "x64-windows-meshlib")
```

### 常用构建命令

**Linux/macOS:**
```bash
# 首先构建第三方依赖
./scripts/build_thirdparty.sh

# 构建主项目
./scripts/build_source.sh

# 直接使用 CMake（从项目根目录）
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

**Windows（推荐使用 Visual Studio）:**
```cmd
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

**现代 CMake 实践：**
- 使用 C++20 标准（可通过 MR_CXX_STANDARD 配置为 23）
- 支持 vcpkg manifest 模式（vcpkg.json）
- 启用 unity builds 以提高编译速度
- 配置正确的 Windows SDK 版本
- 依赖隔离和版本锁定

**推荐的新项目构建方式：**
```bash
# 一键初始化本地 vcpkg（仅需运行一次）
setup-vcpkg.bat        # Windows
./setup-vcpkg.sh       # Linux/macOS

# 构建项目
mkdir build && cd build
cmake ..               # 自动使用 ./vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

## 测试

### Python 测试
位置：`test_python/`
```bash
# Linux/macOS
cd build/Debug/bin  # 或 build/Release/bin
python3 ../../../scripts/run_python_test_script.py -d '../test_python'

# Windows
cd source/x64/Debug  # 或 source/x64/Release  
py -3 ..\..\..\scripts\run_python_test_script.py -d '..\test_python'
```

### C++ 测试
- 使用 GoogleTest 框架
- 当 `BUILD_TESTING` 启用时构建测试
- 在构建目录中使用 `ctest` 运行

### C# 测试
```bash
python3 scripts/run_c_sharp_unit_test_script.py
```

### C 测试
```bash
python3 scripts/run_c_unit_test_script.py
python3 scripts/run_c2_unit_test_script.py
```

### 测试注意事项
- Python 测试脚本会自动检测 vcpkg Python 版本
- 确保在正确的构建目录中运行测试
- Windows 上使用 `py -3` 命令，Linux/macOS 使用 `python3`

## 项目架构

### 核心组件

- **MRMesh**：核心网格数据结构和算法（`source/MRMesh/`）
- **MRViewer**：基于 OpenGL 的 3D 查看器，使用 ImGui 界面（`source/MRViewer/`）
- **MRCuda**：GPU 加速算法（`source/MRCuda/`）
- **MRIOExtras**：附加 I/O 格式支持（`source/MRIOExtras/`）
- **MRCommonPlugins**：标准插件（`source/MRCommonPlugins/`）

### 语言绑定

- **Python**：`source/mrmeshpy/` - 主要 Python 绑定
- **C#**：`source/MRDotNet/` - .NET 绑定  
- **C**：`source/MRMeshC/` 和 `source/MRLibC2/` - C 绑定

### 关键数据结构

库使用半边数据结构进行网格表示，确保流形合规性。核心类型包括：
- `Mesh` - 具有半边连接的三角网格
- `PointCloud` - 3D 点云集合
- `Polyline` - 连接的线段
- `VoxelGrid` - 体素数据表示

### CMake 架构

- **DefaultOptions.cmake**：设置默认编译选项和 C++ 标准
- **ConfigureVcpkg.cmake**：配置 vcpkg 包管理器
- **DetectPlatform.cmake**：平台检测逻辑
- **CompilerOptions.cmake**：编译器特定选项

## 代码风格指南

基于 `CONTRIBUTING.md`：

- **缩进**：4 个空格，大括号单独占行
- **命名约定**：
  - 命名空间、类型：`FooBar`
  - 函数、变量：`fooBar`  
  - 宏：`MR_FOO_BAR`
  - 私有成员：后缀 `_`
  - 常量：前缀 `c`
- **间距**：在非空的 `(...)` 和 `{...}` 内部添加空格
- **导出宏**：公共函数使用 `MRFOO_API`，用于 `typeid` 的类使用 `MRFOO_CLASS`

### 现代 CMake 指导原则

- 使用目标导向的 CMake（target-based）
- 避免全局变量，优先使用 target_* 命令
- 正确处理第三方依赖的 find_package
- 使用适当的可见性（PUBLIC、PRIVATE、INTERFACE）

## 依赖管理

### 核心依赖配置 (v2.0)

**vcpkg.json 关键依赖**：
- `libe57format` - E57 点云格式支持（解决 E57Format.dll 缺失）
- `xerces-c` - XML 解析库（解决 Xerces DLL 缺失）
- `gdcm` - DICOM 医学图像库（MRVoxels 模块支持）
- `boost-*`, `eigen3`, `fmt`, `glfw3`, `jsoncpp`, `spdlog`, `tbb` 等

**手动依赖**：
- **OpenCASCADE 7.9.1**：位于 `thirdparty/opencascade/occt_vc14-64/`
  - 提供 STEP 格式完整支持（83 个 TK*.dll）
  - 避免 vcpkg gettext-libintl 编译问题

**自动 DLL 管理**：
- `cmake/Modules/CopyRuntimeDependencies.cmake` - 自动拷贝 134 个依赖 DLL
- `cmake/Modules/UseManualOpenCASCADE.cmake` - 手动 OpenCASCADE 集成

### 传统依赖

- **第三方库**：位于 `thirdparty/` 目录
- **Python 要求**：使用 `pip install pytest numpy` 安装
- **预提交钩子**：`pip install pre-commit black isort`
- **vcpkg 包管理**：项目配置了特定的 vcpkg triplet（x64-windows-meshlib）
- **Windows 特定**：Visual Studio 2022，Windows 10 SDK (10.0.22621.0)
- **CUDA 支持**：CUDA 11.4（可选，用于 GPU 加速）

## 开发工作流

1. 首先构建第三方依赖
2. 配置并构建主项目
3. 运行测试验证功能
4. 使用预提交钩子进行 Python 代码格式化
5. 遵循现代 CMake 实践进行新功能开发
6. 确保跨平台兼容性

## 平台特定说明

### Windows
- 需要 Visual Studio 进行 C++ 开发（推荐 VS 2022）
- 某些开发工具使用 MSYS2
- Python 测试使用 `py -3` 命令
- 预配置了特定的 vcpkg 路径和 Windows SDK 版本
- CUDA 支持需要 NVIDIA GPU Computing Toolkit

### Linux/macOS  
- 使用标准构建工具和包管理器
- Python 测试使用 `python3` 命令
- 支持 Emscripten 构建（WebAssembly）

## 性能特性

- 通过 CUDA 实现 GPU 加速（如果可用）
- 针对布尔运算和简化的优化算法
- 多线程处理支持
- 内存高效的数据结构
- Unity builds 支持以提高编译速度
- 编译时间跟踪（Linux 上使用 /usr/bin/time）

## 重要开发注意事项

- **CMake 配置**：项目使用现代 CMake 3.16+，遵循目标导向的构建方式
- **C++ 标准**：默认使用 C++20，可配置为 C++23
- **依赖管理**：推荐使用 `vcpkg.json` 进行依赖管理，实现依赖隔离
- **vcpkg 配置**：项目支持自动检测 vcpkg 安装，无需手动设置路径
- **测试执行**：Python 测试脚本会自动检测合适的 Python 版本和路径
- **插件开发**：新增了工具路径插件支持，参考 `README_ToolPathPlugin.md`
- **迁移指导**：详细配置说明请参考 `VCPKG_SETUP.md`