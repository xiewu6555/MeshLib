# CLAUDE.md

此文件为 Claude Code (claude.ai/code) 在此仓库中工作提供指导。

## 项目概览

MeshLib 是一个开源的3D数据处理SDK，用C++编写，提供C、C#和Python绑定。它提供先进的网格处理、布尔运算、孔洞填充、简化等算法。该库跨平台（Windows、macOS、Linux、WebAssembly），并通过CUDA支持GPU加速。

## 构建系统

本项目使用CMake作为主要构建系统。关键组件：

- **主CMakeLists.txt**: 根配置文件，包含构建选项
- **构建脚本**: 位于 `scripts/` 目录
  - `scripts/build_thirdparty.sh`: 构建第三方依赖
  - `scripts/build_source.sh`: 构建主项目

### 常用构建命令

```bash
# 首先构建第三方依赖
./scripts/build_thirdparty.sh

# 构建主项目
./scripts/build_source.sh

# 直接使用CMake（从项目根目录）
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

在Windows上，使用Visual Studio生成器：
```cmd
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

## 测试

### Python测试
位置：`test_python/`
```bash
# Linux/macOS
cd build/Debug/bin  # 或 build/Release/bin
python3 ../../../scripts/run_python_test_script.py -d '../test_python'

# Windows
cd source/x64/Debug  # 或 source/x64/Release  
py -3 ..\..\..\scripts\run_python_test_script.py -d '..\test_python'
```

### C++测试
- 使用GoogleTest框架
- 当启用 `BUILD_TESTING` 时构建测试
- 从构建目录运行 `ctest`

### C#测试
```bash
python3 scripts/run_c_sharp_unit_test_script.py
```

### C测试
```bash
python3 scripts/run_c_unit_test_script.py
python3 scripts/run_c2_unit_test_script.py
```

## 项目架构

### 核心组件

- **MRMesh**: 核心网格数据结构和算法（`source/MRMesh/`）
- **MRViewer**: 基于OpenGL的3D查看器，带ImGui界面（`source/MRViewer/`）
- **MRCuda**: GPU加速算法（`source/MRCuda/`）
- **MRIOExtras**: 额外的I/O格式支持（`source/MRIOExtras/`）
- **MRCommonPlugins**: 标准插件（`source/MRCommonPlugins/`）

### 语言绑定

- **Python**: `source/mrmeshpy/` - 主要Python绑定
- **C#**: `source/MRDotNet/` - .NET绑定  
- **C**: `source/MRMeshC/` 和 `source/MRLibC2/` - C绑定

### 关键数据结构

该库使用半边数据结构进行网格表示，确保流形合规性。核心类型包括：
- `Mesh` - 带半边连接的三角网格
- `PointCloud` - 3D点集合
- `Polyline` - 连接的线段
- `VoxelGrid` - 体积数据表示

## 代码风格指南

来自 `CONTRIBUTING.md`：

- **缩进**: 4个空格，大括号单独一行
- **命名规范**:
  - 命名空间、类型: `FooBar`
  - 函数、变量: `fooBar`  
  - 宏: `MR_FOO_BAR`
  - 私有成员: 后缀 `_`
  - 常量: 前缀 `c`
- **空格**: 在非空的 `(...)` 和 `{...}` 内部添加空格
- **导出宏**: 公共函数使用 `MRFOO_API`，与 `typeid` 一起使用的类使用 `MRFOO_CLASS`

## 依赖项

- **第三方库**: 位于 `thirdparty/` 目录
- **Python要求**: 使用 `pip install pytest numpy` 安装
- **预提交钩子**: `pip install pre-commit black isort`

## 开发工作流

1. 首先构建第三方依赖
2. 配置并构建主项目
3. 运行测试验证功能
4. 对Python代码使用预提交钩子进行格式化

## 平台特定说明

### Windows
- 需要Visual Studio进行C++开发
- 某些开发工具使用MSYS2
- Python测试使用 `py -3` 命令

### Linux/macOS  
- 使用标准构建工具和包管理器
- Python测试使用 `python3` 命令

## 性能特性

- 通过CUDA进行GPU加速（如果可用）
- 针对布尔运算和简化的优化算法
- 多线程处理支持
- 内存高效的数据结构