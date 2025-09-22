# MeshLib vcpkg 独立配置说明

## 概述

MeshLib 采用**项目独立的 vcpkg 配置**，完全避免环境污染。每个项目在其根目录下拥有独立的 vcpkg 安装，确保依赖隔离和可重复构建。

## 配置文件说明

### vcpkg.json
主要的依赖清单文件，包含：
- 项目基本信息
- 核心依赖列表
- 可选特性（features）
- 平台特定依赖

### vcpkg-configuration.json
vcpkg 配置文件，指定：
- 基准版本（baseline）
- 注册表配置
- 自定义 triplet 路径

### .vcpkg
指定默认 triplet：`x64-windows-meshlib`

## 快速开始

### 1. 一键初始化（推荐）

```bash
# Windows
setup-vcpkg.bat

# Linux/macOS  
./setup-vcpkg.sh
```

### 2. 手动设置

```bash
# 1. 克隆本地 vcpkg
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
git checkout 2024.10.21

# 2. 初始化 vcpkg
# Windows
bootstrap-vcpkg.bat
# Linux/macOS
./bootstrap-vcpkg.sh

# 3. 返回项目根目录安装依赖
cd ..
vcpkg/vcpkg install --triplet x64-windows-meshlib  # Windows
vcpkg/vcpkg install --triplet x64-linux            # Linux
```

### 3. 构建项目

```bash
# 创建构建目录
mkdir build && cd build

# 配置 CMake（自动使用项目本地的 vcpkg）
cmake ..

# 构建
cmake --build . --config Release
```

### 4. 启用可选特性

在 `vcpkg.json` 中定义了多个特性：

- `opencascade`: CAD几何建模支持
- `python`: Python 绑定支持  
- `gui`: 图形界面支持（Linux）
- `compression`: 压缩库支持
- `threading`: 多线程支持
- `mesh-formats`: 额外的网格格式支持

使用 CMake 启用特性：
```bash
cmake .. -DVCPKG_MANIFEST_FEATURES="opencascade;python;threading"
```

## 迁移说明

### 从旧配置迁移

1. **保留现有方式**：旧的 `thirdparty/install.bat` 脚本仍然可用
2. **新方式优势**：
   - 依赖隔离
   - 版本锁定
   - 跨平台一致性
   - 更好的团队协作

### 目录结构

```
MeshLib/
├── vcpkg/                  # 项目本地 vcpkg 安装（自动生成）
│   ├── vcpkg.exe          # vcpkg 可执行文件
│   ├── installed/         # 已安装的包
│   └── scripts/           # 构建脚本
├── vcpkg.json              # 依赖清单
├── vcpkg-configuration.json # vcpkg配置  
├── .vcpkg                  # 默认triplet
├── setup-vcpkg.bat         # Windows 初始化脚本
├── setup-vcpkg.sh          # Linux/macOS 初始化脚本
├── thirdparty/             # 现有第三方库（逐步迁移）
└── requirements/           # 平台特定需求文件（向后兼容）
```

**注意**: `vcpkg/` 目录不应提交到版本控制，已在 `.gitignore` 中排除。

## 故障排除

### 常见问题

1. **vcpkg 未找到**
   - 确保 vcpkg 在 PATH 中，或设置 `CMAKE_TOOLCHAIN_FILE`

2. **triplet 错误**
   - 确保自定义 triplet 文件存在：`thirdparty/vcpkg/triplets/x64-windows-meshlib.cmake`

3. **依赖安装失败**
   - 检查网络连接
   - 尝试清理 vcpkg 缓存：`vcpkg remove --outdated`

### 调试命令

```bash
# 查看 vcpkg 版本
vcpkg version

# 列出已安装的包
vcpkg list

# 检查 manifest 文件
vcpkg x-check-support --triplet x64-windows-meshlib
```

## 独立配置的优势

1. **完全依赖隔离**：项目拥有独立的 vcpkg 安装，互不干扰
2. **无环境污染**：不依赖全局 vcpkg 安装，不影响其他项目
3. **可重现构建**：版本锁定确保任何环境下的一致性
4. **团队协作**：所有团队成员使用相同的依赖版本
5. **易于维护**：依赖更新只影响当前项目
6. **简化部署**：新环境只需运行初始化脚本即可
7. **版本控制友好**：只需提交配置文件，不提交二进制包

## 兼容性

- **CMake 3.16+**
- **vcpkg 2024.10.21+**
- **Visual Studio 2019/2022** (Windows)
- **GCC 9+** (Linux)
- **Xcode 12+** (macOS)