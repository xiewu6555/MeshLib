#!/bin/bash

echo "==============================================="
echo "MeshLib 本地 vcpkg 初始化脚本"
echo "==============================================="

# 检查是否已存在 vcpkg 目录
if [ -d "vcpkg" ]; then
    echo "检测到现有的 vcpkg 目录"
    read -p "是否重新初始化 vcpkg? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "跳过 vcpkg 初始化，直接安装依赖..."
        cd vcpkg && ./vcpkg install --triplet x64-linux && cd ..
        exit 0
    fi
    echo "删除现有 vcpkg 目录..."
    rm -rf vcpkg
fi

# 检查 git 是否已安装
if ! command -v git &> /dev/null; then
    echo "错误: 未找到 git 命令"
    echo "请先安装 git: sudo apt-get install git"
    exit 1
fi

echo "1. 克隆 vcpkg 仓库..."
git clone https://github.com/Microsoft/vcpkg.git
if [ $? -ne 0 ]; then
    echo "错误: 克隆 vcpkg 仓库失败"
    echo "请确保已安装 git 并有网络连接"
    exit 1
fi

echo "2. 切换到推荐版本..."
cd vcpkg
git checkout 2024.10.21
cd ..

echo "3. 运行 bootstrap 脚本..."
cd vcpkg
./bootstrap-vcpkg.sh
if [ $? -ne 0 ]; then
    echo "错误: bootstrap 失败"
    cd ..
    exit 1
fi
cd ..

echo "4. 复制自定义 triplet（如果存在）..."
if [ -f "thirdparty/vcpkg/triplets/x64-linux-meshlib.cmake" ]; then
    mkdir -p "vcpkg/triplets/community"
    cp "thirdparty/vcpkg/triplets/x64-linux-meshlib.cmake" "vcpkg/triplets/community/"
    echo "自定义 triplet 已复制"
fi

echo "5. 安装依赖包..."
cd vcpkg

# 根据平台选择 triplet
if [[ "$OSTYPE" == "darwin"* ]]; then
    TRIPLET="x64-osx"
else
    TRIPLET="x64-linux"
fi

echo "使用 triplet: $TRIPLET"
./vcpkg install --triplet $TRIPLET
if [ $? -ne 0 ]; then
    echo "错误: 依赖包安装失败"
    echo "请检查网络连接和 vcpkg.json 配置"
    cd ..
    exit 1
fi
cd ..

echo "==============================================="
echo "✓ 本地 vcpkg 初始化完成！"
echo "==============================================="
echo
echo "接下来可以运行:"
echo "  mkdir build && cd build"
echo "  cmake .."
echo "  cmake --build . --config Release"
echo

echo "脚本执行完成"