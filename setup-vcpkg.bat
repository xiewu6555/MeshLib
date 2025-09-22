@echo off
setlocal enabledelayedexpansion

echo ===============================================
echo MeshLib 本地 vcpkg 初始化脚本
echo ===============================================

REM 检查是否已存在 vcpkg 目录
if exist "vcpkg" (
    echo 检测到现有的 vcpkg 目录
    choice /C YN /M "是否重新初始化 vcpkg? (Y/N)"
    if errorlevel 2 (
        echo 跳过 vcpkg 初始化
        goto :install_deps
    )
    echo 删除现有 vcpkg 目录...
    rmdir /s /q vcpkg
)

echo 1. 克隆 vcpkg 仓库...
git clone https://github.com/Microsoft/vcpkg.git
if errorlevel 1 (
    echo 错误: 克隆 vcpkg 仓库失败
    echo 请确保已安装 git 并有网络连接
    goto :error
)

echo 2. 切换到推荐版本...
cd vcpkg
git checkout 2024.10.21
cd ..

echo 3. 运行 bootstrap 脚本...
cd vcpkg
call bootstrap-vcpkg.bat
if errorlevel 1 (
    echo 错误: bootstrap 失败
    cd ..
    goto :error
)
cd ..

echo 4. 复制自定义 triplet...
if exist "thirdparty\vcpkg\triplets\x64-windows-meshlib.cmake" (
    if not exist "vcpkg\triplets\community" mkdir "vcpkg\triplets\community"
    copy "thirdparty\vcpkg\triplets\x64-windows-meshlib.cmake" "vcpkg\triplets\community\"
    echo 自定义 triplet 已复制
) else (
    echo 警告: 未找到自定义 triplet 文件
)

:install_deps
echo 5. 安装依赖包...
cd vcpkg
vcpkg install --triplet x64-windows-meshlib
if errorlevel 1 (
    echo 错误: 依赖包安装失败
    echo 请检查网络连接和 vcpkg.json 配置
    cd ..
    goto :error
)
cd ..

echo ===============================================
echo ✓ 本地 vcpkg 初始化完成！
echo ===============================================
echo.
echo 接下来可以运行:
echo   mkdir build ^&^& cd build
echo   cmake ..
echo   cmake --build . --config Release
echo.
goto :end

:error
echo ===============================================
echo ✗ 初始化失败！
echo ===============================================
echo 请检查错误信息并重试
exit /b 1

:end
echo 脚本执行完成
pause