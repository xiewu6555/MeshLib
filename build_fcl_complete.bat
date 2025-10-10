@echo off
REM ========================================
REM FCL 完整构建脚本
REM 用途: 自动构建 libccd -> FCL -> MRTest 的完整流程
REM ========================================

setlocal enabledelayedexpansion

echo ========================================
echo FCL 完整构建流程
echo ========================================
echo.
echo 此脚本将执行以下步骤:
echo   1. 构建并安装 libccd
echo   2. 配置并编译 FCL (作为子模块)
echo   3. 配置并编译 MRTest (包含 FCL 性能测试)
echo.

set SCRIPT_DIR=%~dp0
set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%

REM ========================================
REM 步骤 1: 构建 libccd
REM ========================================
echo ========================================
echo 步骤 1/3: 构建 libccd
echo ========================================

if exist "%SCRIPT_DIR%\thirdparty\libccd_install\lib\ccd.lib" (
    echo libccd 已安装，跳过构建
    echo 如需重新构建，请删除目录: thirdparty\libccd_install
) else (
    echo 开始构建 libccd...
    call "%SCRIPT_DIR%\build_libccd.bat"
    if !ERRORLEVEL! neq 0 (
        echo 错误: libccd 构建失败
        pause
        exit /b 1
    )
)

echo.
echo ========================================
echo 步骤 2/3: 构建 FCL
echo ========================================

set FCL_BUILD=%SCRIPT_DIR%\thirdparty\fcl\build

REM 清理旧的 FCL 构建
if exist "%FCL_BUILD%" (
    echo 清理旧的 FCL 构建目录...
    rmdir /s /q "%FCL_BUILD%"
)

mkdir "%FCL_BUILD%"
cd /d "%FCL_BUILD%"

echo 配置 FCL...
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DFCL_STATIC_LIBRARY=ON ^
    -DFCL_WITH_OCTOMAP=OFF ^
    -DBUILD_TESTING=OFF ^
    -DCMAKE_INSTALL_PREFIX="%SCRIPT_DIR%\thirdparty\fcl_install"

if !ERRORLEVEL! neq 0 (
    echo 错误: FCL 配置失败
    cd /d "%SCRIPT_DIR%"
    pause
    exit /b 1
)

echo ✓ FCL 配置成功

echo 编译 FCL (Release)...
cmake --build . --config Release --parallel

if !ERRORLEVEL! neq 0 (
    echo 错误: FCL 编译失败
    cd /d "%SCRIPT_DIR%"
    pause
    exit /b 1
)

echo ✓ FCL 编译成功

echo 安装 FCL...
cmake --install . --config Release

if !ERRORLEVEL! neq 0 (
    echo 警告: FCL 安装失败（非致命错误）
)

echo ✓ FCL 构建完成

echo.
echo ========================================
echo 步骤 3/3: 构建 MRTest (包含 FCL 测试)
echo ========================================

cd /d "%SCRIPT_DIR%"

set BUILD_DIR=build_test

REM 清理旧构建
if exist "%BUILD_DIR%" (
    echo 清理旧的构建目录...
    rmdir /s /q "%BUILD_DIR%"
)

mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"

echo 配置 MeshLib (启用 MRTest)...
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DBUILD_TESTING=ON ^
    -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON ^
    -DMR_PCH=OFF ^
    -DCMAKE_CXX_FLAGS="/bigobj /Zm200"

if !ERRORLEVEL! neq 0 (
    echo 错误: MeshLib 配置失败
    cd /d "%SCRIPT_DIR%"
    pause
    exit /b 1
)

echo ✓ MeshLib 配置成功

echo 编译 MRTest...
cmake --build . --config Release --target MRTest --parallel

if !ERRORLEVEL! neq 0 (
    echo 错误: MRTest 编译失败
    cd /d "%SCRIPT_DIR%"
    pause
    exit /b 1
)

echo ✓ MRTest 编译成功

echo.
echo ========================================
echo 构建完成！
echo ========================================
echo.
echo 构建摘要:
echo   - libccd: %SCRIPT_DIR%\thirdparty\libccd_install
echo   - FCL: %SCRIPT_DIR%\thirdparty\fcl\build
echo   - MRTest: %BUILD_DIR%\bin\Release\MRTest.exe
echo.
echo 运行 FCL 性能测试:
echo   cd %BUILD_DIR%
echo   bin\Release\MRTest.exe --gtest_filter=MRFCLPerformance.*
echo.

cd /d "%SCRIPT_DIR%"
pause
