@echo off
REM ========================================
REM MeshLib Debug 构建脚本（优化内存使用）
REM 解决 PCH 虚拟内存不足问题
REM ========================================

setlocal enabledelayedexpansion

echo ========================================
echo MeshLib Debug 优化构建
echo ========================================

REM 配置参数
set BUILD_DIR=build\viewer-debug
set PARALLEL_JOBS=2
set DISABLE_PCH=ON

echo.
echo 构建配置：
echo   - 构建目录: %BUILD_DIR%
echo   - 并行任务数: %PARALLEL_JOBS%
echo   - 禁用PCH: %DISABLE_PCH%
echo.

REM 清理旧构建（可选）
if exist "%BUILD_DIR%" (
    echo 发现现有构建目录，是否清理？
    choice /C YN /M "清理旧构建（推荐）"
    if !ERRORLEVEL! EQU 1 (
        echo 清理构建目录...
        rmdir /s /q "%BUILD_DIR%"
    )
)

REM 创建构建目录
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd "%BUILD_DIR%"

echo.
echo ========================================
echo 步骤 1/3: CMake 配置
echo ========================================

REM 优化的 CMake 配置
cmake ..\.. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DMR_PCH=OFF ^
    -DCMAKE_CXX_FLAGS="/bigobj /Zm500 /MP%PARALLEL_JOBS%" ^
    -DCMAKE_C_FLAGS="/bigobj /Zm500 /MP%PARALLEL_JOBS%"

if %ERRORLEVEL% neq 0 (
    echo 错误: CMake 配置失败
    pause
    exit /b 1
)

echo ✓ CMake 配置成功

echo.
echo ========================================
echo 步骤 2/3: 编译项目
echo ========================================

REM 方式 1: 使用 cmake --build（推荐）
cmake --build . --config Debug --parallel %PARALLEL_JOBS%

REM 如果上面失败，尝试方式 2: 直接使用 MSBuild
if %ERRORLEVEL% neq 0 (
    echo.
    echo cmake --build 失败，尝试使用 MSBuild...
    msbuild MeshLib.sln /p:Configuration=Debug /m:%PARALLEL_JOBS% /v:minimal
)

if %ERRORLEVEL% neq 0 (
    echo.
    echo ========================================
    echo 编译失败 - 故障排除建议
    echo ========================================
    echo.
    echo 可能原因：
    echo 1. 内存不足 - 关闭其他程序
    echo 2. PCH 仍然启用 - 检查 CMakeCache.txt
    echo 3. 磁盘空间不足 - 清理磁盘
    echo.
    echo 诊断命令：
    echo   type CMakeCache.txt ^| findstr MR_PCH
    echo   wmic OS get FreePhysicalMemory
    echo.
    pause
    exit /b 1
)

echo ✓ 编译成功

echo.
echo ========================================
echo 步骤 3/3: 验证构建
echo ========================================

REM 检查关键可执行文件
set ALL_OK=1

if not exist "bin\Debug\MeshViewer.exe" (
    echo ✗ MeshViewer.exe 缺失
    set ALL_OK=0
) else (
    echo ✓ MeshViewer.exe 存在
)

if not exist "bin\Debug\MRTest.exe" (
    echo ⚠ MRTest.exe 缺失（可能未启用测试）
) else (
    echo ✓ MRTest.exe 存在
)

echo.
if %ALL_OK%==1 (
    echo ========================================
    echo 构建完成！
    echo ========================================
    echo.
    echo 可执行文件位置: %BUILD_DIR%\bin\Debug\
    echo.
    echo 运行主程序:
    echo   cd %BUILD_DIR%
    echo   bin\Debug\MeshViewer.exe
    echo.
) else (
    echo ========================================
    echo 构建不完整
    echo ========================================
    echo 请检查编译日志
)

cd ..\..
pause
