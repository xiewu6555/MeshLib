@echo off
REM FCL 性能测试构建脚本

echo ========================================
echo 构建并测试 FCL 性能对比
echo ========================================

REM 设置构建目录
set BUILD_DIR=build_test

REM 清理旧构建（自动）
if exist %BUILD_DIR% (
    echo 发现现有构建目录 %BUILD_DIR%
    echo 清理构建目录...
    rmdir /s /q %BUILD_DIR%
)

REM 创建构建目录
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
cd %BUILD_DIR%

echo.
echo ========================================
echo 步骤 1/3: CMake 配置
echo ========================================

cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DBUILD_TESTING=ON ^
    -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON ^
    -DMR_PCH=OFF ^
    -DCMAKE_CXX_FLAGS="/bigobj /Zm200"

if %ERRORLEVEL% neq 0 (
    echo 错误: CMake 配置失败
    pause
    exit /b 1
)

echo ✓ CMake 配置成功

echo.
echo ========================================
echo 步骤 2/3: 编译 MRTest
echo ========================================

cmake --build . --config Release --target MRTest --parallel

if %ERRORLEVEL% neq 0 (
    echo 错误: 编译失败
    pause
    exit /b 1
)

echo ✓ 编译成功

echo.
echo ========================================
echo 步骤 3/3: 运行性能测试
echo ========================================

REM 运行 FCL 性能测试
bin\Release\MRTest.exe --gtest_filter=MRFCLPerformance.*

if %ERRORLEVEL% neq 0 (
    echo 警告: 测试执行出现问题
    echo 可能的原因:
    echo 1. 缺少 FCL 的运行时依赖
    echo 2. nanobench 库未正确链接
    echo 3. 测试代码存在问题
    pause
    exit /b 1
)

echo.
echo ========================================
echo 测试完成！
echo ========================================
echo.
echo 测试程序位置: %BUILD_DIR%\bin\Release\MRTest.exe
echo.
pause