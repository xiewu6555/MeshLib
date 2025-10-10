@echo off
REM ========================================
REM libccd 编译和安装脚本
REM 用途: 为 FCL 构建 libccd 静态库
REM 输出: thirdparty/libccd_install/
REM ========================================

setlocal enabledelayedexpansion

echo ========================================
echo 开始构建 libccd 库
echo ========================================

REM 获取脚本所在目录（项目根目录）
set SCRIPT_DIR=%~dp0
set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%

REM 设置路径
set LIBCCD_SOURCE=%SCRIPT_DIR%\thirdparty\libccd
set LIBCCD_BUILD=%SCRIPT_DIR%\thirdparty\libccd\build
set LIBCCD_INSTALL=%SCRIPT_DIR%\thirdparty\libccd_install

REM 检查源码目录
if not exist "%LIBCCD_SOURCE%\CMakeLists.txt" (
    echo 错误: 找不到 libccd 源码目录: %LIBCCD_SOURCE%
    echo 请确保 git submodule 已正确初始化
    pause
    exit /b 1
)

echo.
echo 配置信息:
echo   源码目录: %LIBCCD_SOURCE%
echo   构建目录: %LIBCCD_BUILD%
echo   安装目录: %LIBCCD_INSTALL%
echo.

REM 清理旧的构建目录
if exist "%LIBCCD_BUILD%" (
    echo 清理旧的构建目录...
    rmdir /s /q "%LIBCCD_BUILD%"
)

REM 创建构建目录
mkdir "%LIBCCD_BUILD%"
cd /d "%LIBCCD_BUILD%"

echo ========================================
echo 步骤 1/4: CMake 配置 (Debug)
echo ========================================

cmake .. ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_INSTALL_PREFIX="%LIBCCD_INSTALL%" ^
    -DBUILD_SHARED_LIBS=OFF ^
    -DENABLE_DOUBLE_PRECISION=ON ^
    -DCCD_HIDE_ALL_SYMBOLS=OFF ^
    -DBUILD_TESTING=OFF

if %ERRORLEVEL% neq 0 (
    echo 错误: CMake 配置失败
    pause
    exit /b 1
)

echo ✓ CMake 配置成功

echo.
echo ========================================
echo 步骤 2/4: 编译 (Debug)
echo ========================================

cmake --build . --config Debug --parallel

if %ERRORLEVEL% neq 0 (
    echo 错误: Debug 编译失败
    pause
    exit /b 1
)

echo ✓ Debug 编译成功

echo.
echo ========================================
echo 步骤 3/4: 编译 (Release)
echo ========================================

cmake --build . --config Release --parallel

if %ERRORLEVEL% neq 0 (
    echo 错误: Release 编译失败
    pause
    exit /b 1
)

echo ✓ Release 编译成功

echo.
echo ========================================
echo 步骤 4/4: 安装库文件
echo ========================================

REM 安装 Debug 版本
cmake --install . --config Debug
if %ERRORLEVEL% neq 0 (
    echo 警告: Debug 安装失败
)

REM 安装 Release 版本
cmake --install . --config Release
if %ERRORLEVEL% neq 0 (
    echo 错误: Release 安装失败
    pause
    exit /b 1
)

echo ✓ 安装成功

echo.
echo ========================================
echo 验证安装结果
echo ========================================

REM 检查关键文件
set ALL_OK=1

if not exist "%LIBCCD_INSTALL%\include\ccd\ccd.h" (
    echo ✗ 头文件缺失: ccd/ccd.h
    set ALL_OK=0
) else (
    echo ✓ 头文件存在: ccd/ccd.h
)

if not exist "%LIBCCD_INSTALL%\lib\ccd.lib" (
    echo ✗ 库文件缺失: ccd.lib
    set ALL_OK=0
) else (
    echo ✓ 库文件存在: ccd.lib
)

if not exist "%LIBCCD_INSTALL%\lib\ccd\ccd-config.cmake" (
    echo ✗ CMake 配置文件缺失: ccd-config.cmake
    set ALL_OK=0
) else (
    echo ✓ CMake 配置文件存在: ccd-config.cmake
)

echo.
if %ALL_OK%==1 (
    echo ========================================
    echo libccd 构建完成！
    echo ========================================
    echo.
    echo 安装位置: %LIBCCD_INSTALL%
    echo.
    echo 下一步可以运行:
    echo   build_and_test_fcl.bat    - 构建并测试 FCL
    echo.
) else (
    echo ========================================
    echo 构建完成但缺少部分文件
    echo ========================================
    echo 请检查构建日志
    pause
    exit /b 1
)

cd /d "%SCRIPT_DIR%"
pause
