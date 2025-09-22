@echo off
REM 测试手动安装的 OpenCASCADE 7.9.1 配置脚本
REM 用于验证 STEP 格式支持是否正常工作

echo ========================================
echo 测试手动安装的 OpenCASCADE 7.9.1 
echo ========================================

REM 设置 OpenCASCADE 路径
set OPENCASCADE_ROOT=%~dp0thirdparty\opencascade\occt_vc14-64
set OpenCASCADE_DIR=%OPENCASCADE_ROOT%\cmake
set PATH=%OPENCASCADE_ROOT%\win64\vc14\bin;%PATH%

echo OpenCASCADE 根目录: %OPENCASCADE_ROOT%
echo OpenCASCADE CMake 目录: %OpenCASCADE_DIR%

REM 检查 OpenCASCADE 安装
if not exist "%OPENCASCADE_ROOT%\cmake\OpenCASCADEConfig.cmake" (
    echo 错误: 找不到 OpenCASCADE CMake 配置文件
    echo 请确保已经解压了 OpenCASCADE 7.9.1 到 thirdparty\opencascade\ 目录
    pause
    exit /b 1
)

echo ✓ 发现 OpenCASCADE CMake 配置文件

REM 检查必需的库文件
if not exist "%OPENCASCADE_ROOT%\win64\vc14\lib\TKSTEP.lib" (
    echo 警告: 找不到 TKSTEP.lib，但可能使用 TKDESTEP.lib
)

if not exist "%OPENCASCADE_ROOT%\win64\vc14\lib\TKDESTEP.lib" (
    echo 错误: 找不到 TKDESTEP.lib - STEP 格式支持不可用
    pause
    exit /b 1
)

echo ✓ 发现 STEP 格式支持库

REM 列出关键的 OpenCASCADE 库
echo.
echo 检测到的 OpenCASCADE 库文件:
dir "%OPENCASCADE_ROOT%\win64\vc14\lib\TK*.lib" | findstr "TK"

echo.
echo ========================================
echo 配置 CMake 构建
echo ========================================

REM 创建构建目录
if not exist build_manual_occt mkdir build_manual_occt
cd build_manual_occt

REM 运行 CMake 配置
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DUSE_MANUAL_OPENCASCADE=ON ^
    -DOpenCASCADE_DIR="%OpenCASCADE_DIR%" ^
    -DMRIOEXTRAS_NO_STEP=OFF ^
    -DCMAKE_BUILD_TYPE=Release

if %ERRORLEVEL% neq 0 (
    echo 错误: CMake 配置失败
    pause
    exit /b 1
)

echo ✓ CMake 配置成功

echo.
echo ========================================  
echo 测试完成
echo ========================================
echo.
echo 现在可以使用以下命令构建项目:
echo   cd build_manual_occt
echo   cmake --build . --config Release --target MRIOExtras
echo.
echo 或者在 Visual Studio 中打开解决方案文件进行构建
echo.
pause