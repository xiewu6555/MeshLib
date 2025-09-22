@echo off
REM 使用手动安装的 OpenCASCADE 构建 MeshLib 项目

echo ========================================
echo 构建 MeshLib 项目 (使用手动 OpenCASCADE)
echo ========================================

REM 设置 OpenCASCADE 环境变量
set OPENCASCADE_ROOT=%~dp0thirdparty\opencascade\occt_vc14-64
set OpenCASCADE_DIR=%OPENCASCADE_ROOT%\cmake
set PATH=%OPENCASCADE_ROOT%\win64\vc14\bin;%PATH%

echo OpenCASCADE 根目录: %OPENCASCADE_ROOT%
echo OpenCASCADE CMake 目录: %OpenCASCADE_DIR%

REM 检查 OpenCASCADE 是否存在
if not exist "%OpenCASCADE_DIR%\OpenCASCADEConfig.cmake" (
    echo 错误: 找不到 OpenCASCADE CMake 配置文件
    echo 请确保已解压 OpenCASCADE 7.9.1 到 thirdparty\opencascade\ 目录
    pause
    exit /b 1
)

echo ✓ 发现 OpenCASCADE 配置文件

REM 清理并创建构建目录
if exist build_occt rmdir /s /q build_occt
mkdir build_occt
cd build_occt

echo.
echo ========================================
echo 运行 CMake 配置
echo ========================================

REM 配置项目
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DOpenCASCADE_DIR="%OpenCASCADE_DIR%" ^
    -DCMAKE_PREFIX_PATH="%OPENCASCADE_ROOT%" ^
    -DMRIOEXTRAS_NO_STEP=OFF ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON

if %ERRORLEVEL% neq 0 (
    echo 错误: CMake 配置失败，错误代码 %ERRORLEVEL%
    pause
    exit /b 1
)

echo ✓ CMake 配置成功

echo.
echo ========================================
echo 开始编译项目
echo ========================================

REM 编译项目（只编译核心库以减少编译时间）
cmake --build . --config Release --target MRIOExtras --parallel

if %ERRORLEVEL% neq 0 (
    echo 错误: 编译失败，错误代码 %ERRORLEVEL%
    pause
    exit /b 1
)

echo ✓ 编译成功

echo.
echo ========================================
echo 构建完成
echo ========================================
echo.
echo MRIOExtras 库已成功构建，现在支持 STEP 格式！
echo.
echo 构建输出位置: 
echo   %cd%\bin\Release\
echo.
echo 可以使用以下命令构建完整项目:
echo   cmake --build . --config Release
echo.
pause