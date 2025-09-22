cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

if(MSVC)
  # 选择一个Windows SDK版本（建议使用已安装的最新版本）
  set(CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION "10.0.22621.0")

  # 或者更精确地控制包含路径
  set(WINDOWS_SDK_INCLUDE_DIR "C:/Program Files (x86)/Windows Kits/10/include/10.0.22621.0")
  include_directories(SYSTEM "${WINDOWS_SDK_INCLUDE_DIR}/ucrt")
  include_directories(SYSTEM "${WINDOWS_SDK_INCLUDE_DIR}/um")
  include_directories(SYSTEM "${WINDOWS_SDK_INCLUDE_DIR}/shared")
endif()


# 设置CMake策略CMP0144，允许使用大写的<PACKAGENAME>_ROOT变量
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# 项目独立的 vcpkg 配置 - 避免环境污染
if(NOT CMAKE_TOOLCHAIN_FILE)
    # 优先使用项目本地的 vcpkg 安装
    set(PROJECT_VCPKG_ROOT "${CMAKE_SOURCE_DIR}/vcpkg")
    
    if(EXISTS "${PROJECT_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
        message(STATUS "Using project local vcpkg: ${PROJECT_VCPKG_ROOT}")
        set(CMAKE_TOOLCHAIN_FILE "${PROJECT_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" 
            CACHE STRING "Vcpkg toolchain file")
    else()
        message(STATUS "Project local vcpkg not found at: ${PROJECT_VCPKG_ROOT}")
        message(STATUS "Please run: git submodule update --init --recursive vcpkg")
        message(STATUS "Or manually setup vcpkg in the project root")
        
        # 回退到全局 vcpkg（如果存在）
        find_program(VCPKG_EXECUTABLE vcpkg PATHS 
            "$ENV{VCPKG_ROOT}"
            "C:/vcpkg" 
            "D:/vcpkg"
            PATH_SUFFIXES "vcpkg"
        )
        
        if(VCPKG_EXECUTABLE)
            get_filename_component(VCPKG_ROOT "${VCPKG_EXECUTABLE}" DIRECTORY)
            message(WARNING "Falling back to global vcpkg: ${VCPKG_ROOT}")
            message(WARNING "Consider using project local vcpkg for better isolation")
            set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" 
                CACHE STRING "Vcpkg toolchain file")
        else()
            message(FATAL_ERROR "vcpkg not found. Please setup project local vcpkg or install globally.")
        endif()
    endif()
endif()

# 设置默认的 vcpkg triplet 
if(NOT VCPKG_TARGET_TRIPLET)
    if(WIN32)
        set(VCPKG_TARGET_TRIPLET "x64-windows-meshlib" CACHE STRING "vcpkg target triplet")
    elseif(APPLE)
        set(VCPKG_TARGET_TRIPLET "x64-osx" CACHE STRING "vcpkg target triplet")
    else()
        set(VCPKG_TARGET_TRIPLET "x64-linux" CACHE STRING "vcpkg target triplet") 
    endif()
endif()

# 启用 vcpkg manifest 模式（vcpkg.json）
set(VCPKG_MANIFEST_MODE ON)

# CUDA 支持（可选）
set(CUDAToolkit_ROOT "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.4/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
include(DefaultOptions)
include(ConfigureVcpkg)

# 包含手动 OpenCASCADE 配置支持
include(UseManualOpenCASCADE)

IF(NOT MR_EMSCRIPTEN AND NOT APPLE AND EXISTS /usr/bin/time)
  # print compile time of each translation unit in the CSV format (user time, system time, command)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "/usr/bin/time -f \",%U,%S,%C\" -a -o compile_timings.txt")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "/usr/bin/time -f \",%U,%S,%C\" -a -o link_timings.txt")
ENDIF()

project(MeshLib CXX)

set(MESHLIB_PROJECT_NAME "MeshLib" CACHE STRING "Project name")
add_compile_definitions(MR_PROJECT_NAME=\"${MESHLIB_PROJECT_NAME}\")

include(CompilerOptions)
IF(EMSCRIPTEN)
  include(ConfigureEmscripten)
ENDIF()

# all binaries will be located in ./build/Release/bin
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# work-around for Visual Studio generator
# see: https://stackoverflow.com/a/7750816
IF(${CMAKE_GENERATOR} MATCHES "^Visual Studio")
  foreach(BUILD_TYPE ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${BUILD_TYPE} BUILD_TYPE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
  endforeach()
ENDIF()

option(MESHLIB_PYTHON_SUPPORT "Python support" ON)
option(MESHLIB_BUILD_MRVIEWER "Build MRViewer library" ON)
option(MESHLIB_BUILD_MESHVIEWER "Build MeshViewer application" ON)
option(MESHLIB_BUILD_PYTHON_MODULES "Build Python modules" ON)
option(MESHLIB_BUILD_MESHCONV "Build meshconv utility" ON)
option(MESHLIB_EXPERIMENTAL_BUILD_C_BINDING "(experimental) Build C binding library" ON)
option(MESHLIB_BUILD_SYMBOLMESH "Build symbol-to-mesh library" ON)
##禁用了MRVoxels组件
option(MESHLIB_BUILD_VOXELS "Build voxels library" ON)
option(MESHLIB_BUILD_EXTRA_IO_FORMATS "Build extra IO format support library" ON)
option(MESHLIB_BUILD_GENERATED_C_BINDINGS "Build C bindings (assuming they are already generated)" OFF)

# 添加一个选项，允许用户完全禁用依赖于OpenVDB的功能
option(MESHLIB_USE_OPENVDB "Use OpenVDB library if available" ON)

option(MESHLIB_BUILD_MRCUDA "Build MRCuda library" ON)
###实验性功能，用于在MRCuda库中使用HIP工具包而不是CUDA。
###HIP (Heterogeneous-Computing Interface for Portability) 是AMD开发的一个C++运行时API和编程模型，允许开发者编写能够在AMD和NVIDIA GPU上运行的代码。
option(MESHLIB_EXPERIMENTAL_HIP "(experimental) Use HIP toolkit for MRCuda library" OFF)
IF(MR_EMSCRIPTEN OR APPLE)
  set(MESHLIB_BUILD_MRCUDA OFF)
ENDIF()
IF(MESHLIB_BUILD_MRCUDA)
  IF(MESHLIB_EXPERIMENTAL_HIP)
    include(ConfigureHip)
  ELSE()
    include(ConfigureCuda)
  ENDIF()
ENDIF()

IF(MR_EMSCRIPTEN)
  set(MESHLIB_PYTHON_SUPPORT OFF)
  set(MESHLIB_BUILD_PYTHON_MODULES OFF)
  set(MESHLIB_BUILD_MESHCONV OFF)
  set(MESHLIB_EXPERIMENTAL_BUILD_C_BINDING OFF)
ENDIF()

set(MESHLIB_THIRDPARTY_DIR "${PROJECT_SOURCE_DIR}/thirdparty")

set(MESHLIB_THIRDPARTY_ROOT_DIR "${PROJECT_SOURCE_DIR}" CACHE PATH "Thirdparty library location")
IF(NOT APPLE)
  # FIXME: the line below breaks building Python modules on macOS
  list(APPEND CMAKE_FIND_ROOT_PATH ${MESHLIB_THIRDPARTY_ROOT_DIR})
ENDIF()
IF(NOT EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/include" OR NOT EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/lib")
  message("thirdparty build directory not found! You can build thirdparty with ./scripts/build_thirdparty.sh")
ENDIF()
IF(EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/include")
  include_directories(${MESHLIB_THIRDPARTY_ROOT_DIR}/include)
ENDIF()
IF(EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/lib")
  link_directories(${MESHLIB_THIRDPARTY_ROOT_DIR}/lib)
ENDIF()

option(MESHLIB_KEEP_CTEST_TARGETS "Keep default CTest targets (Continuous*, Experimental*, Nightly*, etc.)" OFF)
IF(NOT MESHLIB_KEEP_CTEST_TARGETS)
  set_property(GLOBAL PROPERTY
    CTEST_TARGETS_ADDED 1
  )
ENDIF()
include(CTest)

IF(APPLE)
  include(ConfigureHomebrew)

  # need to hardcode some path to fix wheel (no option to pass path)
  # TODO: revise
  set(CMAKE_INSTALL_RPATH "@loader_path;@loader_path/..;@loader_path/../lib;@loader_path/../lib/lib;@loader_path/meshlib;${MESHLIB_THIRDPARTY_ROOT_DIR}/lib;${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

  IF(BUILD_TESTING)
    set(GTest_DIR ${MESHLIB_THIRDPARTY_ROOT_DIR}/lib/cmake/GTest)
    find_package(GTest REQUIRED CONFIG)
    include_directories(${GTEST_INCLUDE_DIRS})
  ENDIF()
ENDIF()

include(DetectPlatform)
IF(MESHLIB_PYTHON_SUPPORT AND NOT MR_EMSCRIPTEN)
  IF(NOT DEFINED MESHLIB_PYTHON_VERSION)
    # TODO: drop Python version hard-coding; set -DMESHLIB_PYTHON_VERSION explicitly in the CI config if needed
    IF(APPLE)
      set(MESHLIB_PYTHON_VERSION "3.10")
    ELSEIF(DEFINED MR_LINUX_DISTRO)
      IF(${MR_LINUX_DISTRO} STREQUAL "Ubuntu")
        set(MESHLIB_PYTHON_VERSION "3.8")
        IF(${MR_PLATFORM} STREQUAL "Ubuntu_22")
          set(MESHLIB_PYTHON_VERSION "3.10")
        ELSEIF(${MR_PLATFORM} STREQUAL "Ubuntu_24")
          set(MESHLIB_PYTHON_VERSION "3.12")
        ENDIF()
      ENDIF()
    ENDIF()

    IF($ENV{MESHLIB_PYTHON_VERSION})
      set(MESHLIB_PYTHON_VERSION $ENV{MESHLIB_PYTHON_VERSION})
    ENDIF()
  ENDIF()

  IF(DEFINED MESHLIB_PYTHON_VERSION)
    find_package(Python ${MESHLIB_PYTHON_VERSION} EXACT REQUIRED COMPONENTS Interpreter Development)
  ELSE()
    find_package(Python REQUIRED COMPONENTS Interpreter Development)
  ENDIF()

  IF(APPLE)
    # TODO: revise
    link_directories(${HOMEBREW_PREFIX}/opt/python@${MESHLIB_PYTHON_VERSION}/Frameworks/Python.framework/Versions/${MESHLIB_PYTHON_VERSION}/lib/)
    find_package(pybind11 REQUIRED)
  ENDIF()
ENDIF()

set(PROJECT_SOURCE_DIR ./source)
include_directories(${PROJECT_SOURCE_DIR})

IF(NOT MR_EMSCRIPTEN)
  IF(NOT TARGET JsonCpp::JsonCpp)
    IF(MSVC)
      find_package(JsonCpp REQUIRED)
    ELSE()
      # fix compatibility with Ubuntu 22.04
      if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
        set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
      endif()
      find_package(jsoncpp)
      IF(NOT JsonCpp_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(jsoncpp REQUIRED IMPORTED_TARGET jsoncpp)
        IF(CMAKE_VERSION VERSION_LESS 3.18)
          set_target_properties(PkgConfig::jsoncpp PROPERTIES IMPORTED_GLOBAL TRUE)
        ENDIF()
        add_library(JsonCpp::JsonCpp ALIAS PkgConfig::jsoncpp)
      ENDIF()
    ENDIF()
  ENDIF()

  find_package(fmt)
  find_package(spdlog REQUIRED)
  find_package(TBB REQUIRED)
  IF(spdlog_VERSION VERSION_GREATER_EQUAL "1.7" AND fmt_VERSION VERSION_GREATER_EQUAL "7.0")
    add_compile_definitions(SPDLOG_FMT_EXTERNAL)
  ENDIF()
  add_compile_definitions(SPDLOG_COMPILED_LIB SPDLOG_SHARED_LIB)
ENDIF() # NOT MR_EMSCRIPTEN

include_directories(${MESHLIB_THIRDPARTY_DIR}/parallel-hashmap)
include_directories(${MESHLIB_THIRDPARTY_DIR}/mrbind-pybind11/include)

IF(NOT $ENV{MR_VERSION} STREQUAL "")
  set(MESHLIB_VERSION $ENV{MR_VERSION})
ELSE()
  set(MESHLIB_VERSION 0.0.0)
ENDIF()
message("version: ${MESHLIB_VERSION}")

IF(MESHLIB_USE_VCPKG)
  # use OS-agnostic directory structure for vcpkg
  set(MR_BIN_DIR bin)
  set(MR_INCLUDE_DIR include/MeshLib)
  set(MR_MAIN_LIB_DIR lib/${MESHLIB_PROJECT_NAME})
  set(MR_PY_LIB_DIR lib/${MESHLIB_PROJECT_NAME}/meshlib)
  set(MR_CONFIG_DIR lib/cmake/MeshLib)
  set(MR_RESOURCES_DIR share/${MESHLIB_PROJECT_NAME})
  set(MR_FONTS_DIR share/${MESHLIB_PROJECT_NAME}/fonts)
  # set rpath manually for both executables and shared libraries
  # related issue: https://gitlab.kitware.com/cmake/cmake/-/issues/23727
  list(APPEND CMAKE_INSTALL_RPATH "$ORIGIN" "$ORIGIN/.." "$ORIGIN/../lib" "$ORIGIN/../lib/${MESHLIB_PROJECT_NAME}")
ELSEIF(NOT APPLE)
  include(GNUInstallDirs)
  set(MR_BIN_DIR "${CMAKE_INSTALL_BINDIR}")
  set(MR_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}/MeshLib")
  set(MR_MAIN_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/${MESHLIB_PROJECT_NAME}")
  set(MR_PY_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/${MESHLIB_PROJECT_NAME}/meshlib")
  set(MR_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/MeshLib")
  set(MR_RESOURCES_DIR "${CMAKE_INSTALL_DATADIR}/${MESHLIB_PROJECT_NAME}")
  set(MR_FONTS_DIR "${CMAKE_INSTALL_DATADIR}/${MESHLIB_PROJECT_NAME}/fonts")
ELSE()
  # TODO: correct directory structure for macOS
  # TODO: https://cmake.org/cmake/help/latest/prop_tgt/RESOURCE.html
  IF(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MR_ROOT "./Library/Frameworks/MeshLib.framework/Versions/${MESHLIB_VERSION}")
    set(MR_BIN_DIR "${MR_ROOT}/bin")
    set(MR_INCLUDE_DIR "${MR_ROOT}/include")
    set(MR_MAIN_LIB_DIR "${MR_ROOT}/lib")
    set(MR_PY_LIB_DIR "${MR_ROOT}/lib/meshlib")
    set(MR_RESOURCES_DIR "${MR_ROOT}/Resources")
    set(MR_FONTS_DIR "${MR_ROOT}/Resources/fonts")
    set(MR_CONFIG_DIR "${MR_ROOT}/Resources/cmake")
  ELSE()
    set(MESHLIB_MACOS_INSTALL_PATH "./usr/local" CACHE PATH "")
    set(MR_ROOT ${MESHLIB_MACOS_INSTALL_PATH})
    set(MR_BIN_DIR "${MR_ROOT}/bin")
    set(MR_INCLUDE_DIR "${MR_ROOT}/include/MeshLib")
    set(MR_MAIN_LIB_DIR "${MR_ROOT}/lib/MeshLib")
    set(MR_PY_LIB_DIR "${MR_ROOT}/lib/MeshLib/meshlib")
    set(MR_RESOURCES_DIR "${MR_ROOT}/etc/MeshLib")
    set(MR_FONTS_DIR "${MR_ROOT}/share/fonts")
    set(MR_CONFIG_DIR "${MR_ROOT}/lib/MeshLib")
  ENDIF()
ENDIF()

# MRMesh library is always built
add_subdirectory(${PROJECT_SOURCE_DIR}/MRPch ./MRPch)
add_subdirectory(${PROJECT_SOURCE_DIR}/MRMesh ./MRMesh)

IF(MESHLIB_BUILD_EXTRA_IO_FORMATS)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRIOExtras ./MRIOExtras)
ENDIF()

IF(MESHLIB_BUILD_GENERATED_C_BINDINGS)
  option(MESHLIB_C_BINDINGS_UNITY_BUILD "Use unity build for C bindings" ON)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MeshLibC2 ./MeshLibC2)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRTestC2 ./MRTestC2)
ENDIF()

IF(MESHLIB_BUILD_SYMBOLMESH)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRSymbolMesh ./MRSymbolMesh)
ENDIF()

IF(MESHLIB_BUILD_VOXELS)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRVoxels ./MRVoxels)
ENDIF()

IF(MESHLIB_BUILD_MRVIEWER)
  IF(NOT MR_EMSCRIPTEN)
    find_package(glfw3 CONFIG REQUIRED)
  ENDIF()

  IF(MR_EMSCRIPTEN)
    include(EmscriptenHelpers)
    mr_emscripten_pack_directory("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets" "/")
    IF(NOT MR_DISABLE_EMSCRIPTEN_ASYNCIFY)
      mr_emscripten_set_async_func_list("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/wasm_async_func_list.txt")
    ENDIF()
  ENDIF()

  add_subdirectory(${PROJECT_SOURCE_DIR}/imgui ./imgui)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRViewer ./MRViewer)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRCommonPlugins ./MRCommonPlugins)

  IF(MESHLIB_BUILD_MESHVIEWER)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MRViewerApp ./MRViewerApp)
  ENDIF()
ENDIF()

IF(NOT MR_EMSCRIPTEN)
  IF(MESHLIB_PYTHON_SUPPORT)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MRPython ./MRPython)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MREmbeddedPython ./MREmbeddedPython)

    IF(MESHLIB_BUILD_PYTHON_MODULES)
      add_subdirectory(${PROJECT_SOURCE_DIR}/mrmeshnumpy ./mrmeshnumpy)
      IF(MESHLIB_BUILD_MRVIEWER)
        add_subdirectory(${PROJECT_SOURCE_DIR}/mrviewerpy ./mrviewerpy)
      ENDIF()
    ENDIF()
  ENDIF()
ENDIF()

IF(MESHLIB_PYTHON_SUPPORT)
  set(PYBIND11_NONLIMITEDAPI_PYTHON_MIN_VERSION_HEX 0x030800f0 CACHE STRING "") # Python 3.8
  set(PYBIND11_NONLIMITEDAPI_SUFFIX "${MESHLIB_PYBIND11_LIB_SUFFIX}" CACHE STRING "")
  set(PYBIND11_NONLIMITEDAPI_LIBRARY_OUTPUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib" CACHE PATH "")
  set(PYBIND11_NONLIMITEDAPI_INSTALL_LIBDIR_STUBS "${MR_MAIN_LIB_DIR}" CACHE PATH "")
  set(PYBIND11_NONLIMITEDAPI_INSTALL_LIBDIR_SHIMS "${MR_PY_LIB_DIR}" CACHE PATH "")
  # We want to always build stubs here because we want to set `PYBIND11_COMPILER_TYPE`, `PYBIND11_BUILD_ABI` for them, and accessing them in `thirdparty/CMakeLists.txt` gets weird.
  # If not for that, we'd build them here only on Windows (because there we don't have a thirdparty build script).
  option(PYBIND11_NONLIMITEDAPI_BUILD_STUBS "" ON)
  option(PYBIND11_NONLIMITEDAPI_INSTALL_EXPORTS "" ON)
  add_subdirectory(${MESHLIB_THIRDPARTY_DIR}/mrbind-pybind11/source/non_limited_api ./meshlib)
ENDIF()

IF(MSVC)
  add_subdirectory(${PROJECT_SOURCE_DIR}/laz-perf)
  add_subdirectory(${PROJECT_SOURCE_DIR}/OpenCTM)
ENDIF()

IF(NOT MR_EMSCRIPTEN)
  IF(MESHLIB_BUILD_MESHCONV)
    add_subdirectory(${PROJECT_SOURCE_DIR}/meshconv ./meshconv)
  ENDIF()
ENDIF()

IF(NOT MR_EMSCRIPTEN AND NOT APPLE)
  IF(MESHLIB_BUILD_MRCUDA)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MRCuda ./MRCuda)
  ELSE()
    message(STATUS "MESHLIB_BUILD_MRCUDA is ${MESHLIB_BUILD_MRCUDA}")
  ENDIF()
ENDIF()

IF(MESHLIB_EXPERIMENTAL_BUILD_C_BINDING)
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRMeshC ./MRMeshC)
ENDIF()

IF(BUILD_TESTING)
  enable_testing()
  add_subdirectory(${PROJECT_SOURCE_DIR}/MRTest ./MRTest)
  IF(MESHLIB_BUILD_MRCUDA)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MRTestCuda ./MRTestCuda)
  ENDIF()
  IF(MESHLIB_EXPERIMENTAL_BUILD_C_BINDING)
    add_subdirectory(${PROJECT_SOURCE_DIR}/MRTestC ./MRTestC)
  ENDIF()
ENDIF()

include(CMakePackageConfigHelpers)
configure_package_config_file(meshlib-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/meshlib-config.cmake
  INSTALL_DESTINATION ${MR_CONFIG_DIR}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR MR_INCLUDE_DIR MR_MAIN_LIB_DIR
)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/meshlib-config-version.cmake
  VERSION ${MESHLIB_VERSION}
  COMPATIBILITY ExactVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib-config-version.cmake
  DESTINATION ${MR_CONFIG_DIR}
)

IF(EMSCRIPTEN)
  install(FILES cmake/Modules/EmscriptenHelpers.cmake DESTINATION ${MR_CONFIG_DIR})
ENDIF()

IF(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(CPACK_GENERATOR "DRAGNDROP")
  include(CPack)
ENDIF()

# vim: ts=2:sw=2
