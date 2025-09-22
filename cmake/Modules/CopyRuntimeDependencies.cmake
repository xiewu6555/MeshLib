# CopyRuntimeDependencies.cmake
#
# 现代 CMake 实践：自动拷贝运行时依赖 DLL 文件
# 适用于 Windows 环境下的 MeshLib 项目
#
# 版本：v1.0
# 日期：2025-09-12

# 获取 vcpkg 安装的 DLL 文件的函数
function(copy_vcpkg_dlls target_name)
    if(NOT WIN32)
        return()
    endif()
    
    # 从 CMAKE_TOOLCHAIN_FILE 推断 vcpkg 根目录
    if(CMAKE_TOOLCHAIN_FILE)
        get_filename_component(VCPKG_ROOT "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
        get_filename_component(VCPKG_ROOT "${VCPKG_ROOT}" DIRECTORY)
        get_filename_component(VCPKG_ROOT "${VCPKG_ROOT}" DIRECTORY)
    endif()
    
    if(VCPKG_ROOT AND EXISTS "${VCPKG_ROOT}")
        if(VCPKG_TARGET_TRIPLET)
            set(VCPKG_BIN_DIR "${VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/bin")
        else()
            set(VCPKG_BIN_DIR "${VCPKG_ROOT}/installed/x64-windows-meshlib/bin")
        endif()
        
        message(STATUS "vcpkg bin目录: ${VCPKG_BIN_DIR}")
        
        if(EXISTS "${VCPKG_BIN_DIR}")
            # 搜索所有 vcpkg DLL 文件
            file(GLOB VCPKG_DLLS "${VCPKG_BIN_DIR}/*.dll")
            
            if(VCPKG_DLLS)
                list(LENGTH VCPKG_DLLS DLL_COUNT)
                message(STATUS "找到 ${DLL_COUNT} 个 vcpkg DLL 文件")
                
                # 创建自定义命令来拷贝 vcpkg DLL 文件
                add_custom_command(TARGET ${target_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E echo "正在拷贝 vcpkg DLL 文件到 $<TARGET_FILE_DIR:${target_name}>"
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${VCPKG_DLLS}
                        $<TARGET_FILE_DIR:${target_name}>
                    COMMENT "拷贝 vcpkg 运行时依赖 DLL 文件"
                    VERBATIM
                )
            else()
                message(WARNING "未在 ${VCPKG_BIN_DIR} 找到 DLL 文件")
            endif()
        else()
            message(WARNING "vcpkg bin 目录不存在: ${VCPKG_BIN_DIR}")
        endif()
    else()
        message(WARNING "无法确定 vcpkg 根目录")
    endif()
endfunction()

# 拷贝手动安装的 OpenCASCADE DLL 文件
function(copy_manual_opencascade_dlls target_name)
    if(NOT WIN32)
        return()
    endif()
    
    # 检查是否启用了手动 OpenCASCADE
    if(USE_MANUAL_OPENCASCADE OR DEFINED OpenCASCADE_BINARY_DIR)
        if(NOT DEFINED OpenCASCADE_BINARY_DIR)
            set(OpenCASCADE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencascade/occt_vc14-64/win64/vc14/bin")
        endif()
        
        message(STATUS "OpenCASCADE DLL目录: ${OpenCASCADE_BINARY_DIR}")
        
        if(EXISTS "${OpenCASCADE_BINARY_DIR}")
            # 搜索所有 OpenCASCADE DLL 文件
            file(GLOB OPENCASCADE_DLLS "${OpenCASCADE_BINARY_DIR}/*.dll")
            
            if(OPENCASCADE_DLLS)
                list(LENGTH OPENCASCADE_DLLS DLL_COUNT)
                message(STATUS "找到 ${DLL_COUNT} 个 OpenCASCADE DLL 文件")
                
                # 创建自定义命令来拷贝 OpenCASCADE DLL 文件
                add_custom_command(TARGET ${target_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E echo "正在拷贝 OpenCASCADE DLL 文件到 $<TARGET_FILE_DIR:${target_name}>"
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${OPENCASCADE_DLLS}
                        $<TARGET_FILE_DIR:${target_name}>
                    COMMENT "拷贝 OpenCASCADE 运行时依赖 DLL 文件"
                    VERBATIM
                )
                
                # 同样拷贝第三方依赖 DLL（如果存在）
                set(OPENCASCADE_3RDPARTY_BIN "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencascade/3rdparty-vc14-64/bin")
                if(EXISTS "${OPENCASCADE_3RDPARTY_BIN}")
                    file(GLOB OPENCASCADE_3RDPARTY_DLLS "${OPENCASCADE_3RDPARTY_BIN}/*.dll")
                    if(OPENCASCADE_3RDPARTY_DLLS)
                        list(LENGTH OPENCASCADE_3RDPARTY_DLLS DLL_3RD_COUNT)
                        message(STATUS "找到 ${DLL_3RD_COUNT} 个 OpenCASCADE 第三方 DLL 文件")
                        
                        add_custom_command(TARGET ${target_name} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                ${OPENCASCADE_3RDPARTY_DLLS}
                                $<TARGET_FILE_DIR:${target_name}>
                            COMMENT "拷贝 OpenCASCADE 第三方 DLL 文件"
                            VERBATIM
                        )
                    endif()
                endif()
            else()
                message(WARNING "未在 ${OpenCASCADE_BINARY_DIR} 找到 OpenCASCADE DLL 文件")
            endif()
        else()
            message(WARNING "OpenCASCADE 二进制目录不存在: ${OpenCASCADE_BINARY_DIR}")
        endif()
    else()
        message(STATUS "未启用手动 OpenCASCADE，跳过拷贝")
    endif()
endfunction()

# 拷贝系统运行时 DLL 文件（如果需要）
function(copy_system_runtime_dlls target_name)
    if(NOT WIN32)
        return()
    endif()
    
    # Visual C++ 运行时通常由系统提供，但某些情况下可能需要手动部署
    # 这里预留接口，可以根据需要添加特定的运行时 DLL
    
    message(STATUS "系统运行时 DLL 通常由 Visual C++ Redistributable 提供")
endfunction()

# 主函数：拷贝所有运行时依赖
function(copy_runtime_dependencies target_name)
    if(NOT WIN32)
        message(STATUS "非 Windows 系统，跳过 DLL 拷贝")
        return()
    endif()
    
    message(STATUS "配置 ${target_name} 的运行时依赖拷贝")
    
    # 拷贝 vcpkg DLL
    copy_vcpkg_dlls(${target_name})
    
    # 拷贝手动 OpenCASCADE DLL
    copy_manual_opencascade_dlls(${target_name})
    
    # 拷贝系统运行时 DLL（如果需要）
    copy_system_runtime_dlls(${target_name})
    
    message(STATUS "${target_name} 的运行时依赖配置完成")
endfunction()

# 安装时拷贝 DLL 的辅助函数
function(install_runtime_dependencies target_name destination)
    if(NOT WIN32)
        return()
    endif()
    
    # 这个函数用于在安装时拷贝 DLL 文件
    # 当前我们主要关注构建后的拷贝，所以这里先留空
    message(STATUS "安装时的 DLL 拷贝配置（待实现）")
endfunction()