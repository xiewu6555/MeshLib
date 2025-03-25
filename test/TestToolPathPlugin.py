#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试刀具路径生成插件功能的脚本
"""

import os
import sys
import subprocess
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description='测试刀具路径生成插件')
    parser.add_argument('--viewer_path', type=str, default='../bin/MRViewer',
                        help='MRViewer可执行文件路径')
    parser.add_argument('--model_path', type=str, default='../model/VRnew.STEP',
                        help='测试STEP模型路径')
    args = parser.parse_args()
    
    # 检查MRViewer路径
    viewer_path = os.path.abspath(args.viewer_path)
    if not os.path.exists(viewer_path):
        print(f"错误: 找不到MRViewer可执行文件: {viewer_path}")
        return 1
    
    # 检查模型路径
    model_path = os.path.abspath(args.model_path)
    if not os.path.exists(model_path):
        print(f"错误: 找不到模型文件: {model_path}")
        return 1
    
    print(f"启动MRViewer: {viewer_path}")
    print(f"测试模型: {model_path}")
    
    # 启动MRViewer
    try:
        process = subprocess.Popen([viewer_path, model_path])
        
        print("MRViewer已启动，请进行如下测试:")
        print("1. 点击"工具 -> 刀具路径生成器"打开插件面板")
        print("2. 在"模型选择"面板中选择已加载的模型")
        print("3. 调整"刀具参数"和"算法参数"")
        print("4. 点击"生成当前选择算法路径"生成刀具路径")
        print("5. 测试动画控制、多视图和路径分析功能")
        print("6. 测试导出功能，尝试保存为G代码或其他格式")
        print("\n按Ctrl+C退出测试...")
        
        # 等待用户手动退出
        process.wait()
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
        if process:
            process.terminate()
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        if process:
            process.terminate()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 