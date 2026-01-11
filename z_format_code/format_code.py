#!/usr/bin/env python3
"""
软著代码格式化脚本
自动将源代码文件处理成每页50行、带页眉的格式。
"""

import os
import sys

def format_code_for_copyright(source_file_path, output_file_path, software_name, lines_per_page=50):
    """
    核心格式化函数
    :param source_file_path: 原始源代码文件路径
    :param output_file_path: 输出文件路径
    :param software_name: 软件名称（用于页眉）
    :param lines_per_page: 每页行数，默认50
    """
    try:
        with open(source_file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"错误：找不到源文件 {source_file_path}")
        return
    except Exception as e:
        print(f"读取文件时出错：{e}")
        return

    # 计算需要多少页
    total_lines = len(lines)
    total_pages = (total_lines + lines_per_page - 1) // lines_per_page  # 向上取整

    print(f"正在处理文件：{source_file_path}")
    print(f"代码总行数：{total_lines}，将生成约 {total_pages} 页。")

    with open(output_file_path, 'w', encoding='utf-8') as out_f:
        current_page = 1
        line_count_in_page = 0

        for i, line in enumerate(lines):
            # 每页开始时，添加页眉
            if line_count_in_page == 0:
                # 页眉格式示例，可根据官方要求调整
                header = f"/{'='*70}\\\n"
                header += f"| 《{software_name}》 核心源代码 - 第 {current_page:03d} 页 |\n"
                header += f"\\{'='*70}/\n\n"
                out_f.write(header)

            # 写入代码行（保留原格式）
            out_f.write(line)
            line_count_in_page += 1

            # 判断是否满一页或文件结束
            if line_count_in_page >= lines_per_page or i == total_lines - 1:
                # 用空行补足一页（如果需要）
                while line_count_in_page < lines_per_page:
                    out_f.write("\n")
                    line_count_in_page += 1

                # 页尾分隔符
                out_f.write(f"\n// {'-'*40} 页 {current_page:03d} 结束 {'-'*40}\n\n")
                current_page += 1
                line_count_in_page = 0

    print(f"处理完成！格式化后的代码已保存至：{output_file_path}")

if __name__ == "__main__":
    # ============ 在这里修改参数 ============
    # 1. 您的软件名称（必须与申请表一致）
    SOFTWARE_NAME = "智巡·速办——无人机无人车协同巡查处置系统软件 V1.0"
    
    # 2. 需要处理的原始代码文件列表（可以是您精简后的“核心版”）
    SOURCE_FILES = [
        "./src/z_format_code/src/red_cube_capture_node_xml.cpp",  # 您简化后的视觉节点
    ]
    
    # 3. 输出文件路径
    OUTPUT_FILE = "./src/z_format_code/soft_copyright_source_code.txt"
    # =======================================

    # 清空或创建输出文件
    open(OUTPUT_FILE, 'w', encoding='utf-8').close()
    
    # 逐个处理文件并追加到同一个输出文件中
    for idx, src_file in enumerate(SOURCE_FILES):
        if os.path.exists(src_file):
            # 临时文件用于存储单个文件的格式化结果
            temp_output = f"./temp_page_{idx}.txt"
            format_code_for_copyright(src_file, temp_output, SOFTWARE_NAME)
            
            # 将临时文件内容追加到总输出文件
            with open(temp_output, 'r', encoding='utf-8') as temp_f, \
                 open(OUTPUT_FILE, 'a', encoding='utf-8') as final_f:
                final_f.write(temp_f.read())
                final_f.write("\n" + "="*80 + "\n\n")  # 文件间分隔符
            
            # 删除临时文件
            os.remove(temp_output)
        else:
            print(f"警告：跳过不存在的文件 {src_file}")
    
    print(f"\n所有文件处理完成！最终合并的代码文件位于：{OUTPUT_FILE}")
    print("请使用文本编辑器打开检查，确保格式正确。")