import os
import sys
from pathlib import Path

Import("env")

# 修复中文路径问题
def fix_chinese_path(source, target, env):
    project_dir = Path(env["PROJECT_DIR"])
    build_dir = Path(env["BUILD_DIR"])
    
    # 确保构建目录存在
    build_dir.mkdir(parents=True, exist_ok=True)
    
    # 设置环境变量以支持UTF-8
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    
    return None

# 在构建前执行
env.AddPreAction("buildprog", fix_chinese_path)
