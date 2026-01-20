"""
测试MuJoCo模型加载
"""

import sys
import os

# 添加项目路径
project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_dir)

try:
    import mujoco
    print("[OK] MuJoCo已安装")
    print(f"  版本: {mujoco.__version__}")
except ImportError:
    print("[ERROR] MuJoCo未安装")
    print("  请运行: pip install mujoco>=3.0.0")
    sys.exit(1)

try:
    import numpy as np
    print("[OK] NumPy已安装")
except ImportError:
    print("[ERROR] NumPy未安装")
    print("  请运行: pip install numpy>=1.24.0")
    sys.exit(1)

# 测试加载模型
print("\n测试加载模型文件...")
model_path = os.path.join(project_dir, "models", "laboratory.xml")

if not os.path.exists(model_path):
    print(f"[ERROR] 模型文件不存在: {model_path}")
    sys.exit(1)

print(f"[OK] 模型文件存在: {model_path}")

try:
    print("\n正在加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    print("[OK] 模型加载成功!")
    
    # 打印模型信息
    print("\n" + "="*60)
    print("模型信息:")
    print("="*60)
    print(f"几何体数量: {model.ngeom}")
    print(f"物体数量: {model.nbody}")
    print(f"关节数量: {model.njnt}")
    print(f"执行器数量: {model.nu}")
    print(f"相机数量: {model.ncam}")
    print(f"光源数量: {model.nlight}")
    print(f"材质数量: {model.nmat}")
    print(f"纹理数量: {model.ntex}")
    
    # 列出相机
    print("\n相机列表:")
    for i in range(model.ncam):
        cam_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        if cam_name:
            print(f"  - {cam_name}")
    
    # 列出部分关节
    print("\n关节列表 (前10个):")
    for i in range(min(10, model.njnt)):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if jnt_name:
            print(f"  - {jnt_name}")
    if model.njnt > 10:
        print(f"  ... 还有 {model.njnt-10} 个关节")

    for i in range(model.nmat):
        mat_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, i)
        if mat_name:
            print(f"  - {mat_name}")
    
    # 列出材质
    print("\n材质列表:")
    for i in range(model.nmat):
        mat_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, i)
        if mat_name:
            print(f"  - {mat_name}")
    
    print("\n" + "="*60)
    print("[OK] 所有测试通过!")
    print("="*60)
    print("\n可以运行主程序:")
    print("  python controllers/simulation.py")
    
except Exception as e:
    print(f"[ERROR] 模型加载失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

