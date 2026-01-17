"""
科学实验室MuJoCo仿真控制模块主文件
Laboratory Simulation Control Module
"""

import mujoco
import mujoco.viewer
import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Dict, List, Tuple
import time


class DeviceState(Enum):
    """设备状态枚举"""
    OFF = 0
    ON = 1
    STANDBY = 2
    ERROR = 3


@dataclass
class LabDevice:
    """实验室设备基类"""
    name: str
    position: Tuple[float, float, float]
    is_controllable: bool = False
    state: DeviceState = DeviceState.OFF
    
    def get_info(self) -> Dict:
        """获取设备信息"""
        return {
            "name": self.name,
            "position": self.position,
            "state": self.state.name,
            "controllable": self.is_controllable
        }


class LaboratorySimulation:
    """实验室仿真主类"""
    
    def __init__(self, model_path: str):
        """
        初始化实验室仿真
        
        Args:
            model_path: MuJoCo XML模型文件路径
        """
        print(f"正在加载模型: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer: Optional[mujoco.viewer.Handle] = None
        
        # 设备注册表
        self.devices: Dict[str, LabDevice] = {}
        self._register_devices()
        
        # 执行器映射
        self.actuators: Dict[str, int] = {}
        self._map_actuators()
        
        # 关节映射
        self.joints: Dict[str, int] = {}
        self._map_joints()
        
        print(f"模型加载完成! 包含 {self.model.ngeom} 个几何体, {self.model.njnt} 个关节")
    
    def _register_devices(self):
        """注册所有可控设备"""
        # 大型离心机
        self.devices["centrifuge_large"] = LabDevice(
            name="大型分析仪/离心机",
            position=(-1.5, -1.2, 0),
            is_controllable=True
        )
        
        # 超净工作台
        self.devices["clean_bench"] = LabDevice(
            name="超净工作台",
            position=(-2.8, 1.2, 0),
            is_controllable=True
        )
        
        # 通风橱
        self.devices["fume_hood"] = LabDevice(
            name="通风橱",
            position=(2.8, -2.4, 0.9),
            is_controllable=True
        )
        
        # 显微镜
        self.devices["microscope"] = LabDevice(
            name="显微镜",
            position=(-0.3, -0.3, 0.95),
            is_controllable=True
        )
        
        # 椅子 (6把)
        for i in range(1, 7):
            self.devices[f"chair_{i}"] = LabDevice(
                name=f"橙色椅子{i}",
                position=(0, 0, 0),
                is_controllable=True
            )
        
        print(f"已注册 {len(self.devices)} 个可控设备")
    
    def _map_actuators(self):
        """映射执行器名称到索引"""
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuators[name] = i
        print(f"映射了 {len(self.actuators)} 个执行器")
    
    def _map_joints(self):
        """映射关节名称到索引"""
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joints[name] = i
        print(f"映射了 {len(self.joints)} 个关节")
    
    def start_viewer(self, camera_name: str = "isometric_main"):
        """
        启动可视化窗口
        
        Args:
            camera_name: 相机名称,默认为等距视角（可自由移动）
        """
        print("启动可视化窗口...")
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # 使用自由相机模式（可以用鼠标旋转/平移/缩放）
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        # 设置初始视角为等距视角
        self.viewer.cam.lookat[:] = [0, 0, 2]      # 观察点
        self.viewer.cam.distance = 35              # 距离
        self.viewer.cam.azimuth = 135              # 方位角（水平旋转）
        self.viewer.cam.elevation = -30            # 俯仰角（垂直旋转）
        
        print(f"使用自由相机模式（初始等距视角）")
        print("提示: 鼠标左键拖动=旋转, 右键拖动=平移, 滚轮=缩放")
    
    def step(self, n_steps: int = 1):
        """
        执行仿真步进
        
        Args:
            n_steps: 步进次数
        """
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)
        if self.viewer and self.viewer.is_running():
            self.viewer.sync()
    
    def run(self, duration: float = 60.0):
        """
        运行仿真主循环
        
        Args:
            duration: 仿真时长(秒),默认60秒
        """
        self.start_viewer()
        start_time = time.time()
        step_count = 0
        
        print(f"开始仿真,预计运行 {duration} 秒...")
        print("提示: 关闭可视化窗口或等待时间结束可停止仿真")
        
        while self.viewer.is_running() and (time.time() - start_time) < duration:
            self.step()
            step_count += 1
            
            # 更新设备状态
            self._update_device_states()
            
            # 每秒打印一次统计
            if step_count % 500 == 0:
                elapsed = time.time() - start_time
                print(f"已运行: {elapsed:.1f}秒, 步数: {step_count}")
            
            time.sleep(self.model.opt.timestep)
        
        elapsed = time.time() - start_time
        print(f"\n仿真结束! 总时长: {elapsed:.2f}秒, 总步数: {step_count}")
        self.viewer.close()
    
    def _update_device_states(self):
        """更新所有设备状态"""
        for device_name, device in self.devices.items():
            if hasattr(device, 'update'):
                device.update(self.data)
    
    def control_device(self, device_name: str, command: str, **kwargs):
        """
        控制指定设备
        
        Args:
            device_name: 设备名称
            command: 控制命令
            **kwargs: 命令参数
        """
        if device_name not in self.devices:
            raise ValueError(f"未知设备: {device_name}")
        
        device = self.devices[device_name]
        if not device.is_controllable:
            raise ValueError(f"设备 {device_name} 不可控制")
        
        print(f"控制设备: {device.name} - 命令: {command}")
        
        # 根据设备类型执行相应命令
        if device_name == "centrifuge_large":
            self._control_centrifuge(command, **kwargs)
        elif device_name == "fume_hood":
            self._control_fume_hood(command, **kwargs)
        elif device_name == "clean_bench":
            self._control_clean_bench(command, **kwargs)
        elif device_name == "microscope":
            self._control_microscope(command, **kwargs)
        elif device_name.startswith("chair_"):
            self._control_chair(device_name, command, **kwargs)
        else:
            print(f"设备 {device_name} 暂无控制逻辑实现")
    
    def _control_centrifuge(self, command: str, **kwargs):
        """控制离心机"""
        if command == "open_lid":
            if "e01_lid_hinge" in self.joints:
                joint_id = self.joints["e01_lid_hinge"]
                self.data.qpos[joint_id] = 1.4  # 打开到80度
                print("离心机盖子已打开")
        elif command == "close_lid":
            if "e01_lid_hinge" in self.joints:
                joint_id = self.joints["e01_lid_hinge"]
                self.data.qpos[joint_id] = 0.0  # 关闭
                print("离心机盖子已关闭")
    
    def _control_fume_hood(self, command: str, **kwargs):
        """控制通风橱"""
        if command == "set_sash_height":
            height = kwargs.get("height", 0.5)  # 0-1
            if "e06_sash_slide" in self.joints:
                joint_id = self.joints["e06_sash_slide"]
                self.data.qpos[joint_id] = height * 0.4  # 最大0.4m
                print(f"通风橱玻璃挡板高度设置为: {height*100:.0f}%")
    
    def _control_clean_bench(self, command: str, **kwargs):
        """控制超净工作台"""
        if command == "open_door":
            height = kwargs.get("height", 0.15)  # 0-0.3m
            if "e03_door_slide" in self.joints:
                joint_id = self.joints["e03_door_slide"]
                self.data.qpos[joint_id] = height
                print(f"超净台玻璃门打开高度: {height*100:.0f}cm")
    
    def _control_microscope(self, command: str, **kwargs):
        """控制显微镜"""
        if command == "adjust_focus":
            delta = kwargs.get("delta", 0.01)
            if "e04_focus" in self.joints:
                joint_id = self.joints["e04_focus"]
                current = self.data.qpos[joint_id]
                new_pos = np.clip(current + delta, -0.05, 0.05)
                self.data.qpos[joint_id] = new_pos
                print(f"显微镜焦距调整: {new_pos:.3f}m")
    
    def _control_chair(self, chair_name: str, command: str, **kwargs):
        """控制椅子"""
        chair_num = chair_name.split("_")[1]
        
        if command == "adjust_height":
            delta = kwargs.get("delta", 0.05)
            joint_name = f"ch{chair_num.zfill(2)}_height"
            if joint_name in self.joints:
                joint_id = self.joints[joint_name]
                current = self.data.qpos[joint_id]
                new_height = np.clip(current + delta, 0, 0.15)
                self.data.qpos[joint_id] = new_height
                print(f"{chair_name} 高度调整至: {new_height:.3f}m")
        
        elif command == "rotate":
            angle = kwargs.get("angle", 0.5)  # 弧度
            joint_name = f"ch{chair_num.zfill(2)}_rotate"
            if joint_name in self.joints:
                joint_id = self.joints[joint_name]
                current = self.data.qpos[joint_id]
                self.data.qpos[joint_id] = current + angle
                print(f"{chair_name} 旋转 {np.degrees(angle):.1f}度")
    
    def get_device_info(self, device_name: str) -> Dict:
        """
        获取设备信息
        
        Args:
            device_name: 设备名称
            
        Returns:
            设备信息字典
        """
        if device_name not in self.devices:
            raise ValueError(f"未知设备: {device_name}")
        return self.devices[device_name].get_info()
    
    def list_devices(self) -> List[str]:
        """列出所有设备"""
        return list(self.devices.keys())
    
    def print_scene_info(self):
        """打印场景信息"""
        print("\n" + "="*60)
        print("MuJoCo科学实验室场景信息")
        print("="*60)
        print(f"几何体数量: {self.model.ngeom}")
        print(f"物体数量: {self.model.nbody}")
        print(f"关节数量: {self.model.njnt}")
        print(f"执行器数量: {self.model.nu}")
        print(f"相机数量: {self.model.ncam}")
        print(f"光源数量: {self.model.nlight}")
        print(f"\n可控设备数量: {len(self.devices)}")
        print("设备列表:")
        for name, device in self.devices.items():
            status = "可控" if device.is_controllable else "不可控"
            print(f"  - {name}: {device.name} ({status})")
        print("="*60 + "\n")


def main():
    """主函数示例"""
    import os
    
    # 获取模型文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "..", "models", "laboratory.xml")
    
    # 初始化仿真
    sim = LaboratorySimulation(model_path)
    
    # 打印场景信息
    sim.print_scene_info()
    
    # 演示设备控制
    print("\n演示设备控制:")
    print("-" * 60)
    
    # 打开离心机盖子
    try:
        sim.control_device("centrifuge_large", "open_lid")
    except Exception as e:
        print(f"控制失败: {e}")
    
    # 调节通风橱
    try:
        sim.control_device("fume_hood", "set_sash_height", height=0.5)
    except Exception as e:
        print(f"控制失败: {e}")
    
    # 调节椅子
    try:
        sim.control_device("chair_01", "adjust_height", delta=0.08)
        sim.control_device("chair_01", "rotate", angle=1.57)  # 旋转90度
    except Exception as e:
        print(f"控制失败: {e}")
    
    print("-" * 60)
    
    # 运行仿真
    print("\n准备启动可视化...")
    print("提示:")
    print("  - 鼠标左键拖动: 旋转视角")
    print("  - 鼠标右键拖动: 平移视角")  
    print("  - 滚轮: 缩放")
    print("  - 关闭窗口或等待60秒结束仿真\n")
    
    sim.run(duration=60.0)


if __name__ == "__main__":
    main()
