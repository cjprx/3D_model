"""
通风橱控制类
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class FumeHood:
    """通风橱控制类"""
    
    name: str
    position: tuple
    sash_height: float = 0.0  # 0-1, 0为关闭, 1为全开
    fan_speed: int = 0  # 0-3档
    light_on: bool = False
    sash_joint_id: Optional[int] = None
    
    def set_sash_height(self, height: float):
        """设置玻璃挡板高度"""
        self.sash_height = np.clip(height, 0, 1)
        print(f"通风橱玻璃挡板高度设置为: {self.sash_height*100:.0f}%")
    
    def set_fan_speed(self, speed: int):
        """设置风扇档位"""
        self.fan_speed = np.clip(speed, 0, 3)
        if self.fan_speed == 0:
            print("通风橱风扇已关闭")
        else:
            print(f"通风橱风扇档位设置为: {self.fan_speed}档")
    
    def toggle_light(self):
        """切换照明"""
        self.light_on = not self.light_on
        status = "开启" if self.light_on else "关闭"
        print(f"通风橱照明{status}")
    
    def emergency_close(self):
        """紧急关闭"""
        self.sash_height = 0.0
        self.fan_speed = 0
        print("通风橱紧急关闭!")
    
    def update(self, data):
        """更新通风橱状态"""
        if self.sash_joint_id is not None and data is not None:
            # 将高度映射到关节位置
            if hasattr(data, 'qpos'):
                data.qpos[self.sash_joint_id] = self.sash_height * 0.6

