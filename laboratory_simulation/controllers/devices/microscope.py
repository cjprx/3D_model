"""
显微镜控制类
"""

import numpy as np3413443
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Microscope:
    """显微镜控制类"""
    
    name: str
    position: tuple
    magnification_levels: List[int] = None
    current_magnification_idx: int = 0
    focus_position: float = 0.0  # -0.05 to 0.05
    light_on: bool = False
    focus_joint_id: Optional[int] = None
    
    def __post_init__(self):
        if self.magnification_levels is None:
            self.magnification_levels = [4, 10, 40, 100]
    
    @property
    def current_magnification(self) -> int:
        """当前放大倍数"""
        return self.magnification_levels[self.current_magnification_idx]
    
    def set_magnification(self, level: int):
        """设置放大倍数"""
        if level in self.magnification_levels:
            self.current_magnification_idx = self.magnification_levels.index(level)
            print(f"显微镜放大倍数设置为: {level}x")
        else:
            print(f"警告: 不支持的放大倍数 {level}x")
    
    def adjust_focus(self, delta: float):
        """调节焦距"""
        self.focus_position = np.clip(self.focus_position + delta, -0.05, 0.05)
        print(f"显微镜焦距调整至: {self.focus_position:.4f}m")
    
    def toggle_light(self):
        """切换光源"""
        self.light_on = not self.light_on
        status = "开启" if self.light_on else "关闭"
        print(f"显微镜光源{status}")
    
    def update(self, data):
        """更新显微镜状态"""
        if self.focus_joint_id is not None and data is not None:
            if hasattr(data, 'qpos'):
                data.qpos[self.focus_joint_id] = self.focus_position

