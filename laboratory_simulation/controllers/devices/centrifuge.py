"""
离心机控制类
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class Centrifuge:
    """离心机控制类"""
    
    name: str
    position: tuple
    max_rpm: int = 15000
    current_rpm: int = 0
    target_rpm: int = 0
    lid_open: bool = False
    rotor_joint_id: Optional[int] = None
    lid_joint_id: Optional[int] = None
    
    def set_speed(self, rpm: int):
        """设置转速"""
        if self.lid_open:
            print("警告: 盖子未关闭,无法启动离心机")
            return
        self.target_rpm = min(rpm, self.max_rpm)
        print(f"离心机目标转速设置为: {self.target_rpm} RPM")
    
    def open_lid(self):
        """打开盖子"""
        if self.current_rpm > 0:
            print("警告: 离心机运行中,无法打开盖子")
            return
        self.lid_open = True
        print("离心机盖子已打开")
    
    def close_lid(self):
        """关闭盖子"""
        self.lid_open = False
        print("离心机盖子已关闭")
    
    def emergency_stop(self):
        """紧急停止"""
        self.target_rpm = 0
        print("离心机紧急停止!")
    
    def update(self, data):
        """更新离心机状态"""
        # 平滑加速/减速
        if self.current_rpm < self.target_rpm:
            self.current_rpm = min(self.current_rpm + 100, self.target_rpm)
        elif self.current_rpm > self.target_rpm:
            self.current_rpm = max(self.current_rpm - 200, self.target_rpm)
        
        # 更新关节角速度
        if self.rotor_joint_id is not None and data is not None:
            angular_vel = self.current_rpm * 2 * np.pi / 60  # RPM转rad/s
            if hasattr(data, 'qvel'):
                data.qvel[self.rotor_joint_id] = angular_vel

