"""
设备控制模块
"""

from .centrifuge import Centrifuge
from .microscope import Microscope
from .fume_hood import FumeHood

__all__ = ['Centrifuge', 'Microscope', 'FumeHood']
