#!/usr/bin/env python
# openarm_follower_config.py
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from lerobot.robots.config import RobotConfig
from lerobot.cameras import CameraConfig  # or lerobot.common.cameras.configs

@RobotConfig.register_subclass("openarm_follower")
@dataclass
class OpenArmFollowerConfig(RobotConfig):
    can_port: str
    send_ids: List[int]
    recv_ids: List[int]
    motor_types: List[str]
    joint_names: List[str]

    pd_gains: Dict[str, List[float]] = field(default_factory=lambda: {"kp": [], "kd": []})

    # --- optional gripper ---
    gripper_name: Optional[str] = None
    gripper_motor_type: Optional[str] = None
    gripper_send_id: Optional[int] = None
    gripper_recv_id: Optional[int] = None

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    # id: str = "openarm_default"