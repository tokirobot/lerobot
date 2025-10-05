#!/usr/bin/env python
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from lerobot.teleoperators.config import TeleoperatorConfig

# from lerobot.robots.config import RobotConfig

@TeleoperatorConfig.register_subclass("openarm_leader")
# @RobotConfig.register_subclass("openarm_leader")
@dataclass
class OpenArmLeaderConfig(TeleoperatorConfig):
    can_port: str
    send_ids: List[int]
    recv_ids: List[int]
    motor_types: List[str]
    joint_names: List[str]


    gripper_name: Optional[str] = None
    gripper_motor_type: Optional[str] = None
    gripper_send_id: Optional[int] = None
    gripper_recv_id: Optional[int] = None
