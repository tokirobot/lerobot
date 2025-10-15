# src/robots/openarm/openarm_leader.py
#!/usr/bin/env python
import logging
import time
from typing import Dict, List, Optional

import openarm_can as oa

from lerobot.teleoperators import Teleoperator
# from .config_openarm_bimanual_leader import OpenArmLeaderConfig
from lerobot.teleoperators.openarm.config_openarm_leader import OpenArmLeaderConfig
from lerobot.teleoperators.openarm.openarm_leader import OpenArmLeader

from .config_openarm_bimanual_leader import OpenArmLeaderBimanualConfig

logger = logging.getLogger(__name__)

MOTOR_TYPE_MAP = {
    "DM8009": oa.MotorType.DM8009,
    "DM4340": oa.MotorType.DM4340,
    "DM4310": oa.MotorType.DM4310,
}

class OpenArmLeaderBimanual(Teleoperator):

    """
    [OpenArm Leader Bimanual]()
    """
    config_class = OpenArmLeaderBimanualConfig
    name = "openarm_leader_bimanual"

    def __init__(self, config: OpenArmLeaderBimanualConfig):
        super().__init__(config)
        self.config = config

        right_arm_config =OpenArmLeaderConfig(
            can_port=config.right_can_port,
            send_ids=[0x01,0x02,0x03,0x04,0x05,0x06,0x07],
            recv_ids=[0x11,0x12,0x13,0x14,0x15,0x16,0x17],
            motor_types=["DM8009","DM8009","DM4340","DM4340","DM4310","DM4310","DM4310"],
            joint_names=["right_" + j for j in ["j1","j2","j3","j4","j5","j6","j7"]],
            gripper_name="right_gripper",
            gripper_motor_type="DM4310",
            gripper_send_id=0x08,
            gripper_recv_id=0x18,
        )

        left_arm_config = OpenArmLeaderConfig(
            can_port=config.left_can_port,
            send_ids=[0x01,0x02,0x03,0x04,0x05,0x06,0x07],
            recv_ids=[0x11,0x12,0x13,0x14,0x15,0x16,0x17],
            motor_types=["DM8009","DM8009","DM4340","DM4340","DM4310","DM4310","DM4310"],
            joint_names=["left_" + j for j in ["j1","j2","j3","j4","j5","j6","j7"]],
            gripper_name="left_gripper",
            gripper_motor_type="DM4310",
            gripper_send_id=0x08,
            gripper_recv_id=0x18,
        )

        self.right_arm = OpenArmLeader(right_arm_config)
        self.left_arm =OpenArmLeader(left_arm_config)

    @property
    def action_features(self) -> dict[str, type]:
        ft = {}

        for name in self.right_arm.joint_names:
            ft[f"{name}.pos"] = float

        for name in self.left_arm.joint_names:
            ft[f"{name}.pos"] = float

        if getattr(self.right_arm.config, "gripper_name", None):
            ft[f"{self.right_arm.config.gripper_name}.pos"] = float

        if getattr(self.left_arm.config, "gripper_name", None):
            ft[f"{self.left_arm.config.gripper_name}.pos"] = float

        return ft

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # ========== connection state ==========
    @property
    def is_connected(self) -> bool:
        return self._arm is not None

    # ========== lifecycle ==========
    def connect(self, calibrate: bool = True) -> None:
        self.right_arm.connect()
        self.left_arm.connect()
        logger.info("OpenArmLeader connected")

    def disconnect(self) -> None:
        self.right_arm.disconnect()
        self.left_arm.disconnect()
        logger.info("OpenArmLeader disconnected")

    def get_action(self) -> Dict[str, float]:
        action: Dict[str, float] = {}
        action.update(self.right_arm.get_action())
        action.update(self.left_arm.get_action())
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        """Leader side also ramps to zero from its current joint angles."""
        if not self._arm:
            raise RuntimeError("configure(): arm is not connected")

        # duration_s = 3.0
        # rate_hz    = 100.0
        # dt         = 1.0 / rate_hz
        # steps      = max(1, int(duration_s * rate_hz))

        # kp_list = (getattr(self.config, "pd_gains", {}) or {}).get("kp", [])
        # kd_list = (getattr(self.config, "pd_gains", {}) or {}).get("kd", [])
        # def gain(i, default_kp=7.0, default_kd=0.03):
        #     kp = float(kp_list[i]) if i < len(kp_list) else default_kp
        #     kd = float(kd_list[i]) if i < len(kd_list) else default_kd
        #     return kp, kd

        # # self._arm.recv_all(220)
        # ms_arm = self._arm.get_arm().get_motors()
        # if not ms_arm:
        #     raise RuntimeError("configure(): arm motors not initialized")
        # q_now = [float(m.get_position()) for m in ms_arm]
        # n_arm = len(q_now)

        # has_gripper = (
        #     getattr(self.config, "gripper_name", None) is not None and
        #     getattr(self.config, "gripper_motor_type", None) is not None and
        #     getattr(self.config, "gripper_send_id", None) is not None and
        #     getattr(self.config, "gripper_recv_id", None) is not None
        # )
        # if has_gripper:
        #     ms_grp = self._arm.get_gripper().get_motors()
        #     if not ms_grp:
        #         raise RuntimeError("configure(): gripper motor not initialized")
        #     q_now.append(float(ms_grp[0].get_position()))

        # n_all = len(q_now)
        # q_goal = [0.0] * n_all

        # import time
        # for k in range(1, steps + 1):
        #     a = k / steps
        #     q_cmd = [(1 - a) * s + a * t for s, t in zip(q_now, q_goal)]

        #     params = []
        #     for i in range(n_all):
        #         kp, kd = gain(i)
        #         params.append(oa.MITParam(float(q_cmd[i]), 0.0, kp, kd, 0.0))

        #     self._arm.get_arm().mit_control_all(params[:n_arm])
        #     if has_gripper:
        #         self._arm.get_gripper().mit_control_all(params[n_arm:])

        #     self._arm.recv_all(220)
        #     time.sleep(dt)

        # params_final = []
        # for i in range(n_all):
        #     kp, kd = gain(i)
        #     params_final.append(oa.MITParam(0.0, 0.0, kp, kd, 0.0))
        # self._arm.get_arm().mit_control_all(params_final[:n_arm])
        # if has_gripper:
        #     self._arm.get_gripper().mit_control_all(params_final[n_arm:])



def main():
    import time

    bi_cfg = OpenArmLeaderBimanualConfig(
        right_can_port="can0",
        left_can_port="can1",
    )

    leader = OpenArmLeaderBimanual(bi_cfg)
    print("Connecting bimanual leader...")
    leader.connect()
    print("Connected!")

    try:
        for i in range(50):
            act = leader.get_action()  # {"right_j1.pos":..., "left_j1.pos":..., "right_gripper.pos":..., ...}

            compact = {k: round(float(v), 4) for k, v in act.items() if k.endswith(".pos")}
            print(f"[{i}] action:", compact)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        leader.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()
