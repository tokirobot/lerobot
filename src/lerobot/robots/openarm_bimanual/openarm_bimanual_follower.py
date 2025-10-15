
#!/usr/bin/env python
import logging
import time
from functools import cached_property
from typing import Any, Dict, List, Optional


from lerobot.cameras.utils import make_cameras_from_configs

from lerobot.robots.openarm import OpenArmFollower
from lerobot.robots.openarm.config_openarm_follower import OpenArmFollowerConfig

from lerobot.robots import Robot
from .config_openarm_bimanual_follower import OpenArmFollowerBimanualConfig

from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation
import openarm_can as oa

logger = logging.getLogger(__name__)

MOTOR_TYPE_MAP = {
    "DM8009": oa.MotorType.DM8009,
    "DM4340": oa.MotorType.DM4340,
    "DM4310": oa.MotorType.DM4310,
}

class OpenArmFollowerBimanual(Robot):
    """
    [OpenArm Follower]()
    """

    config_class = OpenArmFollowerBimanualConfig
    name = "openarm_follower_bimanual"

    def __init__(self, config : OpenArmFollowerBimanualConfig):
        super().__init__(config)
        self.config = config

        right_arm_config =OpenArmFollowerConfig(
            can_port=config.right_can_port,
            send_ids=[0x01,0x02,0x03,0x04,0x05,0x06,0x07],
            recv_ids=[0x11,0x12,0x13,0x14,0x15,0x16,0x17],
            motor_types=["DM8009","DM8009","DM4340","DM4340","DM4310","DM4310","DM4310"],
            joint_names=["right_" + j for j in ["j1","j2","j3","j4","j5","j6","j7"]],
            pd_gains=config.pd_gains,
            gripper_name="right_gripper",
            gripper_motor_type="DM4310",
            gripper_send_id=0x08,
            gripper_recv_id=0x18,
            cameras={},
        )

        left_arm_config = OpenArmFollowerConfig(
            can_port=config.left_can_port,
            send_ids=[0x01,0x02,0x03,0x04,0x05,0x06,0x07],
            recv_ids=[0x11,0x12,0x13,0x14,0x15,0x16,0x17],
            motor_types=["DM8009","DM8009","DM4340","DM4340","DM4310","DM4310","DM4310"],
            joint_names=["left_" + j for j in ["j1","j2","j3","j4","j5","j6","j7"]],
            pd_gains=config.pd_gains,
            gripper_name="left_gripper",
            gripper_motor_type="DM4310",
            gripper_send_id=0x08,
            gripper_recv_id=0x18,
            cameras={},
        )

        self.right_arm = OpenArmFollower(right_arm_config)
        self.left_arm = OpenArmFollower(left_arm_config)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
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
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (cfg.height, cfg.width, 3) for cam, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        r = getattr(self.right_arm, "is_connected", False)
        l = getattr(self.left_arm,  "is_connected",  False)
        cams_ok = (not self.cameras) or all(getattr(cam, "is_connected", True) for cam in self.cameras.values())
        return bool(r and l and cams_ok)

    def connect(self) -> None:
        self.right_arm.connect()
        self.left_arm.connect()

        for cam in self.cameras.values():
            cam.connect()

    def disconnect(self) -> None:
        self.right_arm.disconnect()
        self.left_arm.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

    def get_observation(self) -> Dict[str, Any]:
        obs = {}
        obs.update(self.right_arm.get_observation())
        obs.update(self.left_arm.get_observation())

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("OpenArmFollowerBimanual not connected")

        right_action = {k: v for k, v in action.items() if k.startswith("right_")}
        left_action  = {k: v for k, v in action.items() if k.startswith("left_")}

        out_right = self.right_arm.send_action(right_action) if right_action else {}
        out_left  = self.left_arm.send_action(left_action)   if left_action  else {}

        return {**out_right, **out_left }

    @property
    def is_calibrated(self) -> bool:
        return True
    def calibrate(self) -> None: ...
    # def configure(self) -> None: ...

    def configure(self) -> None:
        """Ramp from current joint angles (and gripper) to zero safely."""
        if not self._arm:
            raise RuntimeError("configure(): arm is not connected")

        # # --- ramp profile ---
        # duration_s = 3.0
        # rate_hz    = 100.0
        # dt         = 1.0 / rate_hz
        # steps      = max(1, int(duration_s * rate_hz))

        # kp_list = (self.config.pd_gains.get("kp") or []) if hasattr(self.config, "pd_gains") else []
        # kd_list = (self.config.pd_gains.get("kd") or []) if hasattr(self.config, "pd_gains") else []
        # def gain(i, default_kp=7.0, default_kd=0.04):
        #     kp = float(kp_list[i]) if i < len(kp_list) else default_kp
        #     kd = float(kd_list[i]) if i < len(kd_list) else default_kd
        #     return kp, kd

        # # self._arm.recv_all(220)
        # ms_arm = self._arm.get_arm().get_motors()
        # if not ms_arm:
        #     raise RuntimeError("configure(): arm motors not initialized")
        # q_now = [float(m.get_position()) for m in ms_arm]
        # n_arm = len(q_now)

        # if self.has_gripper:
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
        #     if self.has_gripper:
        #         self._arm.get_gripper().mit_control_all(params[n_arm:])

        #     self._arm.recv_all(220)
        #     time.sleep(dt)

        # params_final = []
        # for i in range(n_all):
        #     kp, kd = gain(i)
        #     params_final.append(oa.MITParam(0.0, 0.0, kp, kd, 0.0))
        # self._arm.get_arm().mit_control_all(params_final[:n_arm])
        # if self.has_gripper:
        #     self._arm.get_gripper().mit_control_all(params_final[n_arm:])
def main():

    # cameras_cfg = {...}

    print("test openarm bimanual follower")

    # ---- bimanual config ----
    bi_cfg = OpenArmFollowerBimanualConfig(
        right_can_port="can0",
        left_can_port="can1",

        pd_gains={
            "kp": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
            "kd": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
        },
        cameras={},
    )

    robot = OpenArmFollowerBimanual(bi_cfg)

    print("Connecting robot (bimanual)...")
    robot.connect()
    print("Connected!")

    try:
        for step in range(50):
            left_obs = robot.left_arm.get_observation()
            right_obs = robot.right_arm.get_observation()

            action = {}
            action.update({k: v for k, v in left_obs.items()  if k.endswith(".pos")})
            action.update({k: v for k, v in right_obs.items() if k.endswith(".pos")})

            sent = robot.send_action(action)

            q_left_now  = {k: v for k, v in left_obs.items()  if k.endswith(".pos")}
            q_right_now = {k: v for k, v in right_obs.items() if k.endswith(".pos")}
            print(f"[{step}] q_left:  {q_left_now}")
            print(f"[{step}] q_right: {q_right_now}")
            print(f"[{step}] sent:    {sent}")

            # カメラは使わないので読み取りもコメントアウト
            # if "left_wrist" in robot.cameras and step % 10 == 0:
            #     pass
            # if "right_wrist" in robot.cameras and step % 10 == 0:
            #     pass

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        robot.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()
