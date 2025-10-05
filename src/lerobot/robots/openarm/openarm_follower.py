
#!/usr/bin/env python
import logging
import time
from functools import cached_property
from typing import Any, Dict, List, Optional


from lerobot.cameras.utils import make_cameras_from_configs

from lerobot.robots import Robot
from .config_openarm_follower import OpenArmFollowerConfig

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

class OpenArmFollower(Robot):
    """
    [OpenArm Follower]()
    """

    config_class = OpenArmFollowerConfig
    name = "openarm_follower"

    def __init__(self, config : OpenArmFollowerConfig):
        super().__init__(config)
        self.config = config
        self._arm: Optional[oa.OpenArm] = None

        self.joint_names: List[str] = list(self.config.joint_names)

        # gripper
        self.has_gripper = (
            self.config.gripper_name is not None
            and self.config.gripper_motor_type is not None
            and self.config.gripper_send_id is not None
            and self.config.gripper_recv_id is not None
        )

        # PD gains length = arm joints + (gripper ? 1 : 0)
        n = len(self.joint_names) + (1 if self.has_gripper else 0)
        if not self.config.pd_gains.get("kp"):
            self.config.pd_gains["kp"] = [0.0] * n
        if not self.config.pd_gains.get("kd"):
            self.config.pd_gains["kd"] = [0.0] * n

        self.cameras = {}

    @property
    def _motors_ft(self) -> dict[str, type]:
        ft = {f"{name}.pos": float for name in self.joint_names}
        ft[f"{self.config.gripper_name}.pos"] = float
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
        arm_ok = self._arm is not None
        cams_ok = all(cam.is_connected for cam in self.cameras.values()) if self.cameras else True
        return arm_ok and cams_ok

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self} already connected")

        self._arm = oa.OpenArm(self.config.can_port, True)

        # arm
        motor_types = [MOTOR_TYPE_MAP[m] for m in self.config.motor_types]
        self._arm.init_arm_motors(motor_types, self.config.send_ids, self.config.recv_ids)

        # gripper
        gm = MOTOR_TYPE_MAP[self.config.gripper_motor_type]
        self._arm.init_gripper_motor(gm, self.config.gripper_send_id, self.config.gripper_recv_id)

        self._arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self._arm.enable_all()
        time.sleep(0.1)
        self._arm.recv_all()

        if self.config.cameras:
            self.cameras = make_cameras_from_configs(self.config.cameras)
            for cam in self.cameras.values():
                cam.connect()

        # self.configure()

        logger.info(f"{self} connected")

        logger.info("OpenArmFollower connected")

    def disconnect(self) -> None:
        for cam in self.cameras.values():
            try: cam.disconnect()
            except: pass
        self.cameras.clear()
        if self._arm:
            try: self._arm.disable_all()
            except: pass
            self._arm = None
        logger.info("OpenArmFollower disconnected")

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("OpenArmFollower not connected")

        self._arm.recv_all(220)

        # arm joints
        ms_arm = self._arm.get_arm().get_motors()
        q_arm = [float(m.get_position()) for m in ms_arm]
        obs: Dict[str, Any] = {f"{name}.pos": q_arm[i] for i, name in enumerate(self.joint_names)}

        # gripper (optional)
        if self.has_gripper:
            ms_grp = self._arm.get_gripper().get_motors()
            qg = float(ms_grp[0].get_position())
            obs[f"{self.config.gripper_name}.pos"] = qg

        # cameras
        for name, cam in self.cameras.items():
            frame = cam.async_read()
            if frame is not None:
                obs[name] = frame
        return obs

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("OpenArmFollower not connected")

        # concat q_des for arm + (optional) gripper
        qdes_arm = [float(action.get(f"{name}.pos", 0.0)) for name in self.joint_names]
        qdes = list(qdes_arm)

        if self.has_gripper:
            qg_des = float(action.get(f"{self.config.gripper_name}.pos", 0.0))
            qdes.append(qg_des)

        kp = self.config.pd_gains["kp"]
        kd = self.config.pd_gains["kd"]

        # pack MIT params: p=q_des, v=0, kp=kp, kd=kd, tau=0
        params = [oa.MITParam(float(kp[i]), float(kd[i]), float(qdes[i]), 0.0, 0.0)
                  for i in range(len(qdes))]

        # send to arm + gripper
        n_arm = len(self.joint_names)
        self._arm.get_arm().mit_control_all(params[:n_arm])
        if self.has_gripper:
            self._arm.get_gripper().mit_control_all(params[n_arm:])

        # return sent action in SO100 style
        out = {f"{name}.pos": qdes_arm[i] for i, name in enumerate(self.joint_names)}
        if self.has_gripper:
            out[f"{self.config.gripper_name}.pos"] = qdes[-1]
        return out

    @property
    def is_calibrated(self) -> bool:
        return True
    def calibrate(self) -> None: ...
    # def configure(self) -> None: ...

    def configure(self) -> None:
        """Ramp from current joint angles (and gripper) to zero safely."""
        if not self._arm:
            raise RuntimeError("configure(): arm is not connected")

        # --- ramp profile ---
        duration_s = 3.0
        rate_hz    = 100.0
        dt         = 1.0 / rate_hz
        steps      = max(1, int(duration_s * rate_hz))

        kp_list = (self.config.pd_gains.get("kp") or []) if hasattr(self.config, "pd_gains") else []
        kd_list = (self.config.pd_gains.get("kd") or []) if hasattr(self.config, "pd_gains") else []
        def gain(i, default_kp=7.0, default_kd=0.04):
            kp = float(kp_list[i]) if i < len(kp_list) else default_kp
            kd = float(kd_list[i]) if i < len(kd_list) else default_kd
            return kp, kd

        # self._arm.recv_all(220)
        ms_arm = self._arm.get_arm().get_motors()
        if not ms_arm:
            raise RuntimeError("configure(): arm motors not initialized")
        q_now = [float(m.get_position()) for m in ms_arm]
        n_arm = len(q_now)

        if self.has_gripper:
            ms_grp = self._arm.get_gripper().get_motors()
            if not ms_grp:
                raise RuntimeError("configure(): gripper motor not initialized")
            q_now.append(float(ms_grp[0].get_position()))

        n_all = len(q_now)
        q_goal = [0.0] * n_all

        import time
        for k in range(1, steps + 1):
            a = k / steps
            q_cmd = [(1 - a) * s + a * t for s, t in zip(q_now, q_goal)]

            params = []
            for i in range(n_all):
                kp, kd = gain(i)
                params.append(oa.MITParam(float(q_cmd[i]), 0.0, kp, kd, 0.0))

            self._arm.get_arm().mit_control_all(params[:n_arm])
            if self.has_gripper:
                self._arm.get_gripper().mit_control_all(params[n_arm:])

            self._arm.recv_all(220)
            time.sleep(dt)

        params_final = []
        for i in range(n_all):
            kp, kd = gain(i)
            params_final.append(oa.MITParam(0.0, 0.0, kp, kd, 0.0))
        self._arm.get_arm().mit_control_all(params_final[:n_arm])
        if self.has_gripper:
            self._arm.get_gripper().mit_control_all(params_final[n_arm:])

def main():
    cameras_cfg = {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="218622276565",
            fps=15, width=640, height=480,
            use_depth=True,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION
        ),
        # "front": OpenCVCameraConfig(index_or_path=0, width=1280, height=720, fps=30),
    }

    cfg = OpenArmFollowerConfig(
        can_port="can0",
        send_ids=[0x01,0x02,0x03,0x04,0x05,0x06,0x07],
        recv_ids=[0x11,0x12,0x13,0x14,0x15,0x16,0x17],
        motor_types=["DM8009","DM8009","DM4340","DM4340","DM4310","DM4310","DM4310"],
        joint_names=["j1","j2","j3","j4","j5","j6","j7"],
        pd_gains={"kp":[0.0]*8, "kd":[0.0]*8},
        gripper_name="gripper",
        gripper_motor_type="DM4310",
        gripper_send_id=0x08,
        gripper_recv_id=0x18,
        cameras=cameras_cfg,
    )

    robot = OpenArmFollower(cfg)

    print("Connecting robot...")
    robot.connect()
    print("Connected!")

    try:
        for step in range(50):
            obs = robot.get_observation()
            q_now = {k: v for k, v in obs.items() if k.endswith(".pos")}
            print(f"[{step}] q_now: {q_now}")

            if step % 10 == 0 and "wrist" in robot.cameras:
                color = robot.cameras["wrist"].read()
                try:
                    depth = robot.cameras["wrist"].read_depth()
                    print(f"[{step}] wrist color: {color.shape}, depth: {depth.shape}")
                except AttributeError:
                    print(f"[{step}] wrist color: {color.shape}")

            action = {k: q_now[k] for k in q_now}
            sent = robot.send_action(action)
            print(f"[{step}] sent: {sent}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        robot.disconnect()
        print("Disconnected.")

if __name__ == "__main__":
    main()
