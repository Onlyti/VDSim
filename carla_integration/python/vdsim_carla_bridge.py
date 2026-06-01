"""
VDSim ↔ CARLA bridge — drive a CARLA actor with VDSim's dynamics.

Pipeline (per tick):
    1. read user-side ControlInput (throttle/brake/steer or higher Lc tier)
    2. query 4-wheel contacts from CARLA via raycast (world.cast_ray)
    3. VDSim dynamics.step()  — produces the new pose in ISO 8855 RH world
    4. convert ISO 8855 RH → CARLA / UE4 frame (y, yaw sign flip)
    5. actor.set_transform() — CARLA renders the actor at the new pose

CARLA's own physics is disabled on the actor (set_simulate_physics(False))
so VDSim is the sole authority for vehicle motion. CARLA still handles
rendering, sensors, traffic, world state.

Tested against CARLA 0.9.15 client. 0.9.16 client should also work.
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

# vdsim module path
REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))

try:
    import vdsim
except ImportError as e:
    raise SystemExit(
        f"Could not import vdsim: {e}\n"
        f"Expected at {REPO / 'build/python/'}. "
        f"Build with: cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build"
    )

try:
    import carla
except ImportError as e:
    raise SystemExit(
        f"Could not import carla: {e}\n"
        f"Install with: pip install carla==0.9.15 (or matching server)"
    )


# -----------------------------------------------------------------------------
# Coordinate frame conversion — ISO 8855 RH ↔ UE4 LH
# -----------------------------------------------------------------------------
#
# VDSim body frame:  +x forward, +y leftward,  +z up   (ISO 8855 RH)
# CARLA / UE4:       +x forward, +y rightward, +z up   (left-handed)
#
# Mapping:
#   x_carla   = +x_vdsim
#   y_carla   = -y_vdsim
#   z_carla   = +z_vdsim
#   yaw_carla = -yaw_vdsim
#   roll, pitch — sign flips depend on convention; PoC keeps them direct.
#
# CARLA Rotation uses degrees, VDSim uses radians.


def vdsim_pose_to_carla(x_v: float, y_v: float, z_v: float,
                         yaw_rad: float, roll_rad: float = 0.0,
                         pitch_rad: float = 0.0) -> carla.Transform:
    loc = carla.Location(x=float(x_v), y=float(-y_v), z=float(z_v))
    rot = carla.Rotation(
        yaw=math.degrees(-yaw_rad),
        pitch=math.degrees(-pitch_rad),
        roll=math.degrees(roll_rad),
    )
    return carla.Transform(loc, rot)


def carla_velocity_from_vdsim_body(vx: float, vy: float,
                                    yaw_rad: float) -> carla.Vector3D:
    """Convert VDSim body-frame velocity to CARLA world velocity."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    vx_w = vx * c - vy * s
    vy_w = vx * s + vy * c
    return carla.Vector3D(x=float(vx_w), y=float(-vy_w), z=0.0)


# -----------------------------------------------------------------------------
# Wheel position offsets (body frame)
# -----------------------------------------------------------------------------
def wheel_offsets_body(vp) -> List[Tuple[float, float]]:
    a = vp.cg_to_front
    b = vp.cg_to_rear
    tw_f = vp.track_front / 2.0
    tw_r = vp.track_rear / 2.0
    return [
        (+a, +tw_f),   # FL
        (+a, -tw_f),   # FR
        (-b, +tw_r),   # RL
        (-b, -tw_r),   # RR
    ]


# -----------------------------------------------------------------------------
# Bridge
# -----------------------------------------------------------------------------
@dataclass
class BridgeConfig:
    vehicle_yaml: str
    tire_yaml: str
    level: str = "L2"                              # L1 / L2 / L3
    blueprint: str = "vehicle.tesla.model3"
    spawn_index: int = 0                            # which spawn point
    sync_mode: bool = True                          # CARLA fixed timestep
    fixed_delta_seconds: float = 0.02               # 50 Hz CARLA tick
    inner_dt: float = 0.005                         # VDSim substep
    ray_height_above: float = 0.5                   # m above wheel center
    ray_depth: float = 5.0                          # m raycast max
    default_mu: float = 1.0
    initial_vx: float = 0.0                         # m/s
    initial_world_xyz: Optional[Tuple[float, float, float]] = None
    initial_world_yaw_rad: Optional[float] = None
    # Ld4 hardpoint kinematics — attach offline-computed lookup tables to L3.
    # Paths relative to repo or absolute.  None = legacy phenomenological
    # camber_per_roll fallback.
    kinematics_front_csv: Optional[str] = None
    kinematics_rear_csv:  Optional[str] = None


class VDSimCarlaBridge:
    """Drives a CARLA actor with VDSim's dynamics."""

    def __init__(self, cfg: BridgeConfig,
                 client: carla.Client):
        self.cfg = cfg
        self.client = client
        self.world = client.get_world()

        # World settings — fixed step
        settings = self.world.get_settings()
        self._orig_settings = settings
        if cfg.sync_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = cfg.fixed_delta_seconds
            self.world.apply_settings(settings)

        # VDSim
        self.vp = vdsim.VehicleParams.from_yaml(cfg.vehicle_yaml)
        self.tp = vdsim.TireParams.from_yaml(cfg.tire_yaml)
        self.sp = vdsim.SolverParams()

        if cfg.level == "L1":
            self.dyn = vdsim.create_bicycle()
            self.level_enum = vdsim.Level.L1_Bicycle
        elif cfg.level == "L3":
            self.dyn = vdsim.create_fourteen_dof()
            self.level_enum = vdsim.Level.L3_FourteenDOF
        else:
            self.dyn = vdsim.create_seven_dof()
            self.level_enum = vdsim.Level.L2_SevenDOF
        self.dyn.initialize(self.vp, self.tp, self.sp)

        # Optional Ld4 hardpoint kinematics (only meaningful for L3).
        if cfg.level == "L3":
            if cfg.kinematics_front_csv:
                ok = vdsim.attach_front_kinematics(self.dyn, cfg.kinematics_front_csv)
                if not ok:
                    raise RuntimeError("attach_front_kinematics returned False")
            if cfg.kinematics_rear_csv:
                ok = vdsim.attach_rear_kinematics(self.dyn, cfg.kinematics_rear_csv)
                if not ok:
                    raise RuntimeError("attach_rear_kinematics returned False")

        # Initial state
        st0 = vdsim.State()
        st0.velocity = [cfg.initial_vx, 0.0, 0.0]
        self.dyn.reset(st0)

        # CARLA actor
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.find(cfg.blueprint)
        bp.set_attribute("role_name", "vdsim_ego")
        spawn_points = self.world.get_map().get_spawn_points()
        spawn = spawn_points[cfg.spawn_index] if spawn_points else carla.Transform()
        if cfg.initial_world_xyz is not None:
            spawn.location = carla.Location(*cfg.initial_world_xyz)
        if cfg.initial_world_yaw_rad is not None:
            spawn.rotation.yaw = math.degrees(-cfg.initial_world_yaw_rad)

        self.actor = self.world.try_spawn_actor(bp, spawn)
        if self.actor is None:
            raise RuntimeError(f"Failed to spawn actor at {spawn.location}")
        # Disable CARLA's own physics — VDSim is sole authority
        self.actor.set_simulate_physics(False)

        # Bookkeeping for body-frame state mapping to world
        self._world_x = spawn.location.x
        self._world_y_carla = spawn.location.y
        self._world_z = spawn.location.z
        self._world_yaw_carla_rad = math.radians(spawn.rotation.yaw)

        # Wheel offsets (body frame)
        self._wheel_off = wheel_offsets_body(self.vp)

    # ------------------------------------------------------------------------
    # Contact query via CARLA raycast (world.cast_ray)
    # ------------------------------------------------------------------------
    def query_contacts(self) -> List[vdsim.ContactPoint]:
        """For each wheel, raycast downward; populate VDSim ContactPoint."""
        contacts = []
        cos_y = math.cos(-self._world_yaw_carla_rad)
        sin_y = math.sin(-self._world_yaw_carla_rad)
        for (off_x_body, off_y_body) in self._wheel_off:
            # ISO 8855 RH body → CARLA world (y sign flip on offset too)
            off_y_carla = -off_y_body
            # rotate by yaw (carla is LH so rotation matrix uses -yaw)
            dx_world =  cos_y * off_x_body - sin_y * off_y_carla
            dy_world =  sin_y * off_x_body + cos_y * off_y_carla
            start = carla.Location(
                x=self._world_x + dx_world,
                y=self._world_y_carla + dy_world,
                z=self._world_z + self.cfg.ray_height_above,
            )
            end = carla.Location(start.x, start.y, start.z - self.cfg.ray_depth)
            hits = self.world.cast_ray(start, end)
            cp = vdsim.ContactPoint()
            if hits:
                # closest hit
                h = min(hits, key=lambda hh: abs(hh.location.z - start.z))
                cp.is_valid = True
                cp.normal = [0.0, 0.0, 1.0]                # CARLA cast_ray
                                                            # doesn't return normal;
                                                            # assume flat
                cp.position = [h.location.x, -h.location.y, h.location.z]
                cp.mu_long = self.cfg.default_mu
                cp.mu_lat  = self.cfg.default_mu
                # surface_id 도 default
                cp.surface_id = 0
            else:
                cp.is_valid = False
            contacts.append(cp)
        return contacts

    # ------------------------------------------------------------------------
    # One tick: apply control, step VDSim, sync to CARLA
    # ------------------------------------------------------------------------
    def step(self, throttle: float, brake: float, steer: float):
        cmd = vdsim.CmdL4()
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        cmd.steer_angle_wheel = float(steer)

        contacts = self.query_contacts()
        # VDSim inner step
        outer_dt = self.cfg.fixed_delta_seconds
        n = max(1, int(round(outer_dt / self.cfg.inner_dt)))
        h = outer_dt / n
        for _ in range(n):
            self.dyn.step(cmd, contacts, h)

        # Read VDSim state and push to CARLA actor
        s = self.dyn.state()
        body_x_world = s.position[0]
        body_y_world = s.position[1]
        body_z_world = self._world_z          # PoC keeps z fixed at spawn

        # Translate spawn-relative VDSim coords back to CARLA world.
        # VDSim's frame starts at (0,0); add spawn offset to keep absolute.
        # But VDSim integrates absolute world from reset, so we treat its
        # output as a delta from the *initial* world location.
        # For PoC, we map (s.x, s.y) directly to world relative.

        # On first call, latch the initial world (handled in __init__).
        # Construct full world coordinate:
        # ISO 8855 → CARLA: y flip
        carla_x = body_x_world + (self._world_x if False else 0.0)
        # We instead just place the actor at the integrated VDSim pose + spawn:
        # (Spawn-anchored mode is simpler for PoC.)
        # For demo, use VDSim integration as world-relative to spawn:
        # World pose = spawn_pose ⊕ VDSim integrated delta.

        # Build transform from VDSim absolute pose (spawn anchor handled here)
        loc = carla.Location(
            x=self._world_x + body_x_world,
            y=self._world_y_carla - body_y_world,            # ISO → CARLA y flip
            z=body_z_world,
        )
        rot = carla.Rotation(
            yaw=math.degrees(self._world_yaw_carla_rad - s.yaw()),
            pitch=math.degrees(-self.dyn.pitch_angle_qs()),
            roll=math.degrees(self.dyn.roll_angle_qs()),
        )
        self.actor.set_transform(carla.Transform(loc, rot))
        self.actor.set_target_velocity(
            carla_velocity_from_vdsim_body(s.vx(), s.vy(),
                                            self._world_yaw_carla_rad - s.yaw())
        )
        self.actor.set_target_angular_velocity(
            carla.Vector3D(x=0.0, y=0.0, z=math.degrees(-s.yaw_rate()))
        )

        # Tick CARLA in sync mode (server applies the world step)
        if self.cfg.sync_mode:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    # ------------------------------------------------------------------------
    # Telemetry passthrough
    # ------------------------------------------------------------------------
    def telemetry(self) -> dict:
        s = self.dyn.state()
        return {
            "level": str(self.level_enum),
            "vx": s.vx(),
            "vy": s.vy(),
            "yaw_rate": s.yaw_rate(),
            "ax": self.dyn.ax_body_est(),
            "ay": self.dyn.ay_body_est(),
            "roll": self.dyn.roll_angle_qs(),
            "pitch": self.dyn.pitch_angle_qs(),
            "Fz": list(self.dyn.tire_Fz()),
        }

    # ------------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------------
    def close(self):
        if self.actor is not None and self.actor.is_alive:
            self.actor.destroy()
        if self.cfg.sync_mode and self._orig_settings is not None:
            self.world.apply_settings(self._orig_settings)
