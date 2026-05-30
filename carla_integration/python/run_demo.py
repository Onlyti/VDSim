"""
VDSim + CARLA demo — drive an actor in CARLA with VDSim's Ld2-SevenDOF.

Prereq:
    1. CARLA server running, e.g. ./CarlaUE4.sh -RenderOffScreen
       (default localhost:2000).
    2. pip install carla==0.9.15 matching the server version.
    3. VDSim built with VDSIM_BUILD_PYTHON=ON (build/python/vdsim*.so).

Usage:
    python3 carla_integration/python/run_demo.py \
        --host localhost --port 2000 \
        --vehicle configs/vehicles/sports.yaml \
        --tire    configs/tires/default_pacejka.yaml \
        --level   L2 \
        --blueprint vehicle.tesla.model3 \
        --duration 30 \
        --v_target 12 \
        --driver

When --driver is set, a built-in Pure Pursuit follows a figure-8 around the
spawn point and a Lc6-VTarget PI controls throttle/brake.
Otherwise a square-wave steer demo is used.

A spectator camera follows the ego from behind.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import carla

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "carla_integration" / "python"))

from vdsim_carla_bridge import BridgeConfig, VDSimCarlaBridge   # noqa: E402


def make_figure_eight(spawn, R=20.0, n_per_loop=80):
    """World-frame waypoints (CARLA coords) for a figure-8 centred on spawn."""
    pts = []
    cx, cy = spawn.location.x, spawn.location.y
    cos_y = math.cos(math.radians(spawn.rotation.yaw))
    sin_y = math.sin(math.radians(spawn.rotation.yaw))
    for i in range(n_per_loop):
        t = 2 * math.pi * i / n_per_loop
        lx = R - R * math.cos(t)
        ly = R * math.sin(t)
        pts.append((cx + cos_y * lx - sin_y * ly,
                    cy + sin_y * lx + cos_y * ly))
    for i in range(n_per_loop):
        t = 2 * math.pi * i / n_per_loop
        lx = -R + R * math.cos(t)
        ly = R * math.sin(t)
        pts.append((cx + cos_y * lx - sin_y * ly,
                    cy + sin_y * lx + cos_y * ly))
    return pts


def pure_pursuit_steer(x, y, yaw_rad, vx, wheelbase, path, prev_idx=0):
    Ld = max(2.0, 0.45 * max(vx, 1.0))
    idx = prev_idx
    n = len(path)
    while idx < n:
        dx = path[idx][0] - x
        dy = path[idx][1] - y
        if math.hypot(dx, dy) >= Ld:
            break
        idx += 1
    if idx >= n:
        idx = n - 1
    cp, sp = math.cos(yaw_rad), math.sin(yaw_rad)
    dx =  cp * (path[idx][0] - x) + sp * (path[idx][1] - y)
    dy = -sp * (path[idx][0] - x) + cp * (path[idx][1] - y)
    l2 = dx * dx + dy * dy
    if l2 < 1e-6:
        return 0.0, idx
    kappa = 2.0 * dy / l2
    steer = math.atan(kappa * wheelbase)
    return max(-0.5, min(0.5, steer)), idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--vehicle", default=str(REPO / "configs/vehicles/sports.yaml"))
    ap.add_argument("--tire",    default=str(REPO / "configs/tires/default_pacejka.yaml"))
    ap.add_argument("--level",   default="L2", choices=["L1", "L2", "L3"])
    ap.add_argument("--blueprint", default="vehicle.tesla.model3")
    ap.add_argument("--spawn-idx", type=int, default=0)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--v_target", type=float, default=10.0)
    ap.add_argument("--driver", action="store_true",
                    help="Pure Pursuit on figure-8 (default: square steer demo)")
    ap.add_argument("--no-spectator", action="store_true")
    args = ap.parse_args()

    print(f"[vdsim-carla] connecting to {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    print(f"[vdsim-carla] world map: {world.get_map().name}")

    cfg = BridgeConfig(
        vehicle_yaml=args.vehicle,
        tire_yaml=args.tire,
        level=args.level,
        blueprint=args.blueprint,
        spawn_index=args.spawn_idx,
        fixed_delta_seconds=0.02,   # 50 Hz tick
        inner_dt=0.005,
        initial_vx=args.v_target,
    )

    bridge = VDSimCarlaBridge(cfg, client)
    print(f"[vdsim-carla] spawned ego at {bridge.actor.get_location()}")

    spawn = world.get_actors().find(bridge.actor.id).get_transform()
    path = make_figure_eight(spawn) if args.driver else None
    prev_idx = 0
    spectator = world.get_spectator() if not args.no_spectator else None

    t_start = time.time()
    sim_t = 0.0
    dt = cfg.fixed_delta_seconds
    n_steps = int(args.duration / dt)
    try:
        for step in range(n_steps):
            # control
            tele = bridge.telemetry()
            vx = tele["vx"]
            if args.driver and path is not None:
                loc = bridge.actor.get_transform().location
                rot = bridge.actor.get_transform().rotation
                yaw = math.radians(-rot.yaw)   # CARLA → ISO 8855
                steer, prev_idx = pure_pursuit_steer(
                    loc.x, -loc.y, yaw, vx, bridge.vp.wheelbase, path, prev_idx)
                e = args.v_target - vx
                throttle = max(0.0, min(1.0, 0.10 + 0.30 * e))
                brake = 0.0 if e > -1 else 0.2
            else:
                steer = 0.05 if (sim_t % 8) < 4 else -0.05
                throttle = 0.10
                brake = 0.0

            bridge.step(throttle, brake, steer)
            sim_t += dt

            # spectator camera follows ego
            if spectator is not None:
                tf = bridge.actor.get_transform()
                cam_yaw = math.radians(tf.rotation.yaw)
                cam_loc = carla.Location(
                    x=tf.location.x - 8.0 * math.cos(cam_yaw),
                    y=tf.location.y - 8.0 * math.sin(cam_yaw),
                    z=tf.location.z + 4.0,
                )
                cam_rot = carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw, roll=0)
                spectator.set_transform(carla.Transform(cam_loc, cam_rot))

            # log every second
            if step % int(1.0 / dt) == 0:
                print(f"  t={sim_t:6.2f}  vx={vx:5.2f}  steer={steer:+.3f}  "
                      f"r={tele['yaw_rate']:+.4f}  Fz={[round(f) for f in tele['Fz']]}")
    finally:
        elapsed = time.time() - t_start
        print(f"[vdsim-carla] {n_steps} ticks in {elapsed:.2f}s "
              f"({n_steps/elapsed:.1f} TPS); cleaning up")
        bridge.close()


if __name__ == "__main__":
    main()
