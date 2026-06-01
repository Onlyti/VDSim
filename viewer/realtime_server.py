"""
WebSocket realtime simulation server for VDSim 3D viewer.

Runs vdsim (Ld2-SevenDOF by default) at real-time pacing and streams the
state as JSON to any connected browser client.

Usage:
    pip install websockets
    python3 viewer/realtime_server.py [--port 8765]
                                       [--vehicle configs/vehicles/sports.yaml]
                                       [--tire    configs/tires/default_pacejka.yaml]
                                       [--level L1|L2|L3]
                                       [--fps 30]
                                       [--driver]   # use Pure Pursuit on figure-eight
                                       [--v_target 10.0]
"""
import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))

try:
    import websockets
except ImportError:
    print("Please: pip install websockets", file=sys.stderr)
    sys.exit(1)

try:
    import vdsim
except ImportError as e:
    print(f"Could not import vdsim: {e}", file=sys.stderr)
    print(f"Expected at: {REPO / 'build/python/'}", file=sys.stderr)
    print("Build first: cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build", file=sys.stderr)
    sys.exit(1)


def make_dyn(level: str):
    if level == "L1":
        return vdsim.create_bicycle(), vdsim.Level.L1_Bicycle
    elif level == "L3":
        return vdsim.create_fourteen_dof(), vdsim.Level.L3_FourteenDOF
    return vdsim.create_seven_dof(), vdsim.Level.L2_SevenDOF


def flat_contacts(mu: float = 1.0):
    contacts = [vdsim.ContactPoint() for _ in range(4)]
    for c in contacts:
        c.is_valid = True
        c.normal = [0.0, 0.0, 1.0]
        c.mu_long = mu
        c.mu_lat = mu
    return contacts


class FigureEightPath:
    """Pre-computed waypoints for a figure-eight (radius R)."""
    def __init__(self, R=20.0, n_per_loop=80):
        self.pts = []
        for i in range(n_per_loop):
            t = 2 * math.pi * i / n_per_loop
            self.pts.append((R - R * math.cos(t), R * math.sin(t)))
        for i in range(n_per_loop):
            t = 2 * math.pi * i / n_per_loop
            self.pts.append((-R + R * math.cos(t), R * math.sin(t)))

    def pure_pursuit_steer(self, x, y, yaw, vx, wheelbase, prev_idx=0):
        Ld = max(2.0, 0.45 * max(vx, 1.0))
        idx = prev_idx
        n = len(self.pts)
        while idx < n:
            dx = self.pts[idx][0] - x
            dy = self.pts[idx][1] - y
            if math.hypot(dx, dy) >= Ld:
                break
            idx += 1
        if idx >= n:
            idx = n - 1
        cp, sp = math.cos(yaw), math.sin(yaw)
        dx = cp * (self.pts[idx][0] - x) + sp * (self.pts[idx][1] - y)
        dy = -sp * (self.pts[idx][0] - x) + cp * (self.pts[idx][1] - y)
        l2 = dx * dx + dy * dy
        if l2 < 1e-6:
            return 0.0, idx
        kappa = 2.0 * dy / l2
        steer = math.atan(kappa * wheelbase)
        return max(-0.5, min(0.5, steer)), idx


async def simulation_task(args):
    print(f"[VDSim realtime] loading {args.vehicle}, tire={args.tire}, level={args.level}")
    vp = vdsim.VehicleParams.from_yaml(args.vehicle)
    tp = vdsim.TireParams.from_yaml(args.tire)
    sp = vdsim.SolverParams()
    dyn, level_enum = make_dyn(args.level)
    dyn.initialize(vp, tp, sp)

    state = vdsim.State()
    state.velocity = [args.v_target, 0.0, 0.0]
    dyn.reset(state)

    contacts = flat_contacts(1.0)
    inner_dt = 0.005
    real_dt = 1.0 / args.fps
    steps_per_frame = max(1, int(real_dt / inner_dt))

    path = FigureEightPath() if args.driver else None
    prev_idx = 0
    sim_t = 0.0

    print(f"[VDSim realtime] inner_dt={inner_dt}, steps/frame={steps_per_frame}, "
          f"real_fps={args.fps}, driver={args.driver}")

    while True:
        # Wait until at least one client connected
        if not connected_clients:
            await asyncio.sleep(0.05)
            continue

        for _ in range(steps_per_frame):
            s = dyn.state()
            if args.driver and path is not None:
                steer, prev_idx = path.pure_pursuit_steer(
                    s.position[0], s.position[1], s.yaw(),
                    s.vx(), vp.wheelbase, prev_idx)
                # simple speed PI
                e = args.v_target - s.vx()
                throttle = max(0.0, min(1.0, 0.10 + 0.3 * e))
                brake = 0.0 if e > -1 else 0.2
            else:
                # square steer demo (slalom-ish)
                cycle = (sim_t % 8.0)
                steer = 0.05 if cycle < 4.0 else -0.05
                throttle = 0.10
                brake = 0.0
            cmd = vdsim.CmdL4()
            cmd.throttle = throttle
            cmd.brake = brake
            cmd.steer_angle_wheel = steer
            dyn.step(cmd, contacts, inner_dt)
            sim_t += inner_dt

        s = dyn.state()
        payload = {
            "t": sim_t,
            "x": s.position[0],
            "y": s.position[1],
            "yaw": s.yaw(),
            "vx": s.vx(),
            "vy": s.vy(),
            "r": s.yaw_rate(),
            "ax": dyn.ax_body_est(),
            "ay": dyn.ay_body_est(),
            "roll": dyn.roll_angle_qs(),
            "pitch": dyn.pitch_angle_qs(),
            "throttle": cmd.throttle,
            "brake": cmd.brake,
            "steer": cmd.steer_angle_wheel,
        }
        msg = json.dumps(payload)
        await asyncio.gather(*(ws.send(msg) for ws in list(connected_clients)),
                             return_exceptions=True)
        await asyncio.sleep(real_dt)


connected_clients = set()


async def handle_client(ws):
    connected_clients.add(ws)
    addr = ws.remote_address
    print(f"[VDSim realtime] client connected: {addr}")
    try:
        async for _ in ws:
            pass  # ignore incoming
    except websockets.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(ws)
        print(f"[VDSim realtime] client disconnected: {addr}")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--vehicle", default=str(REPO / "configs/vehicles/sports.yaml"))
    ap.add_argument("--tire",    default=str(REPO / "configs/tires/default_pacejka.yaml"))
    ap.add_argument("--level",   default="L2", choices=["L1", "L2", "L3"])
    ap.add_argument("--fps",     type=int, default=30)
    ap.add_argument("--driver",  action="store_true",
                    help="Closed-loop figure-8 with Pure Pursuit + Lc6 PI")
    ap.add_argument("--v_target", type=float, default=10.0)
    args = ap.parse_args()

    print(f"[VDSim realtime] WebSocket: ws://{args.host}:{args.port}")
    async with websockets.serve(handle_client, args.host, args.port):
        await simulation_task(args)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[VDSim realtime] shutdown")
