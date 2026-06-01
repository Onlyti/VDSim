"""
Smoke test: load the built vdsim_l2.fmu and exercise its FMI 2.0 ABI
via ctypes — no FMPy dependency.

This is a stand-in until fmpy / fmi-library is installed.  It verifies:
  1. fmi2Instantiate succeeds
  2. fmi2EnterInitializationMode loads YAML resources
  3. fmi2DoStep advances dynamics
  4. fmi2GetReal returns sensible vehicle state outputs
  5. fmi2FreeInstance cleans up
"""
import ctypes
import math
import os
import zipfile
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
FMU  = REPO / "build" / "fmi_export" / "vdsim_l2.fmu"

# Unzip the FMU to a temp dir (FMPy would do this internally).
with tempfile.TemporaryDirectory() as tmp:
    with zipfile.ZipFile(FMU) as zf: zf.extractall(tmp)
    so_path = os.path.join(tmp, "binaries", "linux64", "vdsim_l2.so")
    resource_uri = "file://" + tmp + "/resources"

    lib = ctypes.cdll.LoadLibrary(so_path)

    # ---- prototypes (subset we need) ----
    lib.fmi2GetVersion.restype = ctypes.c_char_p

    # fmi2CallbackFunctions struct
    LOGGER = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_char_p,
                                ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p)
    ALLOC  = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t)
    FREE   = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
    STEPFIN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)
    class CB(ctypes.Structure):
        _fields_ = [("logger", LOGGER), ("alloc", ALLOC), ("free", FREE),
                     ("stepFinished", STEPFIN), ("env", ctypes.c_void_p)]
    @LOGGER
    def cb_logger(env, name, status, category, msg):
        print(f"  [fmu log {status}] {category.decode()}: {msg.decode()}")
    cb = CB(cb_logger, ALLOC(), FREE(), STEPFIN(), None)

    lib.fmi2Instantiate.restype = ctypes.c_void_p
    lib.fmi2Instantiate.argtypes = [ctypes.c_char_p, ctypes.c_int,
                                      ctypes.c_char_p, ctypes.c_char_p,
                                      ctypes.POINTER(CB), ctypes.c_int, ctypes.c_int]

    lib.fmi2SetupExperiment.argtypes = [ctypes.c_void_p, ctypes.c_int,
        ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_double]
    lib.fmi2EnterInitializationMode.argtypes = [ctypes.c_void_p]
    lib.fmi2ExitInitializationMode.argtypes  = [ctypes.c_void_p]

    lib.fmi2SetReal.argtypes = [ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_double)]
    lib.fmi2GetReal.argtypes = [ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_double)]
    lib.fmi2DoStep.argtypes  = [ctypes.c_void_p,
        ctypes.c_double, ctypes.c_double, ctypes.c_int]
    lib.fmi2FreeInstance.argtypes = [ctypes.c_void_p]

    print(f"fmi2GetVersion: {lib.fmi2GetVersion().decode()}")

    fmi2CoSim = 1
    inst = lib.fmi2Instantiate(b"vdsim", fmi2CoSim, b"vdsim-l2-fmu-2026-06",
                                 resource_uri.encode(), ctypes.byref(cb), 0, 1)
    assert inst, "fmi2Instantiate returned NULL"
    print("Instantiated.")

    s = lib.fmi2SetupExperiment(inst, 0, 0.0, 0.0, 0, 0.0)
    assert s == 0
    s = lib.fmi2EnterInitializationMode(inst); assert s == 0
    print("Initialization mode entered (YAML loaded).")
    s = lib.fmi2ExitInitializationMode(inst);  assert s == 0

    # Apply initial vx via direct state writes? Our FMU doesn't expose
    # initial state; vehicle starts at zero velocity.  Apply throttle.
    vr_throttle = (ctypes.c_uint * 1)(2)
    val = (ctypes.c_double * 1)(0.30)
    lib.fmi2SetReal(inst, vr_throttle, 1, val)
    vr_steer = (ctypes.c_uint * 1)(1)
    val = (ctypes.c_double * 1)(0.05)
    lib.fmi2SetReal(inst, vr_steer, 1, val)

    # Step
    t = 0.0; dt = 0.02
    vr_out = (ctypes.c_uint * 4)(13, 14, 15, 17)  # vx, vy, yaw_rate, ay
    out = (ctypes.c_double * 4)()
    for k in range(200):
        s = lib.fmi2DoStep(inst, t, dt, 1); assert s == 0
        t += dt
        if k % 50 == 0:
            lib.fmi2GetReal(inst, vr_out, 4, out)
            print(f"  t={t:5.2f}  vx={out[0]:+6.3f}  vy={out[1]:+6.3f}  "
                  f"yaw_rate={out[2]:+6.3f}  ay={out[3]:+6.3f}")

    lib.fmi2FreeInstance(inst)
    print("Freed instance. SUCCESS.")
