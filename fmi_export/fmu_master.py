"""
Generic FMI 2.0 Co-Simulation master / FMU loader (pure Python, ctypes-only).

Wraps any FMI 2.0 CS .fmu file as a Python object with a clean API:

    fmu = FMUMaster.load("path/to/something.fmu")
    fmu.initialize(t0=0.0)
    fmu.set("steer_angle_wheel", 0.05)
    fmu.set("throttle", 0.3)
    fmu.do_step(t=0.0, dt=0.02)
    print(fmu.get("vx"), fmu.get("yaw_rate"))

Works on any standards-compliant CS FMU — including our own vdsim_l2.fmu,
Chrono Vehicle FMU export, CarMaker/CarSim FMU export, Modelica generated
FMU, etc.  Variable lookup uses the name from modelDescription.xml.

No FMPy / fmi-library dependency — only Python stdlib (ctypes, zipfile,
xml.etree, tempfile).  Linux x86-64 binaries assumed; trivial to extend.
"""
from __future__ import annotations

import ctypes
import os
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Union


# ---------- FMI 2.0 callback function struct -----------------------------
_LOGGER  = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_char_p,
                              ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p)
_ALLOC   = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t)
_FREE    = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
_STEPFIN = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)


class _CallbackFunctions(ctypes.Structure):
    _fields_ = [
        ("logger",                _LOGGER),
        ("allocateMemory",        _ALLOC),
        ("freeMemory",            _FREE),
        ("stepFinished",          _STEPFIN),
        ("componentEnvironment",  ctypes.c_void_p),
    ]


# ---------- variable metadata ---------------------------------------------
@dataclass
class FMUVar:
    name: str
    vr: int
    type: str         # "Real" / "Integer" / "Boolean" / "String"
    causality: str    # input / output / parameter / ...
    variability: str  # continuous / discrete / fixed / ...
    start: Optional[float] = None


# ---------- main wrapper ---------------------------------------------------
class FMUMaster:
    PLATFORMS = ["linux64", "linux32", "win64", "win32", "darwin64"]

    def __init__(self, lib, model_name: str,
                 vars_by_name: Dict[str, FMUVar],
                 resource_uri: str, guid: str):
        self._lib = lib
        self._inst: Optional[int] = None
        self._t = 0.0
        self.model_name = model_name
        self.vars_by_name = vars_by_name
        self.resource_uri = resource_uri
        self.guid = guid
        self._cb_keepalive = None    # keep callback objects alive

    @classmethod
    def load(cls, fmu_path: Union[str, Path]) -> "FMUMaster":
        fmu_path = Path(fmu_path)
        tmp = tempfile.mkdtemp(prefix="fmu_")
        with zipfile.ZipFile(fmu_path) as zf:
            zf.extractall(tmp)

        # Parse modelDescription.xml
        tree = ET.parse(os.path.join(tmp, "modelDescription.xml"))
        root = tree.getroot()
        model_name = root.attrib.get("modelName", "model")
        guid = root.attrib.get("guid", "00000000-0000-0000-0000-000000000000")
        cs = root.find("CoSimulation")
        if cs is None:
            raise ValueError(f"{fmu_path}: not a CoSimulation FMU")
        model_id = cs.attrib.get("modelIdentifier", model_name)

        # Variables
        vars_by_name: Dict[str, FMUVar] = {}
        for sv in root.find("ModelVariables").findall("ScalarVariable"):
            type_tag = None
            for tag in ("Real", "Integer", "Boolean", "String"):
                child = sv.find(tag)
                if child is not None:
                    type_tag = tag
                    start = child.attrib.get("start")
                    break
            vars_by_name[sv.attrib["name"]] = FMUVar(
                name=sv.attrib["name"],
                vr=int(sv.attrib["valueReference"]),
                type=type_tag or "Real",
                causality=sv.attrib.get("causality", "local"),
                variability=sv.attrib.get("variability", "continuous"),
                start=(float(start) if start is not None and type_tag in
                       ("Real", "Integer") else None),
            )

        # Find binary (platform-dependent)
        so_path = None
        for plat in cls.PLATFORMS:
            for ext in (".so", ".dll", ".dylib"):
                cand = os.path.join(tmp, "binaries", plat, model_id + ext)
                if os.path.exists(cand): so_path = cand; break
            if so_path: break
        if so_path is None:
            raise FileNotFoundError(
                f"No binaries/<platform>/{model_id}.so found in FMU")

        lib = ctypes.cdll.LoadLibrary(so_path)
        cls._bind_signatures(lib)
        version = lib.fmi2GetVersion().decode()
        if not version.startswith("2."):
            raise ValueError(f"FMU version is '{version}', need 2.x")

        return cls(lib, model_name, vars_by_name,
                   resource_uri="file://" + tmp + "/resources",
                   guid=guid)

    @staticmethod
    def _bind_signatures(lib):
        lib.fmi2GetVersion.restype = ctypes.c_char_p
        lib.fmi2Instantiate.restype = ctypes.c_void_p
        lib.fmi2Instantiate.argtypes = [
            ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p,
            ctypes.POINTER(_CallbackFunctions), ctypes.c_int, ctypes.c_int]
        for name, restype, argtypes in [
            ("fmi2FreeInstance", None, [ctypes.c_void_p]),
            ("fmi2SetupExperiment", ctypes.c_int,
             [ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double,
              ctypes.c_int, ctypes.c_double]),
            ("fmi2EnterInitializationMode", ctypes.c_int, [ctypes.c_void_p]),
            ("fmi2ExitInitializationMode",  ctypes.c_int, [ctypes.c_void_p]),
            ("fmi2Reset", ctypes.c_int, [ctypes.c_void_p]),
            ("fmi2Terminate", ctypes.c_int, [ctypes.c_void_p]),
            ("fmi2SetReal", ctypes.c_int,
             [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint),
              ctypes.c_size_t, ctypes.POINTER(ctypes.c_double)]),
            ("fmi2GetReal", ctypes.c_int,
             [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint),
              ctypes.c_size_t, ctypes.POINTER(ctypes.c_double)]),
            ("fmi2DoStep", ctypes.c_int,
             [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_int]),
        ]:
            f = getattr(lib, name)
            f.restype = restype
            f.argtypes = argtypes

    # ----- Lifecycle -----
    def instantiate(self, *, logging_on: bool = False):
        if self._inst: return
        @_LOGGER
        def log_cb(env, name, status, category, msg):
            print(f"[fmu {status}] {category.decode()}: {msg.decode()}")
        cb = _CallbackFunctions(log_cb, _ALLOC(), _FREE(), _STEPFIN(), None)
        self._cb_keepalive = (cb, log_cb)
        self._inst = self._lib.fmi2Instantiate(
            self.model_name.encode(), 1, self.guid.encode(),
            self.resource_uri.encode(), ctypes.byref(cb), 0,
            1 if logging_on else 0)
        if not self._inst:
            raise RuntimeError("fmi2Instantiate failed")

    def initialize(self, t0: float = 0.0, *, logging_on: bool = False):
        if not self._inst: self.instantiate(logging_on=logging_on)
        s = self._lib.fmi2SetupExperiment(self._inst, 0, 1e-9, t0, 0, 0.0)
        if s != 0: raise RuntimeError(f"SetupExperiment status={s}")
        s = self._lib.fmi2EnterInitializationMode(self._inst)
        if s != 0: raise RuntimeError(f"EnterInit status={s}")
        s = self._lib.fmi2ExitInitializationMode(self._inst)
        if s != 0: raise RuntimeError(f"ExitInit status={s}")
        self._t = t0

    def reset(self):
        if self._inst:
            self._lib.fmi2Reset(self._inst)
        self._t = 0.0

    def free(self):
        if self._inst:
            self._lib.fmi2FreeInstance(self._inst)
            self._inst = None

    def __del__(self):
        try: self.free()
        except Exception: pass

    # ----- I/O -----
    def set(self, name: str, value: float):
        if name not in self.vars_by_name:
            raise KeyError(f"Unknown variable: {name}")
        var = self.vars_by_name[name]
        vr = (ctypes.c_uint * 1)(var.vr)
        v  = (ctypes.c_double * 1)(value)
        s = self._lib.fmi2SetReal(self._inst, vr, 1, v)
        if s != 0:
            raise RuntimeError(f"SetReal('{name}') status={s}")

    def get(self, name: str) -> float:
        if name not in self.vars_by_name:
            raise KeyError(f"Unknown variable: {name}")
        var = self.vars_by_name[name]
        vr = (ctypes.c_uint * 1)(var.vr)
        v  = (ctypes.c_double * 1)()
        s = self._lib.fmi2GetReal(self._inst, vr, 1, v)
        if s != 0:
            raise RuntimeError(f"GetReal('{name}') status={s}")
        return float(v[0])

    def set_many(self, **kwargs):
        for k, v in kwargs.items(): self.set(k, v)

    def get_many(self, *names) -> Dict[str, float]:
        return {n: self.get(n) for n in names}

    def do_step(self, t: float, dt: float):
        s = self._lib.fmi2DoStep(self._inst, t, dt, 1)
        if s != 0:
            raise RuntimeError(f"DoStep status={s}")
        self._t = t + dt

    @property
    def t(self): return self._t

    # ----- Introspection -----
    def inputs(self):  return [v for v in self.vars_by_name.values()
                                if v.causality == "input"]
    def outputs(self): return [v for v in self.vars_by_name.values()
                                if v.causality == "output"]


# ---------- CLI demo -------------------------------------------------------
if __name__ == "__main__":
    import argparse, math
    ap = argparse.ArgumentParser()
    ap.add_argument("--fmu", default="build/fmi_export/vdsim_l2.fmu")
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--dt",       type=float, default=0.02)
    args = ap.parse_args()

    print(f"Loading {args.fmu} ...")
    fmu = FMUMaster.load(args.fmu)
    print(f"  modelName : {fmu.model_name}")
    print(f"  inputs    : {[v.name for v in fmu.inputs()]}")
    print(f"  outputs   : {[v.name for v in fmu.outputs()]}")

    fmu.initialize(0.0, logging_on=True)

    # Square-wave steer + constant throttle
    fmu.set("throttle", 0.30)
    n = int(args.duration / args.dt)
    print(f"Running {n} steps × {args.dt}s ...")
    for k in range(n):
        t = k * args.dt
        fmu.set("steer_angle_wheel",
                 +0.05 if int(t * 0.5) % 2 == 0 else -0.05)
        fmu.do_step(t, args.dt)
        if k % 50 == 0:
            st = fmu.get_many("vx", "vy", "yaw_rate", "ay_body")
            print(f"  t={t:5.2f}  vx={st['vx']:+6.3f}  vy={st['vy']:+6.3f}  "
                  f"r={st['yaw_rate']:+6.3f}  ay={st['ay_body']:+6.3f}")

    fmu.free()
    print("Done.")
