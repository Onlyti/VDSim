from runner.params_schema import (
    ACTUATOR_FIELDS,
    ENUM_MAPS,
    SENSOR_FIELDS,
    TIRE_FIELDS,
    VEHICLE_FIELDS,
)


def get_dotted(obj, path):
    for p in path.split("."):
        obj = getattr(obj, p)
    return obj


def set_dotted(obj, path, value):
    parts = path.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    setattr(obj, parts[-1], value)


def field_value(obj, attr, kind):
    if kind == "enum":
        return getattr(obj, attr).name
    if kind == "bool":
        return bool(getattr(obj, attr))
    if kind == "arr":
        return [float(x) for x in getattr(obj, attr)]
    return float(getattr(obj, attr))


def _read_field(obj, attr, kind):
    if "." in attr:
        v = get_dotted(obj, attr)
        if kind == "bool":
            return bool(v)
        if kind == "arr":
            return [float(x) for x in v]
        return float(v)
    return field_value(obj, attr, kind)


def serialize_fields(obj, fields):
    out = []
    for attr, label, group, kind, *rest in fields:
        levels = rest[0] if rest else "L1,L2,L3"
        d = {"name": attr, "label": label, "group": group, "kind": kind,
             "levels": levels.split(","), "value": _read_field(obj, attr, kind)}
        if kind == "enum":
            d["choices"] = list(ENUM_MAPS[attr].keys())
        out.append(d)
    return out


def params_dict(obj, fields):
    return {f[0]: _read_field(obj, f[0], f[3]) for f in fields}


def flat_sensors(sensors):
    out = {}
    for attr, _, _, kind in SENSOR_FIELDS:
        if kind == "bool":
            out[attr] = bool(getattr(sensors, attr))
        else:
            out[attr] = float(get_dotted(sensors, attr))
    return out


def flat_actuator(act, sensor_delay):
    out = {}
    for attr, _, _, kind in ACTUATOR_FIELDS:
        if attr == "@sensor_delay_s":
            out[attr] = float(sensor_delay)
        elif kind == "bool":
            out[attr] = bool(get_dotted(act, attr))
        else:
            out[attr] = float(get_dotted(act, attr))
    return out


def apply_fields(obj, fields, data):
    kinds = {f[0]: f[3] for f in fields}
    for k, v in data.items():
        kind = kinds.get(k)
        if kind is None:
            continue
        if "." in k:
            if kind == "bool":
                set_dotted(obj, k, bool(v))
            elif kind == "arr":
                set_dotted(obj, k, [float(x) for x in v])
            else:
                set_dotted(obj, k, float(v))
        elif not hasattr(obj, k):
            continue
        elif kind == "enum":
            setattr(obj, k, ENUM_MAPS[k][v])
        elif kind == "bool":
            setattr(obj, k, bool(v))
        elif kind == "arr":
            setattr(obj, k, [float(x) for x in v])
        else:
            setattr(obj, k, float(v))
