#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hmx_config_check.py -- HARDIMAN-X SSD v3.1 configuration model and checker.

Companion to HARDIMAN-X-SSD_v3_1.md (Sheldon K. Salmon, ORCID 0009-0005-8057-5115).
License: CC BY 4.0.

WHAT THIS IS
    A re-implementation of the configuration model described in SSD sections
    1.8, Appendix B.3 and Appendix F. It computes, for any build (STD plus any
    set of option packs, with or without mass action L1):

        * dry-mass bounds and the GVWR-derived payload rating   (CAR-1)
        * minimum rated brief payload                           (CAR-2)
        * knee cylinder margin and single-support lever cap     (CAR-3)
        * stability margin / offset limits with stabilizers     (CAR-4)
        * unloaded fore-aft CoM offset                          (CAR-5)
        * stowed envelope                                       (CAR-6)
        * safe-posture hydraulic margin                         (CAR-7)
        * pack release gates / reserved packs                   (CAR-8)
        * floor-pressure conditions                             (CAR-9)

    and regenerates the Appendix F tables (--emit-md).

WHAT THIS IS NOT
    * It is NOT the original generator. SSD Appendix F.0 records that the
      original generator was never published and its output was never
      regenerated at v3.1. This file is an independent re-implementation
      built from the formulas and inputs stated in the SSD text.
    * It does not verify anything physically. Every input is a design
      baseline ([?R] / [SPEC]) until verified under SSD section 13.
    * It models the STD/pack configuration space only. The tracked variant
      (TRK, HMX-1.4-08) is a separate build and is not modelled.

SELF-TEST
    By default the script recomputes the rows of SSD F.4 / F.4b and compares
    them with the values printed in the SSD. Any row that does not reproduce
    is reported as DIFF. A DIFF is information about the SSD or this model,
    not an error to be hidden: resolve it, then re-run (SSD section 0).

USAGE
    python3 hmx_config_check.py                    # self-test + release stamp
    python3 hmx_config_check.py --all              # all 128 pack combinations
    python3 hmx_config_check.py --build STAB,PAD   # one build
    python3 hmx_config_check.py --build STAB,PAD,UHTH,ODG --l1 --floor-kpa 70
    python3 hmx_config_check.py --sweep 6000       # Monte Carlo (F.7), seed 7
    python3 hmx_config_check.py --combined STD 100 0.4 0.2   # CAR-4 sum of ratios
    python3 hmx_config_check.py --emit-md appendix_f.md      # write F tables
    python3 hmx_config_check.py --json report.json           # machine-readable
    python3 hmx_config_check.py --strict           # exit 1 if self-test has DIFFs

Python 3.8+, standard library only.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import itertools
import json
import math
import pathlib
import platform
import random
import sys

SSD_VERSION = "v3.1"
SSD_DATE = "2026-11-14"
G = 9.81

# --------------------------------------------------------------------------
# Inputs (each is an SSD value; the record that owns it is noted)
# --------------------------------------------------------------------------
C = {
    "gvwr_kg": 1000.0,        # HMX-1.8-04 (P-02)
    "budget_kg": 700.0,       # section 3.1 (P-03)
    "op_mass_kg": 100.0,      # HMX-8.1-07 (P-01), dressed mass floor
    "op_z_m": 1.00,           # Appendix B.3 / F.1
    "op_x_m": 0.00,
    "payload_z_m": 1.10,      # F.1
    "brief_cap_kg": 200.0,    # HMX-1.8-04
    "cont_cap_kg": 100.0,     # HMX-1.8-04
    "min_brief_kg": 100.0,    # CAR-2
    "reach_cap_m": 1.0,       # HMX-3.3-02
    "half_foot_m": 0.20,      # 0.40 m foot, HMX-3.2-02
    "half_width_m": 0.1875,   # 0.375 m foot width
    "cop_margin_m": 0.065,    # HMX-3.2-02
    "slope_op_deg": 3.0,      # HMX-3.2-02
    "slope_test_deg": 5.0,    # HMX-3.2-09
    "stab_reach_m": 0.17,     # HMX-3.2-03
    "knee_cyl_nm": 1040.0,    # half of 2.08 kN.m peak, HMX-5.1-02
    "knee_lever_m": 0.15,     # HMX-5.1-02
    "psi": 1.15,              # HMX-3.2-10
    "knee_margin_min": 1.20,  # CAR-3
    "car5_limit_mm": 20.0,    # CAR-5
    "safe_vol_l": 3.74,       # HMX-4.3-02 (10 L accumulator usable volume)
    "safe_joint_l": 2.04,     # CAR-7 joint term
    "safe_brake_l": 0.40,     # CAR-7
    "safe_stab_l": 0.39,      # CAR-7, with OP-STAB
    "safe_margin_min": 1.25,  # CAR-7
    "foot_area_m2": 0.15,     # HMX-3.2-01
    "pad_area_m2": 0.60,      # per foot, OP-PAD deployed
    "stab_foot_m2": 0.068,    # 0.17 x 0.40 m
    "dyn_factor": 1.25,       # HMX-14-02
    "code_floor_kpa": 23.0,   # ASCE 7 basis, HMX-3.2-01
    "env_stowed_m": [1.83, 0.99, 0.74],   # CAR-6
    "odg_height_m": 1.95,     # CAR-6
    "stab_depth_m": 0.74,     # 0.40 + 2 x 0.17
    "swing_stab_dyn_note": "stabilizer foot pressure from 5 deg reaction x 1.25",
}

# (name, mass_lo, mass_hi, z, x)   -- Appendix F.1; x is forward of foot centre
STD_LINES = [
    ("Structure: legs",                 140, 140, 0.42,  0.00),
    ("Structure: pelvis and torso",     110, 110, 1.05,  0.00),
    ("Structure: arms and shoulders",    80,  80, 1.30,  0.10),
    ("Structure growth (HMX-3.1-03)",     0,  43, 0.95,  0.00),
    ("Actuators: legs",                  55,  55, 0.50,  0.00),
    ("Actuators: torso",                 20,  20, 1.05,  0.00),
    ("Actuators: arms",                  45,  45, 1.25,  0.10),
    ("Accumulator, manifolds, fluid",    50,  50, 1.05, -0.12),
    ("Sensors and controllers",          40,  40, 1.20, -0.12),
    ("Cockpit, restraint, air, MEER",    50,  50, 1.10,  0.00),
    ("Decontamination covers",           25,  25, 1.00,  0.00),
    ("Electronics-module lead",          25,  25, 1.20, -0.12),
    ("Tether fuse module",                3,   5, 0.30, -0.20),
]

# name: (mass_lo, mass_hi, z, x, orderable, release gate)      -- Appendix F.3
PACKS = {
    "STAB":   (27.0, 50.0, 0.30,  0.10, True,  "certified floor 66 kPa dynamic, spreader mat, or TRK"),
    "PAD":    (15.0, 30.0, 0.03,  0.00, True,  "site acceptance (HMX-14-02)"),
    "UHTH":   (6.0,  11.0, 1.15,  0.30, True,  "none open"),
    "ODG":    (13.0, 17.0, 1.90,  0.00, True,  "site acceptance (HMX-14-02); stature and route (HMX-3.3-01)"),
    "VNRA-H": (12.0, 25.0, 0.90, -0.05, True,  "transient test (HMX-13.2-12)"),
    "VNRA-F": (0.5,  1.0,  0.90,  0.00, True,  "radiation qualification (HMX-3.5-04)"),
    "VNRA-I": (0.3,  0.8,  1.00,  0.10, True,  "adoption gate (HMX-6.6-03)"),
    "NEUT":   (80.0, 96.0, 1.00,  0.00, False, "RESERVED, not orderable (HMX-1.8-02, HMX-7.3-01)"),
}
ORDERABLE = [k for k, v in PACKS.items() if v[4]]

# Mass action L1: composite accumulator shell removes up to 15 kg from the accumulator line
MASS_ACTIONS = {
    "L1": ("Composite accumulator shell (HMX-3.1-04)", -15.0, -15.0, 1.05, -0.12),
}

# Named trims (HMX-1.8-07)
TRIMS = [
    ("STD",                           (), ()),
    ("MIN-DEPLOY",                    ("PAD",), ()),
    ("HANDLING",                      ("STAB", "PAD"), ()),
    ("TOOLING",                       ("STAB", "PAD", "UHTH"), ()),
    ("PROTECTED",                     ("STAB", "PAD", "ODG"), ()),
    ("PROTECTED-TOOLING",             ("STAB", "PAD", "UHTH", "ODG"), ()),
    ("FULL",                          tuple(PACKS), ()),
    ("PROTECTED-TOOLING + L1",        ("STAB", "PAD", "UHTH", "ODG"), ("L1",)),
]

# Values printed in the SSD (F.4 and F.4b) for the self-test.
# off_b / off_c: no-stabilizer offset limits, brief / continuous, m (None = at or above the 1.0 m reach)
SSD_REF = {
    "STD":                    dict(dry=(643, 688), brief=200, cont=100, off_b=0.38, off_c=0.69, z=0.970, x=2.9, knee=1.24, ok=True),
    "MIN-DEPLOY":             dict(dry=(658, 718), brief=182, cont=91,  off_b=0.43, off_c=0.77, z=0.951, x=2.8, knee=1.23, ok=True),
    "HANDLING":               dict(dry=(685, 768), brief=132, cont=66,  off_b=0.58, off_c=None, z=0.925, x=3.1, knee=1.23, ok=True),
    "TOOLING":                dict(dry=(691, 779), brief=121, cont=60,  off_b=0.60, off_c=None, z=0.924, x=6.8, knee=1.23, ok=True),
    "PROTECTED":              dict(dry=(698, 785), brief=115, cont=58,  off_b=0.66, off_c=None, z=0.936, x=3.1, knee=1.23, ok=True),
    "PROTECTED-TOOLING":      dict(dry=(704, 796), brief=104, cont=52,  off_b=0.69, off_c=None, z=0.935, x=6.7, knee=1.23, ok=True),
    "FULL":                   dict(dry=(797, 919), brief=0,   cont=0,   off_b=None, off_c=None, z=0.921, x=4.7, knee=1.21, ok=False),
    "PROTECTED-TOOLING + L1": dict(dry=(689, 781), brief=119, cont=60,  off_b=0.59, off_c=None, z=0.936, x=8.9, knee=1.23, ok=True),
}
SSD_COUNTS = {"orderable_combinations": 128, "admissible_no_L1": 116, "admissible_with_L1": 124}


# --------------------------------------------------------------------------
# Release stamp (SSD section 0, Appendix F.0, EXT-30)
# --------------------------------------------------------------------------
def script_hash() -> str:
    """SHA-256 of this file's bytes (the 'version hash' of SSD section 0)."""
    try:
        return hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
    except OSError:
        return "unavailable"


def model_hash() -> str:
    """SHA-256 of the model inputs only (changes when an input changes, not when comments do)."""
    blob = json.dumps(
        {"C": C, "STD": STD_LINES, "PACKS": PACKS, "ACTIONS": MASS_ACTIONS, "TRIMS": TRIMS},
        sort_keys=True, default=list,
    )
    return hashlib.sha256(blob.encode()).hexdigest()


def stamp() -> dict:
    return {
        "script": "hmx_config_check.py",
        "ssd_version": SSD_VERSION,
        "ssd_date": SSD_DATE,
        "script_sha256": script_hash(),
        "model_inputs_sha256": model_hash(),
        "run_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "python": platform.python_version(),
    }


# --------------------------------------------------------------------------
# Core model
# --------------------------------------------------------------------------
def machine_components(packs, actions, bound):
    """List of (mass, z, x) for the machine at bound 0 (low) or 1 (high)."""
    comps = [(ln[1 + bound], ln[3], ln[4]) for ln in STD_LINES]
    for p in packs:
        lo, hi, z, x, _, _ = PACKS[p]
        comps.append(((lo, hi)[bound], z, x))
    for a in actions:
        _, lo, hi, z, x = MASS_ACTIONS[a]
        comps.append(((lo, hi)[bound], z, x))
    return comps


def com(comps):
    m = sum(c[0] for c in comps)
    return m, sum(c[0] * c[1] for c in comps) / m, sum(c[0] * c[2] for c in comps) / m


def tan_deg(d):
    return math.tan(math.radians(d))


def offset_limit(payload, h, x0, m0, stab=False, lateral=False, cap=None):
    """Offset limit (m) from m*d <= dx*(M0+m), CAR-4. Capped at the 1.0 m reach."""
    cap = C["reach_cap_m"] if cap is None else cap
    if payload <= 0:
        return cap
    if lateral:
        dx = C["half_width_m"] - C["cop_margin_m"] - h * tan_deg(C["slope_op_deg"])
    else:
        dx = (C["half_foot_m"] + (C["stab_reach_m"] if stab else 0.0)
              - C["cop_margin_m"] - h * tan_deg(C["slope_op_deg"]) - x0)
    if dx <= 0:
        return 0.0
    return min(cap, dx * (m0 + payload) / payload)


def evaluate(packs=(), actions=(), name=None, op_mass=None, floor_kpa=None, mat=False):
    """Evaluate one build against CAR-1 to CAR-9. Returns a flat dict."""
    packs = tuple(packs)
    actions = tuple(actions)
    op = C["op_mass_kg"] if op_mass is None else max(C["op_mass_kg"], op_mass)
    stab = "STAB" in packs
    pad = "PAD" in packs
    r = {"name": name or ("STD" if not packs else "+".join(packs)) + (" + " + "+".join(actions) if actions else ""),
         "packs": list(packs), "actions": list(actions), "conditions": [], "fails": []}

    lo_c = machine_components(packs, actions, 0)
    hi_c = machine_components(packs, actions, 1)
    m_lo, _, _ = com(lo_c)
    m_hi, _, _ = com(hi_c)
    r["dry_lo"], r["dry_hi"] = m_lo, m_hi

    # CAR-1: rating by the GVWR rule
    brief = max(0.0, min(C["brief_cap_kg"], C["gvwr_kg"] - op - m_hi))
    cont = min(C["cont_cap_kg"], brief / 2.0)
    r["brief"], r["cont"] = brief, cont
    gross = m_hi + op + brief
    r["gross"] = gross
    r["headroom_hi"] = C["budget_kg"] - m_hi
    r["headroom_lo"] = C["budget_kg"] - m_lo

    # CoM: unloaded offset and loaded height at both mass bounds, worst case kept
    zs, xs = [], []
    for comps in (lo_c, hi_c):
        parts = comps + [(op, C["op_z_m"], C["op_x_m"])]
        _, _, x_unl = com(parts)
        _, z_ld, _ = com(parts + [(brief, C["payload_z_m"], 0.0)] if brief > 0 else parts)
        xs.append(abs(x_unl))
        zs.append(z_ld)
    h, x0 = max(zs), max(xs)
    m0 = m_lo + op
    r["h"], r["x0_mm"], r["m0"] = h, x0 * 1000.0, m0

    # Offset limits (no stabilizers, and with stabilizers)
    r["off_b"] = offset_limit(brief, h, x0, m0)
    r["off_c"] = offset_limit(cont, h, x0, m0)
    r["off_b_stab"] = offset_limit(brief, h, x0, m0, stab=True)
    r["off_c_stab"] = offset_limit(cont, h, x0, m0, stab=True)
    r["lat_c"] = offset_limit(cont, h, x0, m0, lateral=True)

    # CAR-2
    if brief < C["min_brief_kg"]:
        r["fails"].append("CAR-2: rated brief %.0f kg is under the %.0f kg minimum" % (brief, C["min_brief_kg"]))

    # CAR-3: knee margin at the build's own gross, and single-support lever cap
    knee = C["knee_cyl_nm"] / (0.5 * gross * G * C["knee_lever_m"] * C["psi"])
    r["knee_margin"] = knee
    r["knee_lever_cap_mm"] = 1000.0 * C["knee_cyl_nm"] / (C["knee_margin_min"] * C["psi"] * gross * G)
    if knee < C["knee_margin_min"]:
        r["fails"].append("CAR-3: knee margin %.2f is under %.2f" % (knee, C["knee_margin_min"]))

    # Joint bases (F.6)
    ankle = C["psi"] * gross * G * (C["half_foot_m"] - C["cop_margin_m"]) / 1000.0
    r["ankle_kNm"], r["ankle_kNm_x12"] = ankle, ankle * 1.2
    ob = r["off_b_stab"] if stab else r["off_b"]
    oc = r["off_c_stab"] if stab else r["off_c"]
    r["shoulder_kNm"] = max((brief / 2.0) * G * ob, cont * G * oc) / 1000.0

    # CAR-4: with OP-STAB, CoP margin >= 65 mm at the 5 deg test slope, brief payload at 1.0 m
    if stab:
        shift = brief * 1.0 / (m0 + brief) if brief > 0 else 0.0
        margin5 = (C["half_foot_m"] + C["stab_reach_m"]) - shift - h * tan_deg(C["slope_test_deg"]) - x0
        r["cop_margin5_mm"] = margin5 * 1000.0
        if margin5 < C["cop_margin_m"]:
            r["fails"].append("CAR-4: CoP margin %.0f mm at 5 deg is under 65 mm" % (margin5 * 1000))
        # stabilizer reaction (B.3): per forward stabilizer, feet carrying to the toe line
        react = gross * G * max(0.0, shift + h * tan_deg(C["slope_test_deg"]) + x0
                                + C["cop_margin_m"] - C["half_foot_m"]) / C["stab_reach_m"] / 2.0
        r["stab_react_kN"] = react / 1000.0
        r["stab_foot_kpa_static"] = react / C["stab_foot_m2"] / 1000.0
        r["stab_foot_kpa_dyn"] = r["stab_foot_kpa_static"] * C["dyn_factor"]
    else:
        r["cop_margin5_mm"] = None

    # CAR-5
    if r["x0_mm"] > C["car5_limit_mm"]:
        r["fails"].append("CAR-5: unloaded CoM offset %.1f mm exceeds 20 mm" % r["x0_mm"])

    # CAR-6: stowed envelope (OP-ODG is the only pack that exceeds the height target)
    height = C["odg_height_m"] if "ODG" in packs else C["env_stowed_m"][0]
    depth = C["stab_depth_m"] if stab else C["env_stowed_m"][2]
    r["height_m"], r["deployed_depth_m"] = height, depth
    if depth > C["env_stowed_m"][2] + 1e-9:
        r["fails"].append("CAR-6: deployed depth %.2f m exceeds 0.74 m" % depth)
    if "ODG" in packs:
        r["conditions"].append("CAR-6: height 1.95 m only with the rated stature 1.54-1.64 m and route openings >= 2.05 m (HMX-3.3-01, HMX-8.1-07)")

    # CAR-7: safe-posture hydraulic margin
    demand = C["safe_joint_l"] + C["safe_brake_l"] + (C["safe_stab_l"] if stab else 0.0)
    r["safe_margin"] = C["safe_vol_l"] / demand
    if r["safe_margin"] < C["safe_margin_min"]:
        r["fails"].append("CAR-7: safe-posture margin %.2f is under 1.25" % r["safe_margin"])

    # CAR-8: reserved packs and release gates
    for p in packs:
        if not PACKS[p][4]:
            r["fails"].append("CAR-8: %s is reserved and not orderable" % p)
        elif PACKS[p][5] != "none open":
            r["conditions"].append("CAR-8 gate, %s: %s" % (p, PACKS[p][5]))

    # CAR-9: floor (site-dependent; judged only if --floor-kpa is given)
    bare = C["dyn_factor"] * C["gvwr_kg"] * G / C["foot_area_m2"] / 1000.0
    padp = C["dyn_factor"] * C["gvwr_kg"] * G / C["pad_area_m2"] / 1000.0
    r["bare_foot_kpa_dyn"], r["pad_kpa_dyn"] = bare, padp
    if floor_kpa is None:
        r["conditions"].append("CAR-9: floor not stated. Bare foot needs a floor certified >= %.0f kPa dynamic%s%s"
                               % (bare, "; pads give %.0f kPa" % padp if pad else "; otherwise OP-PAD",
                                  "; stabilizer feet need >= %.0f kPa, a mat, or TRK" % r["stab_foot_kpa_dyn"] if stab else ""))
    else:
        need = padp if pad else bare
        if floor_kpa < need:
            r["fails"].append("CAR-9: floor %.0f kPa is under the %.0f kPa needed under the feet%s"
                              % (floor_kpa, need, "" if pad else " (fit OP-PAD)"))
        if stab and not mat and floor_kpa < r["stab_foot_kpa_dyn"]:
            r["fails"].append("CAR-9: floor %.0f kPa is under the %.0f kPa needed under the stabilizer feet (mat or TRK)"
                              % (floor_kpa, r["stab_foot_kpa_dyn"]))

    r["admissible"] = not r["fails"]
    return r


def combined_ratio(r, payload, dx, dy):
    """CAR-4 sum-of-ratios for a payload at fore-aft dx and lateral dy (no stabilizer credit)."""
    lx = (C["half_foot_m"] - C["cop_margin_m"] - r["h"] * tan_deg(C["slope_op_deg"]) - r["x0_mm"] / 1000.0) * (r["m0"] + payload)
    ly = (C["half_width_m"] - C["cop_margin_m"] - r["h"] * tan_deg(C["slope_op_deg"])) * (r["m0"] + payload)
    return payload * dx / lx + payload * dy / ly, lx, ly


def enumerate_all(actions=()):
    out = []
    for n in range(len(ORDERABLE) + 1):
        for combo in itertools.combinations(ORDERABLE, n):
            out.append(evaluate(combo, actions))
    return out


# --------------------------------------------------------------------------
# Sensitivity sweep (F.7). Same recipe as Appendix B.3; the original RNG is not
# known, so percentiles agree statistically, not digit for digit.
# --------------------------------------------------------------------------
def sweep(packs=(), actions=(), n=6000, seed=7):
    rng = random.Random(seed)
    res = {"dry": [], "x0_mm": [], "off_c": [], "off_b": []}
    base = ([(ln[1], ln[2], ln[3], ln[4]) for ln in STD_LINES]
            + [(PACKS[p][0], PACKS[p][1], PACKS[p][2], PACKS[p][3]) for p in packs]
            + [(MASS_ACTIONS[a][1], MASS_ACTIONS[a][2], MASS_ACTIONS[a][3], MASS_ACTIONS[a][4]) for a in actions])
    for _ in range(n):
        comps = []
        for lo, hi, z, x in base:
            m = lo if lo == hi else rng.uniform(lo, hi)
            comps.append((m, z + rng.uniform(-0.10, 0.10), x + rng.uniform(-0.05, 0.05)))
        dry = sum(c[0] for c in comps)
        op = (C["op_mass_kg"], C["op_z_m"] + rng.uniform(-0.08, 0.08), C["op_x_m"] + rng.uniform(-0.03, 0.03))
        brief = max(0.0, min(C["brief_cap_kg"], C["gvwr_kg"] - op[0] - dry))
        cont = min(C["cont_cap_kg"], brief / 2.0)
        _, _, x_unl = com(comps + [op])
        _, h, _ = com(comps + [op] + ([(brief, C["payload_z_m"], 0.0)] if brief > 0 else []))
        m0 = dry + op[0]
        res["dry"].append(dry)
        res["x0_mm"].append(abs(x_unl) * 1000.0)
        res["off_b"].append(offset_limit(brief, h, abs(x_unl), m0))
        res["off_c"].append(offset_limit(cont, h, abs(x_unl), m0))
    return res


def pct(vals, q):
    s = sorted(vals)
    return s[min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))]


# --------------------------------------------------------------------------
# Self-test against the SSD's printed tables
# --------------------------------------------------------------------------
def selftest():
    rows, diffs = [], 0
    for name, packs, actions in TRIMS:
        r = evaluate(packs, actions, name=name)
        ref = SSD_REF[name]
        checks = []

        def chk(label, got, want, tol, fmt="%.3f"):
            ok = (want is None and got is None) or (want is not None and got is not None and abs(got - want) <= tol)
            checks.append((label, ok, fmt % got if got is not None else "n/a", fmt % want if want is not None else "n/a"))

        chk("dry lo", r["dry_lo"], ref["dry"][0], 0.6, "%.1f")
        chk("dry hi", r["dry_hi"], ref["dry"][1], 0.6, "%.1f")
        chk("brief", r["brief"], ref["brief"], 0.6, "%.1f")
        chk("cont", r["cont"], ref["cont"], 0.55, "%.1f")
        if ref["off_b"] is not None:
            chk("offset brief", r["off_b"], ref["off_b"], 0.006, "%.3f")
        if ref["off_c"] is not None:
            chk("offset cont", r["off_c"], ref["off_c"], 0.006, "%.3f")
        elif r["cont"] > 0:
            checks.append(("offset cont >=1.0", r["off_c"] >= 0.999, "%.3f" % r["off_c"], ">=1.0"))
        chk("loaded CoM z", r["h"], ref["z"], 0.003, "%.4f")
        chk("CoM offset mm", r["x0_mm"], ref["x"], 0.15, "%.2f")
        chk("knee margin", r["knee_margin"], ref["knee"], 0.006, "%.3f")
        checks.append(("result", r["admissible"] == ref["ok"],
                       "pass" if r["admissible"] else "FAIL", "pass" if ref["ok"] else "FAIL"))
        nd = sum(1 for c in checks if not c[1])
        diffs += nd
        rows.append((name, checks, nd))

    allr = enumerate_all()
    allr_l1 = enumerate_all(("L1",))
    counts = {
        "orderable_combinations": len(allr),
        "admissible_no_L1": sum(1 for x in allr if x["admissible"]),
        "admissible_with_L1": sum(1 for x in allr_l1 if x["admissible"]),
    }
    cdiff = [(k, counts[k], SSD_COUNTS[k]) for k in SSD_COUNTS if counts[k] != SSD_COUNTS[k]]
    diffs += len(cdiff)

    # headline derived values quoted in the SSD text
    std = evaluate((), name="STD")
    ex = [
        ("M0 for STD (kg)", std["m0"], 743.0, 0.5),
        ("STD safe-posture margin", std["safe_margin"], 1.53, 0.006),
        ("STD single-support knee lever cap, mm", std["knee_lever_cap_mm"], 78.0, 0.6),
    ]
    stab_only = evaluate(("STAB",), name="STAB")
    ex.append(("Max stabilizer reaction kN (STAB alone)", stab_only["stab_react_kN"], 3.6, 0.06))
    hand = evaluate(("STAB", "PAD"), name="HANDLING")
    ex.append(("HANDLING lateral limit, 100 kg class (m)", hand["lat_c"], 0.95, 0.006))
    ratio, lx, ly = combined_ratio(std, 100, 0.4, 0.2)
    ex += [("F.8 L_x STD at 100 kg (kg.m)", lx, 68.5, 0.3), ("F.8 L_y STD at 100 kg (kg.m)", ly, 60.4, 0.3),
           ("F.8 ratio at 0.4 m / 0.2 m", ratio, 0.91, 0.006)]
    ratio2, _, _ = combined_ratio(std, 100, 0.5, 0.3)
    ex.append(("F.8 ratio at 0.5 m / 0.3 m", ratio2, 1.23, 0.006))
    exrows = []
    for label, got, want, tol in ex:
        ok = abs(got - want) <= tol
        diffs += 0 if ok else 1
        exrows.append((label, ok, got, want))
    return rows, counts, cdiff, exrows, diffs


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------
def fmt_off(v):
    return ">= 1.0 m (reach)" if v >= 0.999 else "%.2f m" % v


def print_stamp(s):
    print("HARDIMAN-X configuration check  |  SSD %s (%s)" % (s["ssd_version"], s["ssd_date"]))
    print("  script sha256 : %s" % s["script_sha256"])
    print("  model inputs  : %s" % s["model_inputs_sha256"])
    print("  run (UTC)     : %s   python %s" % (s["run_utc"], s["python"]))
    print("  Publish the script hash and run date with each SSD release (SSD section 0, Appendix F.0).")


def print_build(r):
    print("\n%s" % r["name"])
    print("  dry mass            %.0f to %.0f kg  (headroom to 700 kg: %.0f at upper bound, %.0f at lower)"
          % (r["dry_lo"], r["dry_hi"], r["headroom_hi"], r["headroom_lo"]))
    print("  rated brief / cont  %.0f / %.1f kg   gross at rating %.0f kg" % (r["brief"], r["cont"], r["gross"]))
    print("  offsets, no stab    brief %s   cont %s   lateral (cont) %s"
          % (fmt_off(r["off_b"]), fmt_off(r["off_c"]), fmt_off(r["lat_c"])))
    if "STAB" in r["packs"]:
        print("  offsets, stab       brief %s   cont %s   CoP margin at 5 deg %.0f mm"
              % (fmt_off(r["off_b_stab"]), fmt_off(r["off_c_stab"]), r["cop_margin5_mm"]))
        print("  stabilizer feet     %.1f kN reaction, %.0f kPa static, %.0f kPa dynamic"
              % (r["stab_react_kN"], r["stab_foot_kpa_static"], r["stab_foot_kpa_dyn"]))
    print("  loaded CoM z        %.3f m   unloaded fore-aft offset %.1f mm" % (r["h"], r["x0_mm"]))
    print("  knee margin         %.2f   single-support lever cap %.0f mm" % (r["knee_margin"], r["knee_lever_cap_mm"]))
    print("  ankle basis         %.2f kN.m (x1.2 = %.2f)   shoulder demand per arm %.2f kN.m"
          % (r["ankle_kNm"], r["ankle_kNm_x12"], r["shoulder_kNm"]))
    print("  safe-posture margin %.2f" % r["safe_margin"])
    print("  RESULT              %s" % ("pass" if r["admissible"] else "FAIL"))
    for f in r["fails"]:
        print("    FAIL: %s" % f)
    for c in r["conditions"]:
        print("    condition: %s" % c)


def md_tables():
    out = []
    out.append("### F.1 (generated) Standard build mass and CoM model\n")
    out.append("| Component | Mass kg | z m | x m |\n|---|---|---|---|")
    for n, lo, hi, z, x in STD_LINES:
        out.append("| %s | %s | %.2f | %+.2f |" % (n, ("%g" % lo) if lo == hi else "%g to %g" % (lo, hi), z, x))
    s = evaluate((), name="STD")
    out.append("| **STD total** | **%.0f to %.0f** | %.3f (loaded, tallest bound) | %+.1f mm |\n" % (s["dry_lo"], s["dry_hi"], s["h"], s["x0_mm"]))

    out.append("### F.3 (generated) Option packs\n")
    out.append("| Pack | Mass kg | CoM z / x m | Brief rating alone kg | Orderable | Gate |\n|---|---|---|---|---|---|")
    for p, (lo, hi, z, x, o, g) in PACKS.items():
        e = evaluate((p,), name=p)
        out.append("| OP-%s | %g to %g | %.2f / %+.2f | %.0f | %s | %s |" % (p, lo, hi, z, x, e["brief"], "yes" if o else "no (reserved)", g))
    out.append("")

    out.append("### F.4 (generated) Trims and configuration matrix\n")
    out.append("| Build | Dry mass kg | Brief / cont kg | Offset no stab, brief / cont | Offset stab, brief / cont | Loaded CoM z m | CoM offset mm | Knee margin | Result |\n|---|---|---|---|---|---|---|---|---|")
    for name, packs, actions in TRIMS:
        r = evaluate(packs, actions, name=name)
        stabtxt = ("%s / %s" % (fmt_off(r["off_b_stab"]), fmt_off(r["off_c_stab"]))) if "STAB" in packs else "n/a / n/a"
        out.append("| %s | %.0f to %.0f | %.0f / %.0f | %s / %s | %s | %.3f | %.1f | %.2f | %s |" % (
            name, r["dry_lo"], r["dry_hi"], r["brief"], r["cont"], fmt_off(r["off_b"]), fmt_off(r["off_c"]),
            stabtxt, r["h"], r["x0_mm"], r["knee_margin"], "pass" if r["admissible"] else "FAIL"))
    out.append("\nFULL is shown only to bound the space. Floor conditions (CAR-9) are site-dependent and not judged here.\n")

    out.append("### F.6 (generated) Joint torque bases by build\n")
    out.append("| Build | Knee lever cap mm | Ankle basis kN.m (x1.2) | Shoulder per arm kN.m | Lateral limit, cont. class m |\n|---|---|---|---|---|")
    for name, packs, actions in TRIMS[:6]:
        r = evaluate(packs, actions, name=name)
        out.append("| %s | %.0f | %.2f (%.2f) | %.2f | %s |" % (name, r["knee_lever_cap_mm"], r["ankle_kNm"], r["ankle_kNm_x12"], r["shoulder_kNm"], fmt_off(r["lat_c"])))
    out.append("")
    
    out.append("### F.7 (generated) Sensitivity sweep, 6,000 samples, seed 7 (Python random; not the original RNG)\n")
    out.append("| Build | Offset limit, cont. class, m (5th / 50th / 95th) | Offset limit, brief, m (5th / 50th / 95th) | CoM offset mm (50th / 95th), share over 20 mm |\n|---|---|---|---|")
    # Adjusted to exactly match the 4 rows in SSD F.7: STD, HANDLING, TOOLING, PROTECTED
    for name, packs, actions in TRIMS[0:1] + TRIMS[2:5]:
        sw = sweep(packs, actions)
        over = 100.0 * sum(1 for v in sw["x0_mm"] if v > C["car5_limit_mm"]) / len(sw["x0_mm"])
        out.append("| %s | %.2f / %.2f / %.2f | %.2f / %.2f / %.2f | %.1f / %.1f, %.1f%% |" % (
            name, pct(sw["off_c"], 5), pct(sw["off_c"], 50), pct(sw["off_c"], 95),
            pct(sw["off_b"], 5), pct(sw["off_b"], 50), pct(sw["off_b"], 95),
            pct(sw["x0_mm"], 50), pct(sw["x0_mm"], 95), over))
    out.append("")
    return "\n".join(out)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def parse_build(spec):
    if spec.upper() == "STD":
        return ()
    packs = []
    for tok in spec.split(","):
        t = tok.strip().upper().replace("OP-", "")
        if t not in PACKS:
            raise SystemExit("unknown pack '%s'. Packs: %s" % (tok, ", ".join(PACKS)))
        packs.append(t)
    return tuple(packs)


def main(argv=None):
    ap = argparse.ArgumentParser(description="HARDIMAN-X SSD v3.1 configuration checker (see module docstring).")
    ap.add_argument("--build", help="comma list of packs, e.g. STAB,PAD,ODG (or STD)")
    ap.add_argument("--l1", action="store_true", help="apply mass action L1 (composite accumulator shell)")
    ap.add_argument("--all", action="store_true", help="evaluate all 128 orderable pack combinations")
    ap.add_argument("--sweep", type=int, metavar="N", help="Monte Carlo sensitivity sweep for --build (default STD), seed 7")
    ap.add_argument("--combined", nargs=4, metavar=("BUILD", "KG", "DX", "DY"), help="CAR-4 sum of ratios for a payload at fore-aft DX and lateral DY (m)")
    ap.add_argument("--floor-kpa", type=float, help="certified floor capacity, dynamic kPa (enables CAR-9 pass/fail)")
    ap.add_argument("--mat", action="store_true", help="a spreader mat is used under the stabilizer feet (CAR-9)")
    ap.add_argument("--operator-kg", type=float, help="operator dressed mass; the larger of this and 100 kg is used (HMX-1.8-04)")
    ap.add_argument("--emit-md", metavar="FILE", help="write regenerated Appendix F tables to FILE ('-' for stdout)")
    ap.add_argument("--json", metavar="FILE", help="write a machine-readable report to FILE")
    ap.add_argument("--strict", action="store_true", help="exit 1 if the self-test finds any DIFF")
    ap.add_argument("--no-selftest", action="store_true", help="skip the self-test")
    a = ap.parse_args(argv)

    s = stamp()
    print_stamp(s)
    report = {"stamp": s}
    actions = ("L1",) if a.l1 else ()
    diffs = 0

    if not a.no_selftest:
        rows, counts, cdiff, exrows, diffs = selftest()
        print("\nSELF-TEST: recomputed rows vs the values printed in SSD F.4 / F.4b / F.8")
        for name, checks, nd in rows:
            print("  %-26s %s" % (name, "all match" if nd == 0 else "%d DIFF" % nd))
            for label, ok, got, want in checks:
                if not ok:
                    print("      DIFF %-18s model %-9s SSD %s" % (label, got, want))
        print("  pack combinations: %d orderable, %d admissible without L1 (SSD %d), %d with L1 (SSD %d)"
              % (counts["orderable_combinations"], counts["admissible_no_L1"], SSD_COUNTS["admissible_no_L1"],
                 counts["admissible_with_L1"], SSD_COUNTS["admissible_with_L1"]))
        for label, ok, got, want in exrows:
            if not ok:
                print("      DIFF %-42s model %.3f SSD %.3f" % (label, got, want))
        print("  self-test total DIFFs: %d" % diffs)
        print("  Note: matching the SSD shows this model reproduces the SSD's arithmetic. It does not verify any input.")
        report["selftest"] = {"diffs": diffs, "counts": counts}

    if a.build is not None:
        r = evaluate(parse_build(a.build), actions, op_mass=a.operator_kg, floor_kpa=a.floor_kpa, mat=a.mat)
        print_build(r)
        report["build"] = r

    if a.all:
        allr = enumerate_all(actions)
        good = [x for x in allr if x["admissible"]]
        print("\nALL COMBINATIONS (%s): %d of %d admissible (CAR-1 to CAR-8; floor is site-dependent)"
              % ("with L1" if actions else "no mass action", len(good), len(allr)))
        for x in allr:
            if not x["admissible"]:
                print("  FAIL  %-28s brief %3.0f kg   %s" % ("+".join(x["packs"]) or "STD", x["brief"], x["fails"][0]))
        mx = max((x.get("stab_react_kN", 0.0) for x in allr), default=0.0)
        mo = max(x["x0_mm"] for x in allr)
        print("  max unloaded CoM offset over all combinations: %.1f mm (limit 20)" % mo)
        print("  max stabilizer reaction over all combinations: %.1f kN static" % mx)
        report["all"] = {"admissible": len(good), "total": len(allr)}

    if a.sweep:
        packs = parse_build(a.build) if a.build else ()
        sw = sweep(packs, actions, n=a.sweep)
        print("\nSENSITIVITY SWEEP: %d samples, seed 7 (Python random; the SSD's generator and RNG are unpublished)" % a.sweep)
        print("  dry mass kg          5th/50th/95th  %.0f / %.0f / %.0f" % (pct(sw["dry"], 5), pct(sw["dry"], 50), pct(sw["dry"], 95)))
        print("  CoM offset mm        50th/95th      %.1f / %.1f   share over 20 mm: %.1f%%"
              % (pct(sw["x0_mm"], 50), pct(sw["x0_mm"], 95), 100.0 * sum(v > 20 for v in sw["x0_mm"]) / len(sw["x0_mm"])))
        print("  offset limit, cont.  5th/50th/95th  %.2f / %.2f / %.2f m" % (pct(sw["off_c"], 5), pct(sw["off_c"], 50), pct(sw["off_c"], 95)))
        print("  offset limit, brief  5th/50th/95th  %.2f / %.2f / %.2f m" % (pct(sw["off_b"], 5), pct(sw["off_b"], 50), pct(sw["off_b"], 95)))
        print("  Before the first weighing, quote offset ratings at the 5th-percentile values (HMX-1.8-04).")

    if a.combined:
        packs = parse_build(a.combined[0])
        r = evaluate(packs, actions)
        kg, dx, dy = float(a.combined[1]), float(a.combined[2]), float(a.combined[3])
        ratio, lx, ly = combined_ratio(r, kg, dx, dy)
        print("\nCAR-4 combined offsets, %s, %.0f kg at %.2f m fore-aft and %.2f m lateral" % (r["name"], kg, dx, dy))
        print("  L_x = %.1f kg.m, L_y = %.1f kg.m, ratio = %.2f -> %s" % (lx, ly, ratio, "allowed" if ratio <= 1 else "NOT allowed"))

    if a.emit_md:
        text = md_tables()
        header = "<!-- generated by hmx_config_check.py | script sha256 %s | %s -->\n\n" % (s["script_sha256"][:16], s["run_utc"])
        if a.emit_md == "-":
            print("\n" + header + text)
        else:
            pathlib.Path(a.emit_md).write_text(header + text, encoding="utf-8")
            print("\nWrote Appendix F tables to %s" % a.emit_md)

    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(report, indent=2, default=list), encoding="utf-8")
        print("Wrote %s" % a.json)

    if a.strict and diffs:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
