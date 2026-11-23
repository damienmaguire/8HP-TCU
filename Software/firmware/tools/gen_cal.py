#!/usr/bin/env python3
#
# This file is part of the 8HP-TCU project.
#
# Copyright (C) 2026 Damien Maguire <info@evbmw.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
"""
Generate the C calibration tables the TCU firmware needs.

Sources (Ian's exports; none of them is committed to the repo):
  --q030  330e_Q030_cal_full.json: all 15,436 A2L characteristics of
          Damien's 330e flash, EPK = the A2L build, so the labels are exact
          [C]. Shift, gear-in, line pressure, global values, the 330e EDS
          curves, the per-clutch fill data and the OEM Sport tables come
          from here.
  --f11   f11_sas_eds_cal.json: the F11 v058 solenoid curve block at
          0x19FCA8 (seven p->I curves A..E, SYS, TCC).
  --f11-psys f11_psys_gearin_cal.json (Ian, F11_SAM_ADDSCOPE.md 2): F11
          line pressure KF_PSYS_R..8, KF_PSYSN, KL_TMAXPSYS (not used), the
          KF_KIEDSCG* EDS temperature corrections and the R / 1st gear-in
          pressures. Other F11 fields fall back to the 330e, marked.
  --f11-port f11_a81v058_cal_ported.json: only its [C] rows are used
          (KL_TVPSEIN); [I] and [H] rows never drive outputs.
  --v095  directory with params_<cal>.json for the five a81 v095 cals
          (F11 N47 8HP45, N57 8HP70, N63 8HP70, N74 8HP90, RR4 N74 8HP90),
          all [I]. Used for the two v095 solenoid curve sets and the
          per-size reference data (torque class, reverse limit, safety
          offsets, fill / piston / hysteresis tables), never for the shift
          engine's own tables.

Outputs:
  cal_330e.{h,cpp}   Normal and Sport shift / fill tables, gear-in, line
                     pressure, globals, 330e EDS curves (all Q030 [C]).
  cal_f11.{h,cpp}    the F11 v058 EDS block.
  cal_v095.{h,cpp}   v095 EDS sets and the per-cal size / reference data,
                     plus the 330e reference data from Q030.

Only the items the shift engine uses are emitted for it: the
pressure-to-current curves, the per-clutch fill data, line pressure,
N->1/N->R gear-in data, and the shift data for the seven adjacent
upshifts (12..78) and seven adjacent downshifts (21..87). Every table
carries its source name and flash address in a comment.

Sport set: for every shift / fill item the OEM Sport label (the A2L name
with SPORT inserted before the shift pair or clutch suffix, e.g.
KF_TFCGNTSPORT12, KF_TOSFHSSPORT_A) is used where Q030 has one, the Normal
label otherwise (the same fall-back the OEM SAS uses for items without a
Sport copy).

Units in the generated tables (all integers, the F107 has no FPU):
  pressure  mbar          (export unit bar x 1000)
  gradient  mbar/s        (export unit bar/s x 1000)
  current   0.1 mA        (export unit mA x 10)
  time      ms
  torque    Nm
  speed     rpm
  temp      degC
Map layout: z[iy * nx + ix], x = first A2L axis, y = second axis. This is
the orientation Ian checked in 330E_SAS_FILL_RAMP.md (rows = second axis).
Reference tables (cal_v095) are stored in milli-units of the export unit
(value x 1000), with the unit string, because the v095 axes are not all in
firmware units.

Usage:
  python3 tools/gen_cal.py --q030 <330e_Q030_cal_full.json> \
          --f11 <f11_sas_eds_cal.json> --v095 <cal_v095 dir> [--set normal|el]

--set el uses the "_EL" (electric-drive) copy of a table where one exists
and falls back to the normal table otherwise (open question Q7 in Ian's
spec). The default is the Normal programme set.
"""
import argparse
import re
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

UP_PAIRS = ["12", "23", "34", "45", "56", "67", "78"]
DOWN_PAIRS = ["21", "32", "43", "54", "65", "76", "87"]
CLUTCHES = "ABCDE"

# (field, A2L prefix, kind) kind: V value, C curve, M map
UP_FIELDS = [
    ("t_vkzu", "T_VKZU", "V"),
    ("tfcgnt", "KF_TFCGNT", "M"),
    ("pofmtn", "KF_POFMTN", "M"),
    ("tramtnt", "KF_TRAMTNT", "M"),
    ("pkzu", "KF_PKZU", "M"),
    ("pgokzumtnt", "KF_PGOKZUMTNT", "M"),
    ("tsynmtrakzu", "KL_TSYNMTRAKZU", "C"),
    ("pgrasyn", "KL_PGRASYN", "C"),
    ("porasyn", "KL_PORASYN", "C"),
    ("t_syn", "T_SYN", "V"),
    ("t_vkabhs", "T_VKABHS", "V"),
    ("phkabmt", "KL_PHKABMT", "C"),
    ("po_hkab", "PO_HKAB", "V"),
    ("to_hkab", "TO_HKAB", "V"),
    ("thkabmtcg", "KF_THKABMTCG", "M"),
    ("tpkabmtcg", "KF_TPKABMTCG", "M"),
    ("topkabnd", "KL_TOPKABND", "C"),
    ("pkab", "KF_PKAB", "M"),
    ("pgomtkab", "KL_PGOMTKAB", "C"),
    ("t_synkabe", "T_SYNKABE", "V"),
    ("ns_synist", "NS_SYNISTE", "V"),
    ("tmaxcg", "KL_TMAXCG", "C"),
    ("t_ramax", "T_RAMAX", "V"),
    ("p_sysmin", "P_SYSMIN", "V"),
    ("wtsgls", "KF_WTSGLS", "M"),
    ("kp_glskzu", "KP_GLSKZU", "V"),
    ("ki_glskzu", "KI_GLSKZU", "V"),
    ("kp_glskab", "KP_GLSKAB", "V"),
    ("ki_glskab", "KI_GLSKAB", "V"),
    ("tvglskzucg", "KL_TVGLSKZUCG", "C"),
    ("tvglskabcg", "KL_TVGLSKABCG", "C"),
    ("ndglsposhs", "KL_NDGLSPOSHS", "C"),
    ("ndglsneghs", "KL_NDGLSNEGHS", "C"),
    ("mehaupt", "KF_MEHAUPT", "M"),
    ("t_ramee", "T_RAMEE", "V"),
]

DOWN_FIELDS = [
    ("t_vkzu", "T_VKZU", "V"),
    ("t_f", "T_F", "V"),
    ("tra", "KL_TRA", "C"),
    ("pkzu", "KF_PKZU", "M"),
    ("pohinsynkzu", "KF_POHINSYNKZU", "M"),
    ("pohinsynkab", "KF_POHINSYNKAB", "M"),
    ("tsyncghinsynekzu", "KL_TSYNCGHINSYNEKZU", "C"),
    ("t_hinsyn", "T_HINSYN", "V"),
    ("pohkab", "KL_POHKAB", "C"),
    ("trahkab", "KL_TRAHKAB", "C"),
    ("pkab", "KF_PKAB", "M"),
    ("pgokab", "KL_PGOKAB", "C"),
    ("tsynmtkab", "KL_TSYNMTKAB", "C"),
    ("tsyncgmthinsyn", "KF_TSYNCGMTHINSYN", "M"),
    ("tramthinsynkab", "KL_TRAMTHINSYNKAB", "C"),
    ("pgrasynkab", "KL_PGRASYNKAB", "C"),
    ("tvkabmtnmotrs", "KF_TVKABMTNMOTRS", "M"),
    ("tmaxcg", "KL_TMAXCG", "C"),
    ("pgrasyn", "KL_PGRASYN", "C"),
    ("p_sysmin", "P_SYSMIN", "V"),
    ("mehaupt", "KF_MEHAUPT", "M"),
    ("menach", "KF_MENACH", "M"),
    ("wtsgls", "KF_WTSGLS", "M"),
    ("kp_glskzu", "KP_GLSKZU", "V"),
    ("ki_glskzu", "KI_GLSKZU", "V"),
    ("kp_glskab", "KP_GLSKAB", "V"),
    ("ki_glskab", "KI_GLSKAB", "V"),
    ("tvglskzucg", "KL_TVGLSKZUCG", "C"),
    ("tvglskabcg", "KL_TVGLSKABCG", "C"),
    ("nd_glsrs", "ND_GLSRS", "V"),
    ("tohkabmtnt", "KF_TOHKABMTNT", "M"),
    ("f_trahkab", "F_TRAHKAB", "V"),
    ("f_prahkab", "F_PRAHKAB", "V"),
]

CLUTCH_FIELDS = [
    ("psfcgmt", "KF_PSFCGMT_", "M"),
    ("tsfcg", "KL_TSFCG_", "C"),
    ("pfcg", "KL_PFCG_", "C"),
    ("pfhyscg", "KL_PFHYSCG_", "C"),
    ("tosfhs", "KF_TOSFHS_", "M"),
    ("tosfrs", "KF_TOSFRS_", "M"),
]

ENGAGE_GEARS = ["1", "R"]
ENGAGE_FIELDS = [
    ("tfv1", "KL_TFV1_", "C"),
    ("tfcgv2", "KL_TFCGV2_", "C"),
    ("tvkzu", "KL_TVKZU_", "C"),
    ("pkzu", "KF_PKZU_", "M"),
    ("pgrasyn", "KL_PGRASYN_", "C"),
    ("pgrasynv1", "KL_PGRASYNV1_", "C"),
    ("pgrasynv2", "KL_PGRASYNV2_", "C"),
    ("tvkzucgv2", "KL_TVKZUCGV2_", "C"),
    ("t_syn", "T_SYN_", "V"),
    ("porasyn", "KL_PORASYN_", "C"),
]

# EDS pressure->current curves in EdsValve order (see eds.h). POS is left
# out on purpose: its axis is partly zero in the export, so it cannot be
# interpolated. POS is driven by a direct current command instead.
EDS_CURVES = ["KL_EDSKVA", "KL_EDSKVB", "KL_EDSKVC", "KL_EDSKVD",
              "KL_EDSKVE", "KL_EDSKVSYS", "KL_EDSKVK0"]

GLOBAL_VALUES = [
    ("p_sysminpsaus", "P_SYSMINPSAUS"),
    ("p_sysmin_d", "P_SYSMIN_D"),
    ("p_sysmin_r", "P_SYSMIN_R"),
    ("p_maxobgls", "P_MAXOBGLS"),
    ("p_maxungls", "P_MAXUNGLS"),
    ("t_fngtgls", "T_FNGTGLS"),
    ("i_edsposv", "I_EDSPOSV"),
    ("i_edspsv", "I_EDSPSV"),
]
GLOBAL_TABLES = [
    ("kieds_rising", "KF_KIEDSCGSTACE", "M"),
    ("kieds_falling", "KF_KIEDSCGFAACE", "M"),
    ("psysn", "KF_PSYSN", "M"),
    ("tvpsein", "KL_TVPSEIN", "C"),
]
PSYS_GEARS = ["R", "1", "2", "3", "4", "5", "6", "7", "8"]


def value_scale(conv):
    """Scale factor from the A2L physical unit to the firmware integer unit.

    conv is the old export's conversion name or, for the Q030 export, the
    A2L unit string (see q030_items)."""
    unit_scale = {"[bar]": 1000.0, "[bar/s]": 1000.0, "[mA]": 10.0,
                  "[-]%": 100.0}
    if conv in unit_scale:
        return unit_scale[conv]
    if conv.startswith("UMR_BAR_SEC"):
        return 1000.0          # bar/s -> mbar/s
    if conv.startswith("UMR_BAR__"):
        return 1000.0          # bar -> mbar
    if conv.startswith("UMR_MA__"):
        return 10.0            # mA -> 0.1 mA
    if conv.startswith("UMR___1_100"):
        return 100.0           # factor -> percent (F_TRAHKAB, F_PRAHKAB)
    return 1.0                 # ms, Nm, rpm, degC, PI gains


def axis_scale(name):
    # EDS curve axes are pressure in mbar already; everything else is in
    # rpm, Nm, degC, ms or mA, which are used as-is.
    return 1.0


class Gen:
    def __init__(self, items, use_el):
        self.by = {x["name"]: x for x in items}
        self.use_el = use_el
        self.out = []
        self.count = 0
        self.used = []
        self.export = False  # 330e: shift-table arrays get external linkage


    def get(self, name):
        if self.use_el and (name + "_EL") in self.by:
            name = name + "_EL"
        x = self.by.get(name)
        if x is None:
            sys.exit("missing calibration item " + name)
        self.used.append(name)
        return x

    def arr(self, ctype, cname, vals):
        body = ", ".join(str(v) for v in vals)
        # The 330e up / down arrays are shared with cal_f11.cpp (its
        # fallback fields point at them instead of carrying copies).
        link = "extern" if self.export and cname.startswith(("up", "dn")) else "static"
        self.out.append("%s const %s %s[] = {%s};" % (link, ctype, cname, body))

    def ints(self, vals, scale, lo, hi, what):
        res = []
        for v in vals:
            iv = int(round(v * scale))
            if iv < lo or iv > hi:
                sys.exit("%s value %s out of range" % (what, iv))
            res.append(iv)
        return res

    def curve(self, name, cname):
        x = self.get(name)
        assert x["type"] == "CURVE", name
        ax = x["axes"][0]
        xs = self.ints(ax[1], axis_scale(ax[0]), -32768, 32767, name)
        ys = self.ints(x["values"], value_scale(x["conv"]), -2**31, 2**31 - 1, name)
        assert len(xs) == len(ys), name
        self.out.append("// %s @%s  x=%s  (%s)" % (x["name"], x["addr"], ax[0], x["conv"]))
        self.arr("int16_t", cname + "_x", xs)
        self.arr("int32_t", cname + "_y", ys)
        self.count += len(ys)
        return "{%d, %s_x, %s_y}" % (len(xs), cname, cname)

    def map(self, name, cname):
        x = self.get(name)
        assert x["type"] == "MAP", name
        ax, ay = x["axes"][0], x["axes"][1]
        xs = self.ints(ax[1], axis_scale(ax[0]), -32768, 32767, name)
        ys = self.ints(ay[1], axis_scale(ay[0]), -32768, 32767, name)
        zs = self.ints(x["values"], value_scale(x["conv"]), -2**31, 2**31 - 1, name)
        assert len(zs) == len(xs) * len(ys), name
        self.out.append("// %s @%s  x=%s y=%s  (%s)" % (x["name"], x["addr"], ax[0], ay[0], x["conv"]))
        self.arr("int16_t", cname + "_x", xs)
        self.arr("int16_t", cname + "_y", ys)
        self.arr("int32_t", cname + "_z", zs)
        self.count += len(zs)
        return "{%d, %d, %s_x, %s_y, %s_z}" % (len(xs), len(ys), cname, cname, cname)

    def value(self, name):
        x = self.get(name)
        assert x["type"] == "VALUE", name
        v = self.ints(x["values"][:1], value_scale(x["conv"]), -2**31, 2**31 - 1, name)[0]
        self.count += 1
        return v, "%s @%s" % (x["name"], x["addr"])

    def struct_init(self, fields, suffix, prefix_c):
        parts = []
        for field, a2l, kind in fields:
            name = a2l + suffix
            cname = "%s_%s" % (prefix_c, field)
            if kind == "V":
                v, src = self.value(name)
                parts.append("   %d, // %s %s" % (v, field, src))
            elif kind == "C":
                parts.append("   %s, // %s" % (self.curve(name, cname), field))
            else:
                parts.append("   %s, // %s" % (self.map(name, cname), field))
        return "{\n" + "\n".join(parts) + "\n}"


def struct_def(name, fields):
    lines = ["struct %s" % name, "{"]
    for field, a2l, kind in fields:
        t = {"V": "int32_t", "C": "CalCurve", "M": "CalMap"}[kind]
        lines.append("   %s %s; // %sxy" % (t, field, a2l))
    lines.append("};")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Sources

def q030_items(path):
    """Bring the Q030 full export into the format Gen expects.

    conv is set to the A2L unit string (value_scale knows the ones that
    need scaling; a factor "[-]" on an F_ item is a 1/100 factor, stored
    as percent like the old export). Axis inputs are cut to the last
    path element (e.g. zff_atsys._.iSignale._.obj.sig_c_getr -> sig_c_getr).
    """
    out = []
    for x in json.load(open(path)):
        if x["type"] not in ("VALUE", "CURVE", "MAP"):
            continue
        unit = x.get("unit") or ""
        if unit == "[-]" and x["name"].startswith("F_"):
            unit = "[-]%"
        out.append({
            "name": x["name"], "addr": x["addr"], "type": x["type"],
            "conv": unit, "desc": x.get("desc", ""),
            "evidence": x.get("evidence", ""),
            "axes": [[a["input"].split(".")[-1], a["values"]] for a in x["axes"]],
            "values": x["values"],
        })
    return out


N6_CURVES = [
    ("k_n6", "D_k_n6"), ("k_nab", "D_k_nab"), ("k_nmot", "D_k_nmot"),
    ("k_n6_ntopt", "D_k_n6_ntOpt"), ("k_nab_ntopt", "D_k_nab_ntOpt"),
    ("targetgear_ntopt", "D_k_TargetGear_ntOpt"), ("i_targetgear", "D_i_TargetGear"),
]


def sport_name(a2l, suffix):
    """OEM Sport label: SPORT goes in front of the pair / clutch suffix."""
    if a2l.endswith("_"):
        return a2l[:-1] + "SPORT_" + suffix
    return a2l + "SPORT" + suffix


class SportGen(Gen):
    """Gen that prefers the OEM Sport label of an item."""

    def __init__(self, items, use_el):
        Gen.__init__(self, items, use_el)
        self.sport_hits = []
        self.sport_miss = []

    def struct_init_sport(self, fields, suffix, prefix_c):
        parts = []
        for field, a2l, kind in fields:
            sp = sport_name(a2l, suffix)
            name = sp if sp in self.by else a2l + suffix
            (self.sport_hits if sp in self.by else self.sport_miss).append(name)
            cname = "%s_%s" % (prefix_c, field)
            if kind == "V":
                v, src = self.value(name)
                parts.append("   %d, // %s %s" % (v, field, src))
            elif kind == "C":
                parts.append("   %s, // %s" % (self.curve(name, cname), field))
            else:
                parts.append("   %s, // %s" % (self.map(name, cname), field))
        return "{\n" + "\n".join(parts) + "\n}"


class Q030RefGen(SportGen):
    """Builds the same initialisers as the 330e generator but emits no
    arrays: it only notes the extern declarations of the 330e arrays."""

    def __init__(self, items, use_el):
        SportGen.__init__(self, items, use_el)
        self.decls = []

    def arr(self, ctype, cname, vals):
        self.decls.append("extern const %s %s[];" % (ctype, cname))

    def field(self, a2l, suffix, kind, prefix_c, field, sport):
        """One field initialiser, same name rules as struct_init(_sport)."""
        name = a2l + suffix
        if sport:
            sp = sport_name(a2l, suffix)
            if sp in self.by:
                name = sp
        cname = "%s_%s" % (prefix_c, field)
        if kind == "V":
            return str(self.value(name)[0])
        if kind == "C":
            return self.curve(name, cname)
        return self.map(name, cname)


def lic_text():
    return open(os.path.join(ROOT, "tools", "license_header.txt")).read().rstrip("\n")


def write(rel, lines):
    with open(os.path.join(ROOT, rel), "w") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------------------
# cal_330e: Normal + Sport from Q030

def gen_330e(items, args):
    g = SportGen(items, args.set == "el")
    g.export = True
    inits = {}
    inits["up"] = [g.struct_init(UP_FIELDS, p, "up%s" % p) for p in UP_PAIRS]
    inits["down"] = [g.struct_init(DOWN_FIELDS, p, "dn%s" % p) for p in DOWN_PAIRS]
    inits["clutch"] = [g.struct_init(CLUTCH_FIELDS, c, "cl%s" % c) for c in CLUTCHES]
    inits["engage"] = [g.struct_init(ENGAGE_FIELDS, n, "ge%s" % n) for n in ENGAGE_GEARS]
    eds = [g.curve(n, "eds_%s" % n[7:].lower()) for n in EDS_CURVES]
    psys = [g.map("KF_PSYS_" + n, "psys_%s" % n.lower()) for n in PSYS_GEARS]
    glob_parts = []
    for field, a2l in GLOBAL_VALUES:
        v, src = g.value(a2l)
        glob_parts.append("   %d, // %s %s" % (v, field, src))
    for field, a2l, kind in GLOBAL_TABLES:
        init = g.curve(a2l, "g_" + field) if kind == "C" else g.map(a2l, "g_" + field)
        glob_parts.append("   %s, // %s %s" % (init, field, a2l))
    # 330e virtual-turbine k-factors (n6): exported, not used for control.
    n6_parts = []
    for field, a2l in N6_CURVES:
        x = g.get(a2l)
        xs = [int(v) for v in x["axes"][0][1]]
        ys = [int(round(v * 10000)) for v in x["values"]]
        cname = "n6_" + field
        g.out.append("// %s @%s  index axis, x10000 [C values, H index mapping]" % (a2l, x["addr"]))
        g.arr("int16_t", cname + "_x", xs)
        g.arr("int32_t", cname + "_y", ys)
        n6_parts.append("   {%d, %s_x, %s_y}, // %s %s" % (len(xs), cname, cname, field, a2l))
    # OEM Sport set (same structs, Sport label where Q030 has one)
    g.out.append("// ---- OEM Sport set (SPORT labels, Normal where none) ----")
    inits["up_sp"] = [g.struct_init_sport(UP_FIELDS, p, "ups%s" % p) for p in UP_PAIRS]
    inits["down_sp"] = [g.struct_init_sport(DOWN_FIELDS, p, "dns%s" % p) for p in DOWN_PAIRS]
    inits["clutch_sp"] = [g.struct_init_sport(CLUTCH_FIELDS, c, "cls%s" % c) for c in CLUTCHES]
    if not g.sport_hits:
        sys.exit("no OEM Sport labels found")

    gen_note = ("/* GENERATED by tools/gen_cal.py from 330e_Q030_cal_full.json "
                "(EPK = A2L build, exact labels [C]; programme set: %s).\n"
                " * Do not edit by hand. Sport set: %d items use their OEM SPORT "
                "label, %d have none\n * and use the Normal one. */"
                % (args.set, len(g.sport_hits), len(g.sport_miss)))
    hdr = [lic_text(), gen_note, "#ifndef CAL_330E_H", "#define CAL_330E_H", "",
           '#include "caltypes.h"', "",
           "// Adjacent shift pairs. Index 0 = 1->2 (or 2->1), index 6 = 7->8 (or 8->7).",
           "#define CAL_NUM_PAIRS 7", "",
           struct_def("UpShiftCal", UP_FIELDS), "",
           struct_def("DownShiftCal", DOWN_FIELDS), "",
           struct_def("ClutchFillCal", CLUTCH_FIELDS), "",
           struct_def("EngageCal", ENGAGE_FIELDS), "",
           "struct GlobalCal", "{"]
    for field, a2l in GLOBAL_VALUES:
        hdr.append("   int32_t %s; // %s" % (field, a2l))
    for field, a2l, kind in GLOBAL_TABLES:
        hdr.append("   %s %s; // %s" % ("CalCurve" if kind == "C" else "CalMap", field, a2l))
    hdr += ["};", "",
            "extern const UpShiftCal calUp[CAL_NUM_PAIRS];     // 12 23 34 45 56 67 78",
            "extern const DownShiftCal calDown[CAL_NUM_PAIRS]; // 21 32 43 54 65 76 87",
            "extern const ClutchFillCal calClutch[5];          // A B C D E",
            "extern const EngageCal calEngage[2];              // 0 = N->1, 1 = N->R",
            "extern const CalCurve calEds[7];                  // A B C D E SYS K0, mbar -> 0.1 mA",
            "extern const CalMap calPsys[9];                   // R 1 2 3 4 5 6 7 8, x=m_t Nm, y=speed rpm",
            "extern const GlobalCal calGlobal;",
            "// OEM Sport set (Q030 *SPORT* labels, Normal label where none exists)",
            "extern const UpShiftCal calUpSport[CAL_NUM_PAIRS];",
            "extern const DownShiftCal calDownSport[CAL_NUM_PAIRS];",
            "extern const ClutchFillCal calClutchSport[5];",
            "#define CAL_SPORT_LABELS %d // items with their own SPORT label" % len(g.sport_hits),
            "",
            "/* 330e virtual-turbine k-factors (TurbineSpeedVirtualIn): n_TuVirt =",
            " * k_n6 n6 + k_nab n_ab (+ k_nmot n_mot) [I]. x10000, on the A2L index axis.",
            " * The index is a shift / state index whose mapping to gears is NOT",
            " * resolved [H]: exported for reference, not used for control. The F11",
            " * has none of these tables and uses its turbine sensor directly [C]. */",
            "struct CalN6", "{"] + ["   CalCurve %s; // %s" % (f, a) for f, a in N6_CURVES] + [
            "};",
            "extern const CalN6 calN6;",
            "", "#endif // CAL_330E_H", ""]

    src = [lic_text(), gen_note, '#include "cal_330e.h"', ""] + g.out + [""]
    src.append("const UpShiftCal calUp[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["up"]) + "\n};\n")
    src.append("const DownShiftCal calDown[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["down"]) + "\n};\n")
    src.append("const ClutchFillCal calClutch[5] = {\n" + ",\n".join(inits["clutch"]) + "\n};\n")
    src.append("const EngageCal calEngage[2] = {\n" + ",\n".join(inits["engage"]) + "\n};\n")
    src.append("const CalCurve calEds[7] = {\n   " + ",\n   ".join(eds) + "\n};\n")
    src.append("const CalMap calPsys[9] = {\n   " + ",\n   ".join(psys) + "\n};\n")
    src.append("const GlobalCal calGlobal = {\n" + "\n".join(glob_parts) + "\n};\n")
    src.append("const UpShiftCal calUpSport[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["up_sp"]) + "\n};\n")
    src.append("const DownShiftCal calDownSport[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["down_sp"]) + "\n};\n")
    src.append("const ClutchFillCal calClutchSport[5] = {\n" + ",\n".join(inits["clutch_sp"]) + "\n};\n")
    src.append("const CalN6 calN6 = {\n" + "\n".join(n6_parts) + "\n};\n")
    src.append("// %d calibration values from %d items.\n" % (g.count, len(g.used)))
    write("include/cal_330e.h", hdr)
    write("src/cal_330e.cpp", src)
    print("cal_330e: %d values from %d items, %d Sport labels"
          % (g.count, len(g.used), len(g.sport_hits)))


# ---------------------------------------------------------------------------
# cal_f11: the v058 solenoid block (0x19FCA8)

F11_EDS_CURVES = ["KL_EDSKVA", "KL_EDSKVB", "KL_EDSKVC", "KL_EDSKVD",
                  "KL_EDSKVE", "KL_EDSSYS", "KL_EDSTCC"]
F11_EDS_BLOCK = 0x19FCA8


def f11_eds(path):
    by = {x["name"]: x for x in json.load(open(path))}
    curves = []
    for i, n in enumerate(F11_EDS_CURVES):
        x = by[n]
        addr = int(x["addr"], 16)
        if addr != F11_EDS_BLOCK + 0x34 * i:
            sys.exit("%s not in the 0x19FCA8 block" % n)
        curves.append((n, x["addr"], [int(v) for v in x["axes"][0]["values"]],
                       [int(v) for v in x["raw_y"]]))
    return curves


def curve_lines(cname, n, src, xs, ys):
    return ["// %s %s" % (n, src),
            "static const int16_t %s_x[] = {%s};" % (cname, ", ".join(map(str, xs))),
            "static const int32_t %s_y[] = {%s};" % (cname, ", ".join(map(str, ys)))]


F11_PSYS = ["KF_PSYS_" + g for g in PSYS_GEARS]


def f11_fw_items(path):
    """F11 exports in firmware units (dict axes) -> Gen items, names @F11."""
    out = []
    for x in json.load(open(path)):
        if x["type"] not in ("CURVE", "MAP"):
            continue
        y = dict(x)
        y["axes"] = [[a["name"], a["values"]] for a in x["axes"]]
        if x.get("conv") == "mbar_to_0.1mA":
            y["values"] = x["raw_y"]
        y["conv"] = "F11_" + x.get("conv", "fw")
        y["name"] = x["name"] + "@F11"
        out.append(y)
    return out


def f11_ported_c(path):
    """[C] rows of the F11 v058 port only (Q030 format); never I / H."""
    return [dict(x, name=x["name"] + "@F11C") for x in q030_items(path)
            if x["evidence"].startswith("C")]


F11_PROGRAMS = 3  # 0 base, 1 SP, 2 SPORT [I] (F11_SAM_ADDSCOPE.md 5.2)
F11_GEARS = ["R", "1", "2", "3", "4", "5", "6", "7", "8"]
F11_ENGAGE_GEARIN = ["pgrasyn", "pgrasynv1", "pgrasynv2", "porasyn"]  # R / 1 only
F11_ENGAGE_STAGE4 = ["tfv1", "tvkzu", "tvkzucgv2", "t_syn"]
F11_GLOBAL_OWN = {"p_sysmin_d": "P_SYSMIN_D", "p_sysmin_r": "P_SYSMIN_R",
                  "p_maxobgls": "P_MAXOBGLS", "p_maxungls": "P_MAXUNGLS",
                  "t_fngtgls": "T_FNGTGLS"}


def f11_stage4(sp_path, gi_path):
    """Ian's stage 4 exports -> Gen items keyed by role, not by name.

    Shift maps are keyed (direction, program, pair) from the index tables
    [C]; the porter's names are never used. The alternate gear-in family
    (descriptor 0x50BB4, F11_KF_PKZU_GEALT_*) is left out [H trigger].
    Returns (items, keys) with keys[(dir, prog, pair)] = item name."""
    items, keys = [], {}
    sp = json.load(open(sp_path))["items"]
    for x in sp:
        grp = x["group"]
        if "alternate" in grp or x["block"] == "0x50bb4":
            continue
        if grp.startswith("upshift_on-coming"):
            k = ("up_on", x["program"], x["pair"])
        elif grp.startswith("upshift_off-going"):
            k = ("up_off", None, x["pair"])
        elif grp.startswith("downshift_off-going"):
            k = ("dn_off", x["program"], x["pair"])
        elif grp.startswith("gear_in ("):
            k = ("gearin", None, x["pair"][2:])
        else:
            continue  # follow-up / multiple downshifts: no firmware field
        name = "F11S4:%s:%s:%s" % k
        y = dict(x)
        y["name"] = name
        y["axes"] = [[a["name"], a["values"]] for a in x["axes"]]
        y["conv"] = "F11_fw_mbar"
        items.append(y)
        keys[k] = name
    gi = json.load(open(gi_path))
    for x in gi["gear_in"] + gi["globals"]:
        if x["type"] not in ("VALUE", "CURVE"):
            continue
        y = dict(x)
        y["name"] = x["name"] + "@F11T"
        y["axes"] = [[a["name"], a["values"]] for a in x.get("axes", [])]
        y["conv"] = "F11_fw"
        items.append(y)
    return items, keys


def gen_f11(curves, args, q030):
    s4, keys = f11_stage4(args.f11_shift, args.f11_gearin)
    g = Gen(q030 + f11_fw_items(args.f11_psys) + f11_ported_c(args.f11_port) + s4, False)
    ref = Q030RefGen(q030, args.set == "el")
    psys = [g.map(n + "@F11", "f11psys_%s" % n[8:].lower()) for n in F11_PSYS]

    def fld(kind, name, cname):
        if kind == "V":
            v, src = g.value(name)
            return str(v), src
        if kind == "C":
            return g.curve(name, cname), name
        return g.map(name, cname), name

    # Shift tables per program. F11 [stage 4]: upshift on-coming KF_PKZU
    # (per program), upshift off-going KF_PKAB (one set), downshift
    # off-going KF_PKAB (per program), P_SYSMINxy. The rest point at the
    # 330e arrays (program 0: Normal, 1 / 2: the 330e Sport set).
    shared = {}
    up_sets, dn_sets = [], []
    for prog in range(F11_PROGRAMS):
        ups = []
        for pair in UP_PAIRS:
            parts = []
            for field, a2l, kind in UP_FIELDS:
                if field == "pkzu":
                    init, _ = fld("M", keys[("up_on", prog, pair)], "f11up%d_%s_pkzu" % (prog, pair))
                    parts.append("   %s, // pkzu F11 stage 4 [C pair/program, I name]" % init)
                elif field == "pkab":
                    k = ("up_off", None, pair)
                    if k not in shared:
                        shared[k] = fld("M", keys[k], "f11up_%s_pkab" % pair)[0]
                    parts.append("   %s, // pkab F11 stage 4 [C pair, I name]" % shared[k])
                elif field == "p_sysmin":
                    v, src = fld("V", "P_SYSMIN%s@F11T" % pair, None)
                    parts.append("   %s, // p_sysmin F11 %s [I]" % (v, src))
                else:
                    pre = ("up%s" if prog == 0 else "ups%s") % pair
                    init = ref.field(a2l, pair, kind, pre, field, prog > 0)
                    parts.append("   %s, // %s (330e%s FALLBACK)" % (init, field, "" if prog == 0 else " Sport"))
            ups.append("{\n" + "\n".join(parts) + "\n}")
        up_sets.append(ups)
        dns = []
        for pair in DOWN_PAIRS:
            parts = []
            for field, a2l, kind in DOWN_FIELDS:
                if field == "pkab":
                    init, _ = fld("M", keys[("dn_off", prog, pair)], "f11dn%d_%s_pkab" % (prog, pair))
                    parts.append("   %s, // pkab F11 stage 4 [C pair/program, I name]" % init)
                elif field == "p_sysmin":
                    v, src = fld("V", "P_SYSMIN%s@F11T" % pair, None)
                    parts.append("   %s, // p_sysmin F11 %s [I]" % (v, src))
                else:
                    pre = ("dn%s" if prog == 0 else "dns%s") % pair
                    init = ref.field(a2l, pair, kind, pre, field, prog > 0)
                    note = "330e%s FALLBACK" % ("" if prog == 0 else " Sport")
                    if field == "pkzu":
                        note += ": F11 down on-coming 8x8 at 0x54BA0 not mapped"
                    parts.append("   %s, // %s (%s)" % (init, field, note))
            dns.append("{\n" + "\n".join(parts) + "\n}")
        dn_sets.append(dns)

    # Gear-in for R and 1..8 (stage 4 timings, F11 KF_PKZU_g).
    eng = {}
    for gear in F11_GEARS:
        parts = []
        for field, a2l, kind in ENGAGE_FIELDS:
            cname = "f11ge%s_%s" % (gear, field)
            if field == "pkzu":
                init, _ = fld("M", keys[("gearin", None, gear)], cname)
                note = "F11 stage 4 [%s]" % ("C" if gear == "8" else "I")
            elif field in F11_ENGAGE_STAGE4:
                init, src = fld(kind, a2l + gear + "@F11T", cname)
                note = "F11 stage 4 [I]"
            elif field in F11_ENGAGE_GEARIN and gear in ("R", "1"):
                init, src = fld(kind, a2l + gear + "@F11", cname)
                note = "F11 [I]"
            else:
                init, src = fld(kind, a2l + gear, cname)
                note = "330e Q030 FALLBACK"
                if field == "tfcgv2":
                    note += " [H: the F11 record at this slot is a 16-bit curve on another axis]"
            parts.append("   %s, // %s (%s)" % (init, field, note))
        eng[gear] = "{\n" + "\n".join(parts) + "\n}"

    glob = []
    for field, a2l in GLOBAL_VALUES:
        if field in F11_GLOBAL_OWN:
            v, src = g.value(F11_GLOBAL_OWN[field] + "@F11T")
            glob.append("   %d, // %s %s (F11 stage 4 [I])" % (v, field, src))
        else:
            v, src = g.value(a2l)
            glob.append("   %d, // %s %s (330e Q030 FALLBACK [H: no F11 item])" % (v, field, src))
    own_tables = {"kieds_rising": "KF_KIEDSCGSTACE@F11", "kieds_falling": "KF_KIEDSCGFAACE@F11",
                  "psysn": "KF_PSYSN@F11", "tvpsein": "KL_TVPSEIN@F11C"}
    for field, a2l, kind in GLOBAL_TABLES:
        name = own_tables[field]
        note = "F11 port [C]" if name.endswith("@F11C") else "F11 [C structure, I name]"
        init = g.curve(name, "f11g_" + field) if kind == "C" else g.map(name, "f11g_" + field)
        glob.append("   %s, // %s %s (%s)" % (init, field, a2l, note))
    tmax = g.curve("KL_TMAXPSYS@F11", "f11_tmaxpsys")
    posys = g.curve("KL_POSYSCG@F11T", "f11_posyscg")

    gen_note = ("/* GENERATED by tools/gen_cal.py from f11_sas_eds_cal.json, "
                "f11_psys_gearin_cal.json,\n"
                " * f11_shift_pressure_cal.json, f11_gearin_timing_cal.json (stage 4), "
                "the [C] rows of\n * f11_a81v058_cal_ported and 330e_Q030_cal_full.json. "
                "Do not edit by hand.\n"
                " * EDS: the F11 v058 solenoid p->I block at 0x19FCA8 (A..E, SYS, TCC), "
                "mbar -> 0.1 mA.\n * A..E and TCC are byte-identical to the a81 v095 "
                "8HP90 set [C, checked by the\n * generator]; SYS is not. The next item "
                "at 0x19FE14 is KF_KIEDSCGFAACE (EDS\n * temperature correction), not POS.\n"
                " * Line pressure, EDS temperature corrections, R / 1st gear-in curves: "
                "f11_psys_gearin_cal\n * [C structure / I name]. Stage 4: shift pressure "
                "maps keyed by program (0 base,\n * 1 SP, 2 SPORT [I]) + pair + direction "
                "from the F11 index tables [C], axes speed\n * raw*32 rpm, torque Nm, "
                "values mbar [C]; gear-in timings for R, 1..8; P_SYSMIN\n * globals. The "
                "alternate gear-in family 0x50BB4 is left out. Fields marked\n * FALLBACK "
                "point at the 330e Q030 arrays (cal_330e.cpp). */")
    src = [lic_text(), gen_note, '#include "cal_f11.h"', "",
           "// 330e arrays used by the FALLBACK fields (defined in cal_330e.cpp)."]
    src += sorted(set(ref.decls)) + [""]
    inits = []
    for n, addr, xs, ys in curves:
        cname = "f11eds_%s" % n[6:].lower()
        src += curve_lines(cname, n, "@" + addr, xs, ys)
        inits.append("{%d, %s_x, %s_y}" % (len(xs), cname, cname))
    src += g.out + [""]
    src += ["const CalCurve calF11Eds[7] = {\n   " + ",\n   ".join(inits) + "\n};", ""]
    src += ["const CalMap calF11Psys[9] = {\n   " + ",\n   ".join(psys) + "\n};", ""]
    src += ["const UpShiftCal calF11Up[F11_PROGRAMS][CAL_NUM_PAIRS] = {\n" +
            ",\n".join("{\n" + ",\n".join(u) + "\n}" for u in up_sets) + "\n};", ""]
    src += ["const DownShiftCal calF11Down[F11_PROGRAMS][CAL_NUM_PAIRS] = {\n" +
            ",\n".join("{\n" + ",\n".join(d) + "\n}" for d in dn_sets) + "\n};", ""]
    src += ["const EngageCal calF11EngageAll[9] = {\n" +
            ",\n".join(eng[gr] for gr in F11_GEARS) + "\n};", ""]
    src += ["const EngageCal calF11Engage[2] = {\n" + eng["1"] + ",\n" + eng["R"] + "\n};", ""]
    src += ["const GlobalCal calF11Global = {\n" + "\n".join(glob) + "\n};", ""]
    src += ["// KL_TMAXPSYS: raw axis / values (unit not known), not used.",
            "const CalCurve calF11TmaxPsys = %s;" % tmax, ""]
    src += ["// KL_POSYSCG [I]: line pressure offset vs oil temperature, mbar. Exported,",
            "// not used yet.",
            "const CalCurve calF11PosysCg = %s;" % posys, ""]
    hdr = [lic_text(), gen_note, "#ifndef CAL_F11_H", "#define CAL_F11_H", "",
           '#include "cal_330e.h"', "",
           "#define F11_PROGRAMS 3 // shift programs: 0 base, 1 SP, 2 SPORT [I]", "",
           "extern const CalCurve calF11Eds[7];   // A B C D E SYS TCC, mbar -> 0.1 mA",
           "extern const CalMap calF11Psys[9];    // R 1..8, x=m_t Nm, y=n_mot rpm [I]",
           "// Shift tables per program, pairs as calUp / calDown.",
           "extern const UpShiftCal calF11Up[F11_PROGRAMS][CAL_NUM_PAIRS];",
           "extern const DownShiftCal calF11Down[F11_PROGRAMS][CAL_NUM_PAIRS];",
           "extern const EngageCal calF11EngageAll[9]; // gear-in R, 1..8 (reference)",
           "extern const EngageCal calF11Engage[2]; // N->1, N->R (used)",
           "extern const GlobalCal calF11Global;  // F11 P_SYSMIN / GLS, psysn, kieds, tvpsein",
           "extern const CalCurve calF11TmaxPsys; // raw, not used",
           "extern const CalCurve calF11PosysCg;  // KL_POSYSCG, mbar vs degC, not used",
           "", "#endif // CAL_F11_H", ""]
    write("include/cal_f11.h", hdr)
    write("src/cal_f11.cpp", src)
    print("cal_f11: EDS, line pressure, %d programs x up/down, gear-in R..8, %d values"
          % (F11_PROGRAMS, g.count))


# ---------------------------------------------------------------------------
# cal_v095: v095 solenoid sets, per-cal size and reference data

V095_CALS = [  # (file key, C name, gearbox)
    ("F11_N47_8HP45", "F11N47", "8HP45"),
    ("N57_8HP70", "N57", "8HP70"),
    ("N63_8HP70", "N63", "8HP70"),
    ("N74_8HP90", "N74", "8HP90"),
    ("RR4_N74_8HP90", "RR4N74", "8HP90"),
]
V095_EDS = ["KL_EDSKVA", "KL_EDSKVB", "KL_EDSKVC", "KL_EDSKVD",
            "KL_EDSKVE", "KL_EDSKVSYS", "KL_EDSKVK0"]
# Per-size reference items Damien asked for (VARIANTS_SUMMARY.md 2 / 5.2).
REF_RE = re.compile(r"^(TAB_MVG_MT|KL_RGANGLIMIT|KL_MOMOTMAXNDRSS|"
                    r"KL_HYSB_KPL_.*_HYSKOMP|KF_TOSF|KL_TOSF|KL_TSFCG|KF_PSFCGMT|"
                    r"KL_PFCG_|KL_PFHYSCG_|KF_FKOLBENPOSKOR|KL_PFCGGA|"
                    r"KL_TVCGSAEND|TAB_MT_GLSADGR)|FUELLOPT|FUELLRED")


def v095_eds(p):
    out = []
    for n in V095_EDS:
        x = p[n]
        if x["evidence"] not in ("C", "I"):
            sys.exit("v095 %s evidence %s" % (n, x["evidence"]))
        out.append((n, x["cal_addr"], [int(round(v)) for v in x["axes"][0]["values"]],
                    [int(round(v * 10)) for v in x["values"]]))
    return out


def milli(vals):
    return [int(round(v * 1000)) for v in vals]


class RefGen:
    """Reference tables, arrays shared by content."""

    def __init__(self):
        self.arrays = {}
        self.lines = []

    def arr(self, vals):
        key = tuple(vals)
        if key not in self.arrays:
            cname = "ref_a%d" % len(self.arrays)
            self.arrays[key] = cname
            self.lines.append("static const int32_t %s[] = {%s};"
                              % (cname, ", ".join(map(str, vals))))
        return self.arrays[key]

    def table(self, name, typ, unit, evidence, axes, values):
        kind = {"VALUE": 0, "CURVE": 1, "MAP": 2}.get(typ)
        if kind is None:
            return None
        nx = len(axes[0]) if len(axes) > 0 else 1
        ny = len(axes[1]) if len(axes) > 1 else 1
        if nx * ny != len(values) and kind != 0:
            return None  # layout does not fit: leave it out
        xa = self.arr(milli(axes[0])) if len(axes) > 0 else "0"
        ya = self.arr(milli(axes[1])) if len(axes) > 1 else "0"
        za = self.arr(milli(values))
        return '{"%s", "%s", \'%s\', %d, %d, %d, %s, %s, %s}' % (
            name, unit.replace('"', ""), evidence, kind, nx, ny, xa, ya, za)


def first(p, n, which):
    x = p.get(n)
    if not x:
        return None
    return x["values"][which]


def gen_v095(args, q030):
    cals = {}
    for key, cname, box in V095_CALS:
        f = os.path.join(args.v095, "params_%s.json" % key)
        cals[key] = json.load(open(f))["params"]
    # Solenoid sets: 8HP45/70 cals share one, the 8HP90 cals another.
    eds45 = v095_eds(cals["F11_N47_8HP45"])
    for k in ("N57_8HP70", "N63_8HP70"):
        if v095_eds(cals[k]) != eds45:
            sys.exit("v095 %s EDS set differs from the 8HP45 one" % k)
    eds90 = v095_eds(cals["N74_8HP90"])
    if v095_eds(cals["RR4_N74_8HP90"]) != eds90:
        sys.exit("v095 RR4 EDS set differs from N74")
    f11 = f11_eds(args.f11)
    same = [i for i in range(7) if (f11[i][2], f11[i][3]) == (eds90[i][2], eds90[i][3])]
    if same != [0, 1, 2, 3, 4, 6]:
        sys.exit("F11 v058 vs v095 8HP90 EDS identity changed: %s" % same)

    src = []
    eds_inits = {}
    for tag, curves in (("v45", eds45), ("v90", eds90)):
        inits = []
        for n, addr, xs, ys in curves:
            cname = "%seds_%s" % (tag, n[7:].lower())
            src += curve_lines(cname, n, "cal@" + addr + " [I]", xs, ys)
            inits.append("{%d, %s_x, %s_y}" % (len(xs), cname, cname))
        eds_inits[tag] = inits

    # Size scalars and reference tables per row. Rows in calset order.
    rg = RefGen()
    q = {x["name"]: x for x in q030}
    rows = []  # (cname, desc, torque class, reverse start, safety offset, refs)
    ref_names = set()
    for key, cname, box in V095_CALS:
        ref_names |= set(n for n in cals[key] if REF_RE.search(n))
    # 330e (Q030, exact labels): only the names the v095 sets have, so the
    # rows compare like for like.
    refs = []
    for n in sorted(ref_names):
        x = q.get(n)
        if not x:
            continue
        t = rg.table(n, x["type"], x["conv"], "C", [a[1] for a in x["axes"]], x["values"])
        if t:
            refs.append(t)
    rows.append(("Q030", "8HP75 330e Q030 (A2L exact)",
                 q["TAB_MVG_MT"]["values"][-1], q["KL_RGANGLIMIT"]["values"][0],
                 q["KL_MOMOTMAXNDRSS32"]["values"][0], refs))
    for key, cname, box in V095_CALS:
        p = cals[key]
        refs = []
        for n in sorted(x for x in p if REF_RE.search(x)):
            x = p[n]
            if x["evidence"] not in ("C", "I"):
                continue  # never [H]
            t = rg.table(n, x["type"], x.get("unit", ""), x["evidence"],
                         [a["values"] for a in x["axes"]], x["values"])
            if t:
                refs.append(t)
        rows.append((cname, "%s a81 v095 %s [I]" % (box, key),
                     first(p, "TAB_MVG_MT", -1), first(p, "KL_RGANGLIMIT", 0),
                     first(p, "KL_MOMOTMAXNDRSS32", 0), refs))

    gen_note = ("/* GENERATED by tools/gen_cal.py from the a81 v095 params_<cal>.json "
                "files and\n * 330e_Q030_cal_full.json. Do not edit by hand.\n"
                " * Solenoid sets: v095 8HP45/70 (shared by F11 N47, N57, N63) and v095 "
                "8HP90\n * (N74, RR4 N74) [I]. Size data: torque class = TAB_MVG_MT top, "
                "reverse limit\n * start = KL_RGANGLIMIT first point, safety offset = "
                "KL_MOMOTMAXNDRSS32 first\n * point; v095 values are [I], the 330e "
                "ones are exact labels but their per-size\n * meaning is [I] too. "
                "Reference tables are data only (milli-units of the\n * export unit); "
                "no output uses them. */")
    out = [lic_text(), gen_note, '#include "cal_v095.h"', ""] + src + [""] + rg.lines + [""]
    for tag in ("v45", "v90"):
        out.append("const CalCurve calV095Eds%s[7] = {\n   %s\n};\n"
                   % ("45" if tag == "v45" else "90", ",\n   ".join(eds_inits[tag])))
    size_inits = []
    for cname, desc, tq, rv, so, refs in rows:
        out.append("static const CalRef ref%s[] = {\n   %s\n};" % (cname, ",\n   ".join(refs)))
        size_inits.append('   {"%s", %d, %d, %d, %d, ref%s}, // %s'
                          % (cname, int(round(tq)), int(round(rv)), int(round(so)),
                             len(refs), cname, desc))
    out.append("")
    out.append("const CalSizeInfo calSizes[CAL_SIZE_COUNT] = {\n%s\n};\n" % "\n".join(size_inits))
    hdr = [lic_text(), gen_note, "#ifndef CAL_V095_H", "#define CAL_V095_H", "",
           '#include "caltypes.h"', "",
           "/// One reference table (data only), values in milli-units of `unit`.",
           "struct CalRef", "{",
           "   const char *name;  // A2L label",
           "   const char *unit;  // export unit",
           "   char evidence;     // 'C' or 'I'",
           "   uint8_t kind;      // 0 value, 1 curve, 2 map",
           "   uint8_t nx, ny;",
           "   const int32_t *x;  // milli-units, 0 for a value",
           "   const int32_t *y;  // milli-units, 0 unless a map",
           "   const int32_t *z;  // milli-units, z[iy * nx + ix]",
           "};", "",
           "/// Per-calibration size data [I]: from the exported values only.",
           "struct CalSizeInfo", "{",
           "   const char *cal;        // source cal",
           "   int16_t torqueClassNm;  // TAB_MVG_MT top (550 / 760 / 1100)",
           "   int16_t revLimitNm;     // KL_RGANGLIMIT first point",
           "   int16_t safetyOfsNm;    // KL_MOMOTMAXNDRSS32 first point",
           "   uint16_t nRef;",
           "   const CalRef *ref;      // fill / piston / hysteresis / limit tables",
           "};", "",
           "enum CalSizeIdx",
           "{",
           "   CALSIZE_330E_Q030 = 0,"] + [
           "   CALSIZE_V095_%s = %d," % (c.upper(), i + 1) for i, (k, c, b) in enumerate(V095_CALS)] + [
           "   CAL_SIZE_COUNT", "};", "",
           "extern const CalSizeInfo calSizes[CAL_SIZE_COUNT];",
           "extern const CalCurve calV095Eds45[7]; // 8HP45/70 set: A B C D E SYS ch7 [I]",
           "extern const CalCurve calV095Eds90[7]; // 8HP90 set: A B C D E SYS ch7 [I]",
           "", "#endif // CAL_V095_H", ""]
    write("include/cal_v095.h", hdr)
    write("src/cal_v095.cpp", out)
    print("cal_v095: 2 EDS sets, %d size rows, %d shared arrays"
          % (len(rows), len(rg.arrays)))
    return f11


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q030", required=True, help="330e_Q030_cal_full.json")
    ap.add_argument("--f11", required=True, help="f11_sas_eds_cal.json (v058 EDS block)")
    ap.add_argument("--f11-psys", required=True, help="f11_psys_gearin_cal.json")
    ap.add_argument("--f11-shift", required=True, help="f11_shift_pressure_cal.json (stage 4)")
    ap.add_argument("--f11-gearin", required=True, help="f11_gearin_timing_cal.json (stage 4)")
    ap.add_argument("--f11-port", required=True, help="f11_a81v058_cal_ported.json ([C] rows only)")
    ap.add_argument("--v095", required=True, help="directory with params_<cal>.json")
    ap.add_argument("--set", choices=["normal", "el"], default="normal")
    args = ap.parse_args()
    items = q030_items(args.q030)
    gen_330e(items, args)
    f11 = gen_v095(args, items)
    gen_f11(f11, args, items)
    gen_sig()


# ---------------------------------------------------------------------------
# Calibration signature: CRC-16/CCITT-FALSE over the generated tables, so
# the firmware can check at boot that it runs the calibration the user
# set up for (CalSigExp). test_features.cpp recomputes it from the files.

SIG_FILES = ["src/cal_330e.cpp", "src/cal_f11.cpp", "src/cal_v095.cpp"]


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def gen_sig():
    crc = 0xFFFF
    for rel in SIG_FILES:
        crc = crc16(open(os.path.join(ROOT, rel), "rb").read(), crc)
    write("include/cal_sig.h", [
        lic_text(),
        "#ifndef CAL_SIG_H",
        "#define CAL_SIG_H",
        "",
        "/* Generated by tools/gen_cal.py: CRC-16/CCITT-FALSE over",
        " * " + ", ".join(SIG_FILES) + ".",
        " * Set CalSigExp to this value to have the boot check confirm the",
        " * firmware still carries the calibration it was set up with. */",
        "#define CAL_SIG16 0x%04X" % crc,
        "",
        "#endif // CAL_SIG_H",
        "",
    ])


if __name__ == "__main__":
    main()
