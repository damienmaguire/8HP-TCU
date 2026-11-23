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
Generate the C calibration tables the TCU firmware needs from Ian's export
of the F30 330e GA8P75HZ EGS calibration (330e_sas_eds_cal.json).

Only the items the first cut uses are emitted: the pressure-to-current
curves, the per-clutch fill data, line pressure, N->1/N->R gear-in data,
and the shift data for the seven adjacent upshifts (12..78) and seven
adjacent downshifts (21..87). Every table carries its source name and flash
address in a comment, so values can be traced back to the OEM image.

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

Usage:
  python3 tools/gen_cal.py <path to 330e_sas_eds_cal.json> [--set normal|el]

--set el uses the "_EL" (electric-drive) copy of a table where one exists
and falls back to the normal table otherwise (open question Q7 in Ian's
spec). The default is the Normal programme set.
"""
import argparse
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
    """Scale factor from the A2L physical unit to the firmware integer unit."""
    if conv.startswith("UMR_BAR_SEC"):
        return 1000.0          # bar/s -> mbar/s
    if conv.startswith("UMR_BAR__"):
        return 1000.0          # bar -> mbar
    if conv.startswith("UMR_MA__"):
        return 10.0            # mA -> 0.1 mA
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
        self.out.append("static const %s %s[] = {%s};" % (ctype, cname, body))

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("--set", choices=["normal", "el"], default="normal")
    args = ap.parse_args()
    items = json.load(open(args.json))
    g = Gen(items, args.set == "el")

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

    gen_note = ("/* GENERATED by tools/gen_cal.py from 330e_sas_eds_cal.json "
                "(programme set: %s). Do not edit by hand. */" % args.set)

    lic = open(os.path.join(ROOT, "tools", "license_header.txt")).read().rstrip("\n")
    hdr = [lic, gen_note, "#ifndef CAL_330E_H", "#define CAL_330E_H", "",
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
            "extern const GlobalCal calGlobal;", "",
            "#endif // CAL_330E_H", ""]

    src = [lic, gen_note, '#include "cal_330e.h"', ""] + g.out + [""]
    src.append("const UpShiftCal calUp[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["up"]) + "\n};\n")
    src.append("const DownShiftCal calDown[CAL_NUM_PAIRS] = {\n" + ",\n".join(inits["down"]) + "\n};\n")
    src.append("const ClutchFillCal calClutch[5] = {\n" + ",\n".join(inits["clutch"]) + "\n};\n")
    src.append("const EngageCal calEngage[2] = {\n" + ",\n".join(inits["engage"]) + "\n};\n")
    src.append("const CalCurve calEds[7] = {\n   " + ",\n   ".join(eds) + "\n};\n")
    src.append("const CalMap calPsys[9] = {\n   " + ",\n   ".join(psys) + "\n};\n")
    src.append("const GlobalCal calGlobal = {\n" + "\n".join(glob_parts) + "\n};\n")
    src.append("// %d calibration values from %d items.\n" % (g.count, len(g.used)))

    with open(os.path.join(ROOT, "include", "cal_330e.h"), "w") as f:
        f.write("\n".join(hdr))
    with open(os.path.join(ROOT, "src", "cal_330e.cpp"), "w") as f:
        f.write("\n".join(src))
    print("wrote %d values from %d items" % (g.count, len(g.used)))


if __name__ == "__main__":
    main()
