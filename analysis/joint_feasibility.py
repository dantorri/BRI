"""joint_feasibility.py - static checks for adding hip joints driven by the two spare Pololu 4756 gearmotors.

Why this file exists
--------------------
Before printing legs we want to know, with simple statics, what an actuated hip can and cannot do:
  1. How tall a step the bare wheels can roll over (so we know what the joint has to add).
  2. How much torque each hip needs to hold the body, and how that compares with the gearbox limits.
  3. How far the leg can swing before the body can no longer keep its centre of mass over the axle.
  4. What height and side-to-side levelling range that gives.
Every input is a named parameter so the numbers can be re-run when the real masses are weighed.
Run:  python joint_feasibility.py
"""
import math, csv

G = 9.81
# ---- Motor limits (Pololu 4756, from the project workbook) -------------------------------------
TAU_CONT = 10 * 0.0980665     # N*m, recommended continuous gearbox torque (10 kg*cm)
TAU_INST = 25 * 0.0980665     # N*m, recommended instantaneous gearbox torque (25 kg*cm)
RPM_NOLOAD = 76.0             # output speed at 12 V
COUNTS_PER_REV = 8400         # encoder counts per output revolution

def rolling_step_limit(wheel_radius_m, mu):
    """Tallest step a wheel can climb slowly by traction alone.
    At the step edge the push comes from friction, so the contact normal may lean at most atan(mu)
    from vertical. Geometry then gives h = R * (1 - cos(atan(mu)))."""
    return wheel_radius_m * (1 - math.cos(math.atan(mu)))

def hip_torque(body_mass_kg, leg_len_m, beta_deg):
    """Torque per hip (two hips share the load) to hold the leg at beta from straight-down while balancing.
    Balancing puts the body's centre of mass over the axle, which sits L*sin(beta) ahead of the hip,
    so gravity on the body acts on that lever arm."""
    return body_mass_kg * G * leg_len_m * math.sin(math.radians(beta_deg)) / 2

def max_beta_for_balance(body_com_above_hip_m, leg_len_m):
    """Largest leg angle at which the body can still lean its centre of mass over the axle.
    The body can shift its mass at most d (its COM distance from the hip) sideways, so L*sin(beta) <= d."""
    return math.degrees(math.asin(min(1.0, body_com_above_hip_m / leg_len_m)))

def analyse(body_mass_kg, body_com_above_hip_m, leg_mass_kg, leg_len_m, wheel_radius_m, track_m, label):
    """Return one row of results for a given mass case."""
    bmax_geo = max_beta_for_balance(body_com_above_hip_m, leg_len_m)
    # largest angle the continuous torque rating can hold
    s = TAU_CONT * 2 / (body_mass_kg * G * leg_len_m)
    bmax_tau = math.degrees(math.asin(min(1.0, s)))
    bmax = min(bmax_geo, bmax_tau)
    dz = leg_len_m * (1 - math.cos(math.radians(bmax)))          # height one leg can give up
    return dict(case=label, body_mass_kg=round(body_mass_kg, 3), body_com_above_hip_mm=round(body_com_above_hip_m*1e3),
        leg_len_mm=round(leg_len_m*1e3),
        hip_torque_at_20deg_Nm=round(hip_torque(body_mass_kg, leg_len_m, 20), 2),
        hip_torque_at_45deg_Nm=round(hip_torque(body_mass_kg, leg_len_m, 45), 2),
        beta_max_geometry_deg=round(bmax_geo, 1), beta_max_cont_torque_deg=round(bmax_tau, 1),
        usable_beta_deg=round(bmax, 1), height_range_mm=round(dz*1e3, 1),
        cross_slope_levelled_deg=round(math.degrees(math.atan(dz / track_m)), 1),
        self_right_torque_per_hip_Nm=round(body_mass_kg * G * body_com_above_hip_m / 2, 2),
        leg_swing_torque_Nm=round(leg_mass_kg * G * leg_len_m, 2),
        hip_axis_height_tall_mm=round((wheel_radius_m + leg_len_m)*1e3))

def cases_from_cad_report(path):
    """Build the two mass cases (with and without ballast) from the chassis report written by CAD/bri_chassis.py,
    so the joint numbers always follow the current CAD model instead of hand-typed guesses."""
    import json
    r = json.load(open(path))
    L = r["params"]["leg_len"] / 1000.0
    out = []
    for key, label in (("mass_no_ballast", "no ballast (CAD mass model)"), ("mass_with_ballast", "1.5 kg ballast (CAD mass model)")):
        m = r[key]
        out.append(dict(label=label, body_mass_kg=m["body_mass_g"] / 1000.0, body_com_above_hip_m=m["body_com_z_above_hip_mm"] / 1000.0,
                        leg_mass_kg=m["both_legs_mass_g"] / 2000.0, leg_len_m=L,
                        wheel_radius_m=r["params"]["wheel_d"] / 2000.0, track_m=r["derived"]["track_mm"] / 1000.0))
    return out

if __name__ == "__main__":
    import os, sys
    here = os.path.dirname(os.path.abspath(__file__))
    report = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "CAD", "export", "chassis_report.json")
    cases = cases_from_cad_report(report)
    rows = [analyse(**c) for c in cases]
    with open(os.path.join(here, "joint_feasibility.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    for r in rows:
        print(r["case"]); [print(f"   {k}: {v}") for k, v in r.items() if k != "case"]
    print("Rolling-only step limit for a 110 mm wheel:")
    for mu in (0.6, 0.8, 1.0):
        print(f"   mu={mu}: {rolling_step_limit(0.055, mu)*1e3:.1f} mm")
    print(f"Motor limits: continuous {TAU_CONT:.2f} N*m, instantaneous {TAU_INST:.2f} N*m per hip")
    print(f"Hip speed {RPM_NOLOAD*6:.0f} deg/s no-load; resolution {360/COUNTS_PER_REV:.3f} deg per count")
