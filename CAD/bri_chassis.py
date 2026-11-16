"""bri_chassis.py - parametric CadQuery model of the BRI v0 wheel-leg test chassis.

What this file is
-----------------
One script that builds every printed part of the v0 chassis, places them (plus simple stand-ins for the
motors, wheels, hubs and standoffs) into an assembly, checks the result, and exports files for printing.

How the robot is laid out (X = forward, Y = left, Z = up, origin = middle of the hip axis)
-------------------------------------------------------------------------------------------
  hip_block        open-top box that holds the two HIP gearmotors end to end; their shafts poke out the sides
  leg (x2)         flat link bolted to a Pololu 6 mm hub on each hip shaft; carries one WHEEL gearmotor
  deck_electronics plate bolted on top of the hip block: motor drivers, regulator, perfboard, IMU pad
  deck_battery     plate on standoffs with slots so the battery tray can slide forward/back
  battery_tray     holds the LiPo; two bolts ride in the deck slots (this is the centre-of-mass adjuster)
  deck_top         interface plate: ODrive M8325s bolt circle, Raspberry Pi hole pattern, ballast cup
  ballast_cup      bolt-on cup filled with steel shot/nuts; stands in for the future jump module mass

Phase 1 (simple balancer): each leg is locked to the hip block with two M3 lock bolts, so the chassis is rigid.
Phase 2 (actuated hips):   remove the lock bolts and the hip motors swing the legs.

Every dimension is a field of `Params`; change a value there and re-run. Units are millimetres.
Run:  python bri_chassis.py            (writes ./export/*.step, *.stl and chassis_report.json)
"""
from dataclasses import dataclass, asdict
import json, math, os
import cadquery as cq


@dataclass
class Params:
    # ---- Pololu 37D gearmotor, measured from the vendor STEP file in Components/ ----
    motor_can_d: float = 36.8        # gearbox/motor can diameter
    motor_len: float = 72.6          # mounting face to back of encoder cap
    motor_shaft_offset: float = 7.0  # output shaft is this far from the can centre
    motor_shaft_proud: float = 22.0  # shaft tip distance from the mounting face
    motor_boss_d: float = 12.0       # round boss around the shaft
    motor_boss_h: float = 6.5
    motor_bolt_circle: float = 31.0  # six M3 holes on this circle (holes sit 30 deg either side of the shaft)
    # ---- Pololu #1999 hub for 6 mm shaft (diameter and thickness from the product page) ----
    hub_d: float = 25.4
    hub_t: float = 9.2
    hub_bolt_circle: float = 19.05   # ASSUMED 0.75 in circle for the six M3 holes - check the Pololu drawing
    # ---- wheels the team already owns ----
    wheel_d: float = 110.0
    wheel_w: float = 45.0
    wheel_gap: float = 3.0           # leg outer face to wheel inner face
    # ---- hip block ----
    wall_t: float = 5.0              # side wall; 5 mm + M3x8 countersunk gives the 3 mm maximum thread engagement
    inner_w: float = 170.0           # inside width (Y); must fit one motor plus one motor-with-shaft for assembly
    inner_half_x: float = 30.0       # inside half depth (X)
    fb_wall_t: float = 3.0           # front/back wall thickness
    base_t: float = 4.0
    base_top_z: float = -14.0        # inside floor height; the can bottom is at -11.4
    top_z: float = 30.0              # top edge of the block; decks sit here
    pillar: float = 9.0              # square corner pillars that take the deck screws and the leg lock bolts
    lock_r: float = 24.0             # leg lock bolts sit on this radius from the hip axis
    # ---- leg ----
    leg_len: float = 90.0            # hip shaft axis to wheel shaft axis
    leg_t: float = 8.0
    leg_gap: float = 1.0             # air gap between hip block wall and leg (fit a 1 mm washer on each lock bolt)
    leg_top_r: float = 30.0
    leg_bot_r: float = 22.0
    lock_angles: tuple = (-40, -20, 0, 20, 40)   # extra lock positions so the leg can be pinned at other angles
    # ---- decks ----
    deck_x: float = 110.0
    deck_y: float = 180.0
    deck_t: float = 4.0
    standoff_len: float = 45.0       # M3 hex standoffs between decks
    standoff_xy: tuple = (48.0, 83.0)
    block_bolt_xy: tuple = (25.5, 80.5)
    # ---- battery (ASSUMED typical 3S 2200 mAh pack; the real pack is still TBD) ----
    batt_l: float = 108.0
    batt_w: float = 36.0
    batt_h: float = 26.0
    tray_travel: float = 30.0        # the tray can slide this far forward or back from centre
    # ---- top deck interfaces ----
    odrive_bolt_circle: float = 40.0 # M8325s mounts on M4 at a 40 mm circle
    pi_holes: tuple = (49.0, 58.0)   # Raspberry Pi mounting rectangle (X, Y), M2.5
    # ---- ballast cup ----
    cup_od: float = 100.0
    cup_h: float = 60.0
    cup_wall: float = 2.4
    cup_floor: float = 4.0
    # ---- hole sizes ----
    m3_clear: float = 3.4
    m3_insert: float = 4.0           # pilot hole for an M3 heat-set insert
    m2_clear: float = 2.4
    m25_clear: float = 2.9
    m4_clear: float = 4.5

    # ---- derived positions (functions so they always follow the numbers above) ----
    @property
    def wall_in_y(self): return self.inner_w / 2                      # inside face of a side wall
    @property
    def wall_out_y(self): return self.inner_w / 2 + self.wall_t       # outside face of a side wall
    @property
    def leg_in_y(self): return self.wall_out_y + self.leg_gap         # leg face nearest the body
    @property
    def leg_out_y(self): return self.leg_in_y + self.leg_t
    @property
    def wheel_centre_y(self): return self.leg_out_y + self.wheel_gap + self.wheel_w / 2
    @property
    def deck_e_z(self): return self.top_z                             # bottom of electronics deck
    @property
    def deck_b_z(self): return self.deck_e_z + self.deck_t + self.standoff_len
    @property
    def deck_t_z(self): return self.deck_b_z + self.deck_t + self.standoff_len
    @property
    def ground_z(self): return -self.leg_len - self.wheel_d / 2       # with the legs straight down


def motor_hole_points(p, shaft_x, shaft_y):
    """Six M3 hole centres of a 37D motor face in a 2D sketch plane.
    The can centre sits `motor_shaft_offset` 'above' (+y of the sketch) the shaft, and the holes are every
    60 degrees around the can centre starting at 0 degrees, which is how the real gearbox is drilled."""
    cx, cy, r = shaft_x, shaft_y + p.motor_shaft_offset, p.motor_bolt_circle / 2
    return [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a))) for a in range(0, 360, 60)]


def hip_block(p):
    """Open-top box for the two hip motors. Printed base-down, no supports.
    Why a single piece: the two side walls must stay square to each other so the two hip shafts stay in line."""
    ox, oy = p.inner_half_x + p.fb_wall_t, p.wall_out_y
    z0 = p.base_top_z - p.base_t
    h = p.top_z - z0
    body = cq.Workplane("XY").workplane(offset=z0).rect(2 * ox, 2 * oy).extrude(h).edges("|Z").fillet(3.0)
    cavity = cq.Workplane("XY").workplane(offset=p.base_top_z).rect(2 * p.inner_half_x, p.inner_w).extrude(h)
    body = body.cut(cavity)
    # Corner pillars: thick meat for the deck screws (from the top) and the leg lock bolts (from the side).
    for sx in (-1, 1):
        for sy in (-1, 1):
            cxp = sx * (p.inner_half_x - p.pillar / 2); cyp = sy * (p.wall_in_y - p.pillar / 2)
            body = body.union(cq.Workplane("XY").workplane(offset=p.base_top_z).center(cxp, cyp)
                              .rect(p.pillar, p.pillar).extrude(p.top_z - p.base_top_z))
    # Saddle ribs: support the back half of each heavy motor so the face screws are not the only support.
    can = cq.Workplane("XZ").center(0, p.motor_shaft_offset).circle(p.motor_can_d / 2 + 0.4).extrude(-200).translate((0, -100, 0))
    for sy in (-1, 1):
        rib = cq.Workplane("XY").workplane(offset=p.base_top_z).center(0, sy * 40.0).rect(2 * p.inner_half_x, 4.0).extrude(11.0)
        body = body.union(rib.cut(can))
    # Zip-tie slots through the floor either side of each rib (tie the motor down onto its saddle).
    slots = [(sx * 24.0, sy * (40.0 + d)) for sx in (-1, 1) for sy in (-1, 1) for d in (-6.0, 6.0)]
    body = body.cut(cq.Workplane("XY").workplane(offset=z0 - 1).pushPoints(slots).rect(8.0, 3.0).extrude(p.base_t + 2))
    # Deck screw pilot holes (M3 heat-set inserts) in the pillar tops.
    bx, by = p.block_bolt_xy
    body = body.cut(cq.Workplane("XY").workplane(offset=p.top_z - 8.0)
                    .pushPoints([(sx * bx, sy * by) for sx in (-1, 1) for sy in (-1, 1)]).circle(p.m3_insert / 2).extrude(9.0))
    # Wiring windows in the front and back walls.
    for sx in (-1, 1):
        for sy in (-1, 1):
            body = body.cut(cq.Workplane("XY").workplane(offset=-4.0).center(sx * (p.inner_half_x + p.fb_wall_t / 2), sy * 36.0)
                            .rect(p.fb_wall_t + 2, 52.0).extrude(24.0))
    # Side-wall features, drilled along Y: shaft boss hole, six countersunk motor screws, two lock-bolt pilots.
    for sy in (-1, 1):
        face_y = sy * p.wall_out_y
        wp = cq.Workplane("XZ", origin=(0, face_y, 0))            # sketch x = world X, sketch y = world Z
        depth_dir = 1 if sy > 0 else -1                           # XZ workplane normal is -Y; extrude into the wall
        boss = wp.circle(p.motor_boss_d / 2 + 0.3).extrude(depth_dir * (p.wall_t + 1))
        screws = wp.pushPoints(motor_hole_points(p, 0, 0)).circle(p.m3_clear / 2).extrude(depth_dir * (p.wall_t + 1))
        # 90-degree countersink so flat-head M3x8 screws sit flush under the leg.
        csk = None
        for (hx, hz) in motor_hole_points(p, 0, 0):
            cone = cq.Solid.makeCone(3.3, 1.6, 1.7, pnt=cq.Vector(hx, face_y, hz), dir=cq.Vector(0, -sy, 0))
            csk = cone if csk is None else csk.fuse(cone)
        locks = wp.pushPoints([(-p.lock_r, 0), (p.lock_r, 0)]).circle(p.m3_insert / 2).extrude(depth_dir * 12.0)
        body = body.cut(boss).cut(screws).cut(cq.Workplane(obj=csk)).cut(locks)
    return body


def leg(p):
    """One leg, modelled lying flat in its print orientation: outer face on the bed (z = 0), inner face up.
    Local frame: hip shaft at the origin, wheel shaft at (0, -leg_len). Left and right legs are the same part."""
    d = p.leg_len - p.motor_shaft_offset            # distance from hip axis to the wheel-motor can centre
    r1, r2 = p.leg_top_r, p.leg_bot_r
    ny = (r1 - r2) / d; nx = math.sqrt(1 - ny * ny)  # direction of the tangent-line normal between the two discs
    web = [(r1 * nx, r1 * ny), (r2 * nx, -d + r2 * ny), (-r2 * nx, -d + r2 * ny), (-r1 * nx, r1 * ny)]
    body = (cq.Workplane("XY").circle(r1).extrude(p.leg_t)
            .union(cq.Workplane("XY").center(0, -d).circle(r2).extrude(p.leg_t))
            .union(cq.Workplane("XY").polyline(web).close().extrude(p.leg_t)))
    top = cq.Workplane("XY").workplane(offset=p.leg_t)
    # Hip end: clearance for the motor boss, six hub screws (heads sunk into the inner face), lock-bolt holes.
    body = body.cut(cq.Workplane("XY").circle(p.motor_boss_d / 2 + 0.5).extrude(p.leg_t))
    hub_pts = [(p.hub_bolt_circle / 2 * math.cos(math.radians(a)), p.hub_bolt_circle / 2 * math.sin(math.radians(a))) for a in range(30, 390, 60)]
    body = body.cut(cq.Workplane("XY").pushPoints(hub_pts).circle(p.m3_clear / 2).extrude(p.leg_t))
    body = body.cut(top.pushPoints(hub_pts).circle(3.1).extrude(-3.4))
    lock_pts = [(p.lock_r * math.cos(math.radians(base + a)), p.lock_r * math.sin(math.radians(base + a)))
                for base in (0, 180) for a in p.lock_angles]
    body = body.cut(cq.Workplane("XY").pushPoints(lock_pts).circle(p.m3_clear / 2).extrude(p.leg_t))
    # Wheel end: boss hole, six motor screws with a 1 mm counterbore on the outer face (M3x10 -> 3 mm engagement).
    body = body.cut(cq.Workplane("XY").center(0, -p.leg_len).circle(p.motor_boss_d / 2 + 0.3).extrude(p.leg_t))
    mpts = motor_hole_points(p, 0, -p.leg_len)
    body = body.cut(cq.Workplane("XY").pushPoints(mpts).circle(p.m3_clear / 2).extrude(p.leg_t))
    body = body.cut(cq.Workplane("XY").pushPoints(mpts).circle(3.25).extrude(1.0))
    # Zip-tie slots for the wheel-motor cable, which must run up the leg with a slack loop at the hip.
    body = body.cut(cq.Workplane("XY").pushPoints([(-9.0, -42.0), (9.0, -42.0)]).rect(3.0, 10.0).extrude(p.leg_t))
    return body


def _deck_blank(p):
    """Common outline and the four standoff holes shared by all three decks."""
    sx, sy = p.standoff_xy
    return (cq.Workplane("XY").rect(p.deck_x, p.deck_y).extrude(p.deck_t).edges("|Z").fillet(6.0)
            .faces(">Z").workplane().pushPoints([(a * sx, b * sy) for a in (-1, 1) for b in (-1, 1)]).hole(p.m3_clear))


def deck_electronics(p):
    """Plate that closes the hip block and carries the electronics. Hole patterns come from the vendor STEP files."""
    d = _deck_blank(p)
    bx, by = p.block_bolt_xy
    d = d.faces(">Z").workplane().pushPoints([(a * bx, b * by) for a in (-1, 1) for b in (-1, 1)]).hole(p.m3_clear)
    pts = []
    # Four G2 18v17 drivers in a row along the back edge (board 33.02 x 20.32, two M2 holes).
    for cy in (-58.0, -20.0, 20.0, 58.0):
        for (u, v) in ((8.25, 2.29), (24.13, 18.03)):
            pts.append((-32.0 - 10.16 + v, cy - 16.51 + u))
    # D24V90F5 regulator (board 40.64 x 20.32, four M2 holes at 35.56 x 15.24).
    for u in (2.54, 38.1):
        for v in (2.54, 17.78):
            pts.append((40.0 - 10.16 + v, -50.0 - 20.32 + u))
    d = d.faces(">Z").workplane().pushPoints(pts).hole(p.m2_clear)
    # 10 mm M3 grid for a perfboard carrying the Teensy and level shifter (the Teensy itself has no mounting holes).
    grid = [(x, y) for x in (10, 20, 30, 40, 50) for y in range(-10, 61, 10)]
    d = d.faces(">Z").workplane().pushPoints(grid).hole(p.m3_clear)
    # Raised IMU pad with two slots: rigid screw mounting for whichever MPU breakout is used.
    pad = cq.Workplane("XY").workplane(offset=p.deck_t).center(-8.0, 0).rect(22.0, 34.0).extrude(2.0)
    d = d.union(pad)
    d = d.cut(cq.Workplane("XY").pushPoints([(-8.0, -10.0), (-8.0, 10.0)]).slot2D(12.0, p.m25_clear, 90).extrude(p.deck_t + 2.0))
    # Cable pass-throughs down into the hip block.
    d = d.cut(cq.Workplane("XY").pushPoints([(-8.0, -62.0), (-8.0, 62.0)]).slot2D(34.0, 14.0, 90).extrude(p.deck_t))
    return d


def deck_battery(p):
    """Plate with two fore-aft slots. The battery tray bolts ride in the slots, which is how the centre of mass is trimmed."""
    d = _deck_blank(p)
    d = d.cut(cq.Workplane("XY").pushPoints([(0, -30.0), (0, 30.0)]).slot2D(2 * p.tray_travel + p.m3_clear, p.m3_clear, 0).extrude(p.deck_t))
    # Lightening / cable windows at both ends.
    d = d.cut(cq.Workplane("XY").pushPoints([(0, -66.0), (0, 66.0)]).rect(60.0, 22.0).extrude(p.deck_t).edges("|Z").fillet(4.0))
    return d


def battery_tray(p):
    """Open tray for the LiPo. Two countersunk M3 screws drop through the floor into the deck slots (wing nuts below)."""
    L, W, fl, lip_h, lip_t = p.batt_l + 8.0, p.batt_w + 2 * 2.5 + 1.0, 3.5, 14.0, 2.5
    t = cq.Workplane("XY").rect(W, L).extrude(fl).edges("|Z").fillet(2.0)
    for sx in (-1, 1):
        t = t.union(cq.Workplane("XY").workplane(offset=fl).center(sx * (W / 2 - lip_t / 2), 0).rect(lip_t, L - 8.0).extrude(lip_h))
    # Strap slots beside each lip for two hook-and-loop straps.
    t = t.cut(cq.Workplane("XY").pushPoints([(sx * (W / 2 - lip_t - 2.5), sy * 38.0) for sx in (-1, 1) for sy in (-1, 1)]).rect(3.0, 22.0).extrude(fl))
    t = t.faces("<Z").workplane().pushPoints([(0, -30.0), (0, 30.0)]).hole(p.m3_clear)
    t = t.cut(cq.Workplane(obj=cq.Solid.makeCone(1.7, 3.3, 1.7, pnt=cq.Vector(0, -30.0, fl - 1.7), dir=cq.Vector(0, 0, 1))
                           .fuse(cq.Solid.makeCone(1.7, 3.3, 1.7, pnt=cq.Vector(0, 30.0, fl - 1.7), dir=cq.Vector(0, 0, 1)))))
    return t


def deck_top(p):
    """Interface plate for what comes later: jump motor (M4 on a 40 mm circle), Raspberry Pi (M2.5), and the ballast cup."""
    d = _deck_blank(p)
    r = p.odrive_bolt_circle / 2
    d = d.faces(">Z").workplane().pushPoints([(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]).hole(p.m4_clear)
    d = d.faces(">Z").workplane().hole(22.0)
    px, py = p.pi_holes
    d = d.faces(">Z").workplane().pushPoints([(a * px / 2, b * py / 2) for a in (-1, 1) for b in (-1, 1)]).hole(p.m25_clear)
    d = d.cut(cq.Workplane("XY").pushPoints([(0, -66.0), (0, 66.0)]).rect(60.0, 22.0).extrude(p.deck_t).edges("|Z").fillet(4.0))
    return d


def ballast_cup(p):
    """Cup that bolts to the jump-motor hole pattern. Fill it with steel shot or nuts to stand in for the jump module mass."""
    c = cq.Workplane("XY").circle(p.cup_od / 2).extrude(p.cup_h)
    c = c.cut(cq.Workplane("XY").workplane(offset=p.cup_floor).circle(p.cup_od / 2 - p.cup_wall).extrude(p.cup_h))
    r = p.odrive_bolt_circle / 2
    c = c.faces("<Z").workplane().pushPoints([(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]).hole(p.m4_clear)
    return c


# ------------------------------------------------------------------------------------------------
# Reference (not printed) stand-ins, used for the picture, the interference check and the mass estimate.
# ------------------------------------------------------------------------------------------------
def motor_envelope(p):
    """Simple solid with the outside size of a 37D gearmotor: can, boss and shaft. Shaft along +Z, face at z = 0,
    shaft axis at the origin, can centre offset +Y (same convention as motor_hole_points)."""
    can = cq.Workplane("XY").center(0, p.motor_shaft_offset).circle(p.motor_can_d / 2).extrude(-p.motor_len)
    boss = cq.Workplane("XY").circle(p.motor_boss_d / 2).extrude(p.motor_boss_h)
    shaft = cq.Workplane("XY").circle(3.0).extrude(p.motor_shaft_proud)
    return can.union(boss).union(shaft)


def place_motor(p, side, face_y, shaft_z):
    """Put a motor envelope with its shaft pointing outward (+Y for side=+1, -Y for side=-1),
    mounting face at |y| = face_y, shaft axis at height shaft_z, can centre above the shaft."""
    m = motor_envelope(p)
    if side > 0:
        m = m.rotate((0, 0, 0), (1, 0, 0), -90).rotate((0, 0, 0), (0, 1, 0), 180)
    else:
        m = m.rotate((0, 0, 0), (1, 0, 0), 90)
    return m.translate((0, side * face_y, shaft_z))


def place_leg(p, side, beta_deg=0.0):
    """Stand a flat-modelled leg up beside the hip block. beta_deg swings it about the hip axis (0 = straight down)."""
    l = leg(p).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, p.leg_out_y, 0))   # left side: outer face away from body
    if side < 0:
        l = l.rotate((0, 0, 0), (0, 0, 1), 180)                                  # the same part turned around
    return l.rotate((0, 0, 0), (0, 1, 0), beta_deg)


def build(p, beta_deg=0.0, with_ballast=True):
    """Return {name: (shape, kind, mass_g)} for the whole assembly. kind is 'print' or 'ref'.
    Reference masses are estimates until the real parts are weighed (see MASS_NOTES below)."""
    A = {}
    A["hip_block"] = (hip_block(p), "print", None)
    A["deck_electronics"] = (deck_electronics(p).translate((0, 0, p.deck_e_z)), "print", None)
    A["deck_battery"] = (deck_battery(p).translate((0, 0, p.deck_b_z)), "print", None)
    A["battery_tray"] = (battery_tray(p).rotate((0, 0, 0), (0, 0, 1), 0).translate((0, 0, p.deck_b_z + p.deck_t)), "print", None)
    A["deck_top"] = (deck_top(p).translate((0, 0, p.deck_t_z)), "print", None)
    if with_ballast:
        A["ballast_cup"] = (ballast_cup(p).translate((0, 0, p.deck_t_z + p.deck_t)), "print", None)
    for name, side in (("L", 1), ("R", -1)):
        A[f"leg_{name}"] = (place_leg(p, side, beta_deg), "print", None)
        A[f"hip_motor_{name}"] = (place_motor(p, side, p.wall_in_y, 0.0), "ref", 210.0)
        wm = place_motor(p, side, p.leg_in_y, -p.leg_len).rotate((0, 0, 0), (0, 1, 0), beta_deg)
        A[f"wheel_motor_{name}"] = (wm, "ref", 210.0)
        wheel = (cq.Workplane("XZ").circle(p.wheel_d / 2).circle(8.0).extrude(p.wheel_w)
                 .translate((0, p.wheel_w / 2, 0)).translate((0, side * p.wheel_centre_y, -p.leg_len)).rotate((0, 0, 0), (0, 1, 0), beta_deg))
        A[f"wheel_{name}"] = (wheel, "ref", 130.0)
        hub = (cq.Workplane("XZ").circle(p.hub_d / 2).circle(3.0).extrude(p.hub_t).translate((0, p.hub_t, 0))
               .translate((0, p.leg_out_y, 0)))
        if side < 0:
            hub = hub.rotate((0, 0, 0), (0, 0, 1), 180)
        A[f"hip_hub_{name}"] = (hub, "ref", 7.0)
    # Battery (assumed pack) sitting in the tray, long side across the robot.
    batt = cq.Workplane("XY").rect(p.batt_w, p.batt_l).extrude(p.batt_h).translate((0, 0, p.deck_b_z + p.deck_t + 3.5))
    A["battery"] = (batt, "ref", 180.0)
    # Eight M3 hex standoffs (two levels x four corners).
    sx, sy = p.standoff_xy
    for lvl, z in enumerate((p.deck_e_z + p.deck_t, p.deck_b_z + p.deck_t)):
        for i, (a, b) in enumerate([(a, b) for a in (-1, 1) for b in (-1, 1)]):
            so = cq.Workplane("XY").polygon(6, 6.0).extrude(p.standoff_len).translate((a * sx, b * sy, z))
            A[f"standoff_{lvl}{i}"] = (so, "ref", 2.5)
    return A


MASS_NOTES = {
    "print_density_g_cm3": 1.27,     # PETG
    "print_fill_factor": 0.55,       # ASSUMED effective solid fraction (4 walls + 30 % infill); weigh a printed part to correct
    "electronics_g": 150.0,          # ASSUMED: Teensy, drivers, regulator, perfboard, wiring; lumped on the electronics deck
    "fasteners_g": 60.0,             # ASSUMED; lumped at the body centre
    "ballast_g": 1500.0,             # target fill of the ballast cup
}


def mass_report(p, A):
    """Estimate mass and centre of mass of the 'body' (everything above the hips) and of each leg assembly.
    The hip joint analysis needs two numbers from here: body mass and how far its centre of mass is above the hip axis."""
    rho = MASS_NOTES["print_density_g_cm3"] * MASS_NOTES["print_fill_factor"] / 1000.0   # g per mm^3
    rows = []
    for name, (shape, kind, m) in A.items():
        s = shape.val()
        mass = s.Volume() * rho if kind == "print" else m
        c = cq.Shape.centerOfMass(s)
        rows.append(dict(name=name, kind=kind, mass_g=round(mass, 1), x=round(c.x, 1), y=round(c.y, 1), z=round(c.z, 1)))
    rows.append(dict(name="electronics", kind="ref", mass_g=MASS_NOTES["electronics_g"], x=0.0, y=0.0, z=p.deck_e_z + 12.0))
    rows.append(dict(name="fasteners", kind="ref", mass_g=MASS_NOTES["fasteners_g"], x=0.0, y=0.0, z=40.0))
    if "ballast_cup" in A:
        rows.append(dict(name="ballast_fill", kind="ref", mass_g=MASS_NOTES["ballast_g"], x=0.0, y=0.0,
                         z=p.deck_t_z + p.deck_t + p.cup_floor + 20.0))
    is_leg = lambda n: n.startswith(("leg_", "wheel_", "hip_hub_"))
    def lump(sel):
        mm = sum(r["mass_g"] for r in rows if sel(r["name"]))
        zz = sum(r["mass_g"] * r["z"] for r in rows if sel(r["name"])) / mm
        return round(mm, 1), round(zz, 1)
    body_m, body_z = lump(lambda n: not is_leg(n))
    legs_m, legs_z = lump(is_leg)
    return dict(parts=rows, body_mass_g=body_m, body_com_z_above_hip_mm=body_z, both_legs_mass_g=legs_m,
                legs_com_z_mm=legs_z, total_mass_g=round(body_m + legs_m, 1))


def interference(A, tol_mm3=0.5):
    """List every pair of solids that overlap by more than tol_mm3. An empty list means nothing collides."""
    names = list(A); hits = []
    boxes = {n: A[n][0].val().BoundingBox() for n in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ba, bb = boxes[a], boxes[b]
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue                                   # bounding boxes do not touch: skip the slow solid test
            try:
                v = A[a][0].val().intersect(A[b][0].val()).Volume()
            except Exception:
                v = float("nan")
            if not (v <= tol_mm3):
                hits.append((a, b, round(v, 2)))
    return hits


PRINT_PARTS = {   # name -> (builder, how many to print, print notes)
    "hip_block": (hip_block, 1, "base down, no supports; 4 walls, 30 % infill"),
    "leg": (leg, 2, "outer face down (as exported), no supports; 5 walls, 40 % infill; left and right are the same part"),
    "deck_electronics": (deck_electronics, 1, "flat, IMU pad up"),
    "deck_battery": (deck_battery, 1, "flat"),
    "battery_tray": (battery_tray, 1, "floor down"),
    "deck_top": (deck_top, 1, "flat"),
    "ballast_cup": (ballast_cup, 1, "floor down; 3 walls is enough"),
}


def export(p, outdir="export"):
    """Write one STEP and one STL per printed part (already in print orientation), the assembly STEP, and a JSON report."""
    os.makedirs(outdir, exist_ok=True)
    report = dict(params=asdict(p), derived=dict(track_mm=round(2 * p.wheel_centre_y, 1),
                  outside_width_mm=round(2 * p.wheel_centre_y + p.wheel_w, 1),
                  hip_axis_height_mm=round(-p.ground_z, 1), top_deck_height_mm=round(p.deck_t_z + p.deck_t - p.ground_z, 1)), parts={})
    for name, (fn, qty, note) in PRINT_PARTS.items():
        s = fn(p); v = s.val(); bb = v.BoundingBox()
        s = s.translate((0, 0, -bb.zmin))                   # sit the part on the bed
        cq.exporters.export(s, os.path.join(outdir, f"{name}.step"))
        cq.exporters.export(s, os.path.join(outdir, f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        report["parts"][name] = dict(qty=qty, print_notes=note, valid=bool(v.isValid()), solids=len(s.solids().vals()),
                                     volume_cm3=round(v.Volume() / 1000, 1), bbox_mm=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
    A = build(p)
    asm = cq.Assembly(name="bri_v0_chassis")
    colours = {"print": cq.Color(0.20, 0.45, 0.70), "ref": cq.Color(0.6, 0.6, 0.6)}
    for name, (shape, kind, _) in A.items():
        asm.add(shape, name=name, color=colours[kind])
    asm.save(os.path.join(outdir, "bri_v0_assembly.step"))
    report["interference"] = interference(A)
    report["mass_with_ballast"] = mass_report(p, A)
    report["mass_no_ballast"] = mass_report(p, build(p, with_ballast=False))
    report["mass_notes"] = MASS_NOTES
    json.dump(report, open(os.path.join(outdir, "chassis_report.json"), "w"), indent=1)
    return report


if __name__ == "__main__":
    r = export(Params())
    for n, d in r["parts"].items():
        print(f"{n:18s} x{d['qty']}  valid={d['valid']} solids={d['solids']}  {d['volume_cm3']:6.1f} cm3  bbox {d['bbox_mm']}")
    print("derived:", r["derived"])
    print("interference:", r["interference"])
    for k in ("mass_no_ballast", "mass_with_ballast"):
        m = r[k]; print(k, {kk: m[kk] for kk in m if kk != "parts"})
