"""Simplified motorized fader (60 mm travel) - reference/keep-out geometry.

Coordinate system:
  origin = center of the fader frame (midway between the two M3 mounting holes)
  +X     = across the fader (frame width)
  +Y     = down, from the knob stem toward the PCB pins
  +Z     = along the travel, toward the terminal/pin block (motor is at -Z)
Units: mm

Usage:
  python motor_fader_60mm.py [--stem-pos MM] [--no-pins] [--pcb full|transparent|off] [-o FILE]

Exports a STEP assembly with a named, colored part for each component. The
"transparent" PCB sets an alpha on the board's color; the STEP writer keeps
only the RGB, so set transparency in the receiving CAD tool if needed.
"""

import argparse

from build123d import (
    Box,
    Circle,
    Color,
    Compound,
    Cylinder,
    Part,
    Plane,
    Polygon,
    Pos,
    Rectangle,
    Rot,
    Sketch,
    export_step,
    extrude,
    fillet,
)

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------

C_STEEL = (0.72, 0.73, 0.72)    # frame, stem, motor bracket parts
C_MESH = (0.50, 0.51, 0.52)     # dust cover over the slot
C_CAN = (0.80, 0.80, 0.78)      # motor can
C_PLASTIC = (0.58, 0.55, 0.51)  # gear housing, motor plate, terminal block
C_PIN = (0.86, 0.86, 0.83)      # tinned pins and tabs
C_PCB = (0.12, 0.42, 0.20)      # reference PCB (solder mask green)
PCB_TRANSPARENT_ALPHA = 0.35

# ---------------------------------------------------------------------------
# Key nominal dimensions
# ---------------------------------------------------------------------------

FRAME_Y_TOP = -4.9                  # top face of the frame
BODY_Y_BOT = FRAME_Y_TOP + 26.0     # bottom of motor plate / terminal block
PIN_Y_BOT = BODY_Y_BOT + 3.5        # PCB pin length
FRAME_Z_MIN = -52.75                # motor end, incl. gear housing
FRAME_Z_MAX = 54.0                  # terminal end

# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

# Top cover: forms the top of the frame
COVER_X = 15.0
COVER_Y_BOT = 0.0
COVER_Z_MIN = -50.0
COVER_TAB_X = (6.9, 7.4)            # side tabs folded under the chassis
COVER_TAB_Y = 3.1
COVER_TAB_Z = 25.0                  # +/-
COVER_TAB_W = 3.3

# Dust mesh over the slot, sitting on the cover
MESH_X = 13.2
MESH_Y_TOP = -5.05
MESH_Z = 43.0                       # +/-
MESH_NOTCH_D = 5.0                  # clearance notch around each mounting hole

# Chassis: lower bracket holding the slider (keep-out envelope)
CHASSIS_X = 17.2
CHASSIS_Y_TOP = -3.6
CHASSIS_Y_BOT = 1.7                 # underside of the frame
CHASSIS_Z = 38.0                    # +/-

# M3 mounting holes in the top face
MOUNT_HOLE_Z = 40.0                 # +/- from frame center (80 mm spacing)
MOUNT_HOLE_D = 3.0
MOUNT_HOLE_Y_BOT = -3.5             # bottom of the tapped bore

# Stem (simplified to a flat blade)
STEM_T = 1.2                        # thickness (X)
STEM_W = 8.0                        # width (Z)
STEM_Y_TOP = FRAME_Y_TOP - 9.0      # straight stem, 9.0 above the frame top

# Motor: can and bosses are coaxial on this line, parallel to Z
MOTOR_AXIS_Y = 9.0

# Gear housing at the far end of the motor
GEARBOX_X = 13.0
GEARBOX_Y_TOP = 3.75
GEARBOX_Y_BOT = 14.95
GEARBOX_Z_MAX = -39.65

# Belt pulley, axis along X, between the frame and the gear housing
PULLEY_Y = 0.925
PULLEY_Z = -44.95
PULLEY_R = 3.625
PULLEY_X = 12.5

# Motor can: cylinder with flats top and bottom, filleted where they meet
CAN_R = 7.75
CAN_Y_TOP = 3.0                     # flat facing the frame
CAN_Y_BOT = 15.0                    # flat facing the PCB
CAN_Z_MIN = -37.35
CAN_Z_MAX = -10.95
CAN_FILLET = 1.0
CAN_BOSS_R = 2.375                  # shaft boss toward the gear housing
CAN_BOSS_R2 = 4.0                   # bearing boss toward the motor plate

# Strap under the can
STRAP_X = 10.0
STRAP_Y_BOT = 15.8
STRAP_Z_MIN = -37.85
STRAP_Z_MAX = -10.85

# Motor mounting plate (carries the motor pins)
MOTOR_PLATE_X = 18.0
MOTOR_PLATE_Y_TOP = 5.5
MOTOR_PLATE_Z_MIN = -10.45
MOTOR_PLATE_Z_MAX = -8.55
MOTOR_PLATE_R = 1.0
MOTOR_PLATE_NOTCH_X = 15.8          # width at the side notches
MOTOR_PLATE_NOTCH_Y = (12.5, 16.5)

# Small drive housing between the motor plate and the slider
DRIVE_X = 7.0
DRIVE_Y_BOT = 7.8
DRIVE_Z_MAX = -3.75

# Terminal block: tall body plus a lower shelf the terminals pass through
TERM_X = 14.7
TERM_Y_TOP = 2.1
TERM_Z_MIN = 35.05
TERM_Z_MAX = 46.05
TERM_SHELF_Z_MIN = 43.45
TERM_SHELF_Y_TOP = 14.4

# Motor pins A/B
MOTOR_PIN_Z = -9.55
MOTOR_PIN_X = 4.0                   # +/- (8.0 spacing)
MOTOR_PIN_W = 1.0                   # X
MOTOR_PIN_T = 0.4                   # Z
MOTOR_PIN_TIP = 1.0                 # chamfered tip length

# Fader terminals 1, T, 3, 2: wide strip exposed above the shelf, narrow
# solder pin below it
TERM_PIN_Z = 43.8
TERM_PIN_XS = (-5.625, -1.875, 1.875, 5.625)  # 3.75 pitch
TERM_PIN_W = 1.0                    # X, solder pin
TERM_PIN_W_TOP = 2.0                # X, exposed strip above the shelf
TERM_PIN_T = 0.5                    # Z
TERM_PIN_Y_BOT = PIN_Y_BOT

# Reference PCB: top surface against the motor plate / terminal block
PCB_T = 1.6
PCB_MARGIN = 5.0                    # board extends this far beyond the fader outline
PCB_HOLE_D = 1.3                    # motor pins and fader terminals
PCB_TAB_HOLE_D = 1.5                # frame mounting tabs
PCB_TAB_HOLE_X = 5.625              # +/-, in line with terminals 1 and 2

# Frame mounting tabs: straight leg, bent outward, then back in toward the
# tip. Positions are the inner (-X side) edge of the +X tab.
TAB_Z = 36.65
TAB_W = 1.2                         # Z
TAB_T = 0.4                         # X
TAB_Y_BOT = BODY_Y_BOT + 4.0        # tip
TAB_LEG_X = 5.6
TAB_BEND_Y = 22.3                   # start of the outward bend
TAB_APEX = (13.7 / 2 - TAB_T, 23.7)  # outermost extent 13.7 across both tabs
TAB_TIP_X = 11.7 / 2                # innermost tip spacing 11.7 across both tabs
TAB_PATH = [(TAB_LEG_X, BODY_Y_BOT), (TAB_LEG_X, TAB_BEND_Y), TAB_APEX,
            (TAB_TIP_X, TAB_Y_BOT)]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def box(lo, hi) -> Part:
    """Axis-aligned box from min/max corners."""
    size = [h - l for l, h in zip(lo, hi)]
    center = [(l + h) / 2 for l, h in zip(lo, hi)]
    return Pos(*center) * Box(*size)


def xbox(w, y0, y1, z0, z1) -> Part:
    """Box centered in X, spanning the given Y and Z ranges."""
    return box((-w / 2, y0, z0), (w / 2, y1, z1))


def zprism(profile: Sketch, z0, z1) -> Part:
    """Extrude a 2D XY profile along Z between z0 and z1."""
    return Pos(0, 0, z0) * extrude(profile, amount=z1 - z0)


def xrect(w, y0, y1, r0=0.0, r1=0.0) -> Sketch:
    """2D rectangle centered in X from y0 to y1, with optional corner radii on
    the y0 side (r0) and the y1 side (r1)."""
    rect = Pos(0, (y0 + y1) / 2) * Rectangle(w, y1 - y0)
    for r, y in ((r0, y0), (r1, y1)):
        if r > 0:
            rect = fillet(rect.vertices().filter_by(lambda v: abs(v.Y - y) < 1e-6), r)
    return rect


def ycyl(d, y0, y1, x=0.0, z=0.0) -> Part:
    """Cylinder along Y from y0 to y1."""
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(d / 2, y1 - y0)


def motor_cyl(r, z0, z1) -> Part:
    """Cylinder along Z on the motor axis."""
    return Pos(0, MOTOR_AXIS_Y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)


def mirror_x_pair(shape):
    """The shape plus its mirror across the YZ plane."""
    return [shape, shape.mirror(Plane.YZ)]


def leaf(shape, label, rgb, alpha=1.0):
    """Name and color a part for the STEP assembly."""
    shape.label = label
    shape.color = Color(*rgb, alpha)
    return shape


def group(label, children):
    return Compound(label=label, children=children)


# ---------------------------------------------------------------------------
# Frame
# ---------------------------------------------------------------------------


def mount_holes() -> Part:
    return sum((ycyl(MOUNT_HOLE_D, MESH_Y_TOP - 0.01, MOUNT_HOLE_Y_BOT, z=z)
                for z in (-MOUNT_HOLE_Z, MOUNT_HOLE_Z)), start=Part())


def cover() -> Part:
    shape = xbox(COVER_X, FRAME_Y_TOP, COVER_Y_BOT, COVER_Z_MIN, FRAME_Z_MAX)
    for sz in (-1, 1):
        tab = box((COVER_TAB_X[0], CHASSIS_Y_BOT, sz * COVER_TAB_Z - COVER_TAB_W / 2),
                  (COVER_TAB_X[1], COVER_TAB_Y, sz * COVER_TAB_Z + COVER_TAB_W / 2))
        for t in mirror_x_pair(tab):
            shape += t
    return shape - mount_holes()


def mesh() -> Part:
    shape = xbox(MESH_X, MESH_Y_TOP, FRAME_Y_TOP, -MESH_Z, MESH_Z)
    # U-shaped notches around the mounting holes, open to the mesh ends
    for sz in (-1, 1):
        y0, y1 = MESH_Y_TOP - 0.01, FRAME_Y_TOP + 0.01
        shape -= ycyl(MESH_NOTCH_D, y0, y1, z=sz * MOUNT_HOLE_Z)
        z_end = sz * (MESH_Z + 1)
        shape -= box((-MESH_NOTCH_D / 2, y0, min(sz * MOUNT_HOLE_Z, z_end)),
                     (MESH_NOTCH_D / 2, y1, max(sz * MOUNT_HOLE_Z, z_end)))
    return shape


def chassis() -> Part:
    return xbox(CHASSIS_X, CHASSIS_Y_TOP, CHASSIS_Y_BOT, -CHASSIS_Z, CHASSIS_Z) - mount_holes()


def frame():
    return group("frame", [
        leaf(cover(), "cover", C_STEEL),
        leaf(mesh(), "dust_mesh", C_MESH),
        leaf(chassis(), "chassis", C_STEEL),
    ])


# ---------------------------------------------------------------------------
# Stem
# ---------------------------------------------------------------------------


def stem(stem_pos):
    """Knob stem, positioned along the travel by stem_pos."""
    shape = box((-STEM_T / 2, STEM_Y_TOP, stem_pos - STEM_W / 2),
                (STEM_T / 2, MESH_Y_TOP, stem_pos + STEM_W / 2))
    return leaf(shape, "stem", C_STEEL)


# ---------------------------------------------------------------------------
# Motor (hangs below the frame at the -Z end)
# ---------------------------------------------------------------------------


def gearbox():
    shape = xbox(GEARBOX_X, GEARBOX_Y_TOP, GEARBOX_Y_BOT, FRAME_Z_MIN, GEARBOX_Z_MAX)
    return leaf(shape, "gearbox", C_PLASTIC)


def pulley():
    shape = Pos(0, PULLEY_Y, PULLEY_Z) * Rot(0, 90, 0) * Cylinder(PULLEY_R, PULLEY_X)
    return leaf(shape, "pulley", C_STEEL)


def motor_can():
    profile = (Pos(0, MOTOR_AXIS_Y) * Circle(CAN_R)) & xrect(2 * CAN_R, CAN_Y_TOP, CAN_Y_BOT)
    profile = fillet(profile.vertices(), CAN_FILLET)
    shape = zprism(profile, CAN_Z_MIN, CAN_Z_MAX)
    shape += motor_cyl(CAN_BOSS_R, GEARBOX_Z_MAX, CAN_Z_MIN)
    shape += motor_cyl(CAN_BOSS_R2, CAN_Z_MAX, MOTOR_PLATE_Z_MIN)
    return leaf(shape, "motor_can", C_CAN)


def strap():
    shape = xbox(STRAP_X, CAN_Y_BOT, STRAP_Y_BOT, STRAP_Z_MIN, STRAP_Z_MAX)
    return leaf(shape, "strap", C_STEEL)


def motor_plate():
    profile = xrect(MOTOR_PLATE_X, MOTOR_PLATE_Y_TOP, BODY_Y_BOT, r0=MOTOR_PLATE_R)
    # side notches
    notch_h = MOTOR_PLATE_NOTCH_Y[1] - MOTOR_PLATE_NOTCH_Y[0]
    notch_y = (MOTOR_PLATE_NOTCH_Y[0] + MOTOR_PLATE_NOTCH_Y[1]) / 2
    for sx in (-1, 1):
        profile -= Pos(sx * (MOTOR_PLATE_NOTCH_X / 2 + MOTOR_PLATE_X / 2), notch_y) \
            * Rectangle(MOTOR_PLATE_X, notch_h)
    shape = zprism(profile, MOTOR_PLATE_Z_MIN, MOTOR_PLATE_Z_MAX)
    return leaf(shape, "motor_plate", C_PLASTIC)


def drive_housing():
    shape = xbox(DRIVE_X, CHASSIS_Y_BOT, DRIVE_Y_BOT, MOTOR_PLATE_Z_MAX, DRIVE_Z_MAX)
    return leaf(shape, "drive_housing", C_PLASTIC)


# Each pin is built separately (not as a moved copy) so every one keeps its own
# name in the STEP file instead of becoming an instance of a shared shape.

def motor_pin(x) -> Part:
    w, tip = MOTOR_PIN_W, MOTOR_PIN_TIP
    outline = Polygon(
        (x - w / 2, BODY_Y_BOT), (x + w / 2, BODY_Y_BOT), (x + w / 2, PIN_Y_BOT - tip),
        (x + w / 4, PIN_Y_BOT), (x - w / 4, PIN_Y_BOT), (x - w / 2, PIN_Y_BOT - tip),
        align=None)
    return zprism(outline, MOTOR_PIN_Z - MOTOR_PIN_T / 2, MOTOR_PIN_Z + MOTOR_PIN_T / 2)


def motor_pins():
    return [leaf(motor_pin(sx * MOTOR_PIN_X), f"motor_pin_{name}", C_PIN)
            for sx, name in ((1, "A"), (-1, "B"))]


def motor(show_pins):
    children = [gearbox(), pulley(), motor_can(), strap(), motor_plate(), drive_housing()]
    if show_pins:
        children += motor_pins()
    return group("motor", children)


# ---------------------------------------------------------------------------
# Terminal block (hangs below the frame at the +Z end)
# ---------------------------------------------------------------------------


def terminal_body():
    shape = xbox(TERM_X, TERM_Y_TOP, BODY_Y_BOT, TERM_Z_MIN, TERM_SHELF_Z_MIN)
    shape += xbox(TERM_X, TERM_SHELF_Y_TOP, BODY_Y_BOT, TERM_SHELF_Z_MIN, TERM_Z_MAX)
    return leaf(shape, "terminal_block", C_PLASTIC)


def terminal_pin(x) -> Part:
    """Exposed strip above the shelf plus the solder pin below the block."""
    z0, z1 = TERM_PIN_Z - TERM_PIN_T / 2, TERM_PIN_Z + TERM_PIN_T / 2
    upper = zprism(Pos(x, 0) * xrect(TERM_PIN_W_TOP, COVER_Y_BOT, TERM_SHELF_Y_TOP), z0, z1)
    lower = zprism(Pos(x, 0) * xrect(TERM_PIN_W, BODY_Y_BOT, TERM_PIN_Y_BOT,
                                     r1=TERM_PIN_W / 2), z0, z1)
    return upper + lower


def terminal_pins():
    return [leaf(terminal_pin(x), f"terminal_{i}", C_PIN) for i, x in enumerate(TERM_PIN_XS)]


def mount_tab() -> Part:
    """The +X tab."""
    outline = Sketch()
    for (x0, y0), (x1, y1) in zip(TAB_PATH, TAB_PATH[1:]):
        outline += Polygon((x0, y0), (x0 + TAB_T, y0), (x1 + TAB_T, y1), (x1, y1),
                           align=None)
    return zprism(outline, TAB_Z - TAB_W / 2, TAB_Z + TAB_W / 2)


def mount_tabs():
    return [leaf(mount_tab(), "mount_tab_pos_x", C_PIN),
            leaf(mount_tab().mirror(Plane.YZ), "mount_tab_neg_x", C_PIN)]


def terminal_block(show_pins):
    children = [terminal_body()]
    if show_pins:
        children += terminal_pins() + mount_tabs()
    return group("terminal_block", children)


# ---------------------------------------------------------------------------
# Reference PCB
# ---------------------------------------------------------------------------


def pcb(mode):
    shape = xbox(MOTOR_PLATE_X + 2 * PCB_MARGIN, BODY_Y_BOT, BODY_Y_BOT + PCB_T,
                 FRAME_Z_MIN - PCB_MARGIN, FRAME_Z_MAX + PCB_MARGIN)
    holes = ([(sx * MOTOR_PIN_X, MOTOR_PIN_Z, PCB_HOLE_D) for sx in (-1, 1)]
             + [(x, TERM_PIN_Z, PCB_HOLE_D) for x in TERM_PIN_XS]
             + [(sx * PCB_TAB_HOLE_X, TAB_Z, PCB_TAB_HOLE_D) for sx in (-1, 1)])
    for x, z, d in holes:
        shape -= ycyl(d, BODY_Y_BOT - 0.01, BODY_Y_BOT + PCB_T + 0.01, x=x, z=z)
    alpha = PCB_TRANSPARENT_ALPHA if mode == "transparent" else 1.0
    return leaf(shape, "reference_pcb", C_PCB, alpha)


# ---------------------------------------------------------------------------


def motor_fader(stem_pos=0.0, show_pins=True, show_pcb="transparent"):
    """stem_pos: position along the travel, 0 = center, +/-30 = ends."""
    children = [frame(), stem(stem_pos), motor(show_pins), terminal_block(show_pins)]
    if show_pcb != "off":
        children.append(pcb(show_pcb))
    return group("motor_fader", children)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--stem-pos", type=float, default=0.0,
                        help="stem position along the travel, -30..30 (default 0)")
    parser.add_argument("--no-pins", action="store_true", help="omit PCB pins and tabs")
    parser.add_argument("--pcb", choices=("full", "transparent", "off"),
                        default="transparent", help="reference PCB (default transparent; STEP keeps color only)")
    parser.add_argument("-o", "--output", default="motor_fader_60mm.step")
    args = parser.parse_args()

    model = motor_fader(args.stem_pos, not args.no_pins, args.pcb)
    export_step(model, args.output)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
