#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════
  Mirobot Pick-and-Place Planner — hybrid architecture
═══════════════════════════════════════════════════════════════════

Turns a small image into a 6×6 block mosaic on a grid by picking
coloured blocks off a conveyor and placing them one at a time.

SETUP
  Paste mirobot_controller.lua into /Mirobot's child script.

USAGE
  python3 mirobot_planner.py [pattern]
  pattern ∈ {smiley, heart, flag} or a path to a PNG.
"""

import math
import sys
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

CONTROLLER_FILE = 'mirobot_controller.lua'

ROBOT_Z     = 0.030
PICK_X      = 0.072
PICK_Y      = 0.120

PICK_Z      = 0.150

GRIPPER_DROP = 0.030

SURFACE_CLEARANCE = 0.005

GRID_CX     = 0.160
GRID_CY     = 0.000
CELL        = 0.010
BLOCK       = 0.010
PLACE_Z     = 0.150

APPROACH    = 0.030

HOME_XYZ    = (0.14, 0.00, 0.20)

SHELL_R     = 0.145

BELT_TOP_Z  = PICK_Z  - BLOCK / 2
MAT_TOP_Z   = PLACE_Z - BLOCK / 2

PALETTE = {
    'BLACK':  (0.10, 0.10, 0.10),
    'WHITE':  (0.95, 0.95, 0.95),
    'RED':    (0.85, 0.12, 0.12),
    'GREEN':  (0.12, 0.70, 0.20),
    'BLUE':   (0.12, 0.30, 0.85),
    'YELLOW': (0.95, 0.80, 0.10),
    'ORANGE': (0.95, 0.50, 0.10),
    'PURPLE': (0.55, 0.15, 0.75),
}

PATTERNS = {
    'smiley': [
        '.YYYY.',
        'YBYYBY',
        'YYYYYY',
        'YRYYRY',
        'YYRRYY',
        '.YYYY.',
    ],
    'heart': [
        '.RR.RR',
        'RRRRRR',
        'RRRRRR',
        '.RRRR.',
        '..RR..',
        '...R..',
    ],
    'flag': [
        'OOOOOO',
        'OOOOOO',
        'WWWWWW',
        'WWWWWW',
        'GGGGGG',
        'GGGGGG',
    ],
}
CHAR_TO_COLOUR = {
    'Y': 'YELLOW', 'B': 'BLACK', 'R': 'RED', 'G': 'GREEN',
    'W': 'WHITE',  'O': 'ORANGE', 'U': 'BLUE', 'P': 'PURPLE',
}

def force_mirobot_to_origin(sim):
    mirobot = sim.getObject('/Mirobot')
    cur_pos = sim.getObjectPosition(mirobot, -1)
    cur_ori = sim.getObjectOrientation(mirobot, -1)
    print(f'  Mirobot was at pos={[round(x, 4) for x in cur_pos]}, '
          f'ori={[round(x, 4) for x in cur_ori]}')
    sim.setObjectPosition(mirobot, [0.0, 0.0, ROBOT_Z], -1)
    sim.setObjectOrientation(mirobot, [0.0, 0.0, 0.0], -1)
    print(f'  Forced Mirobot to pos=(0, 0, {ROBOT_Z}), identity orientation.')

def clear_old_scene_items(sim):
    scene_objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    owned_prefixes = ('Block_', 'Belt', 'BeltRail_', 'WorkMat',
                      'GridLine_', 'gripper2f_', 'RailL', 'RailR')
    for h in scene_objs:
        try:
            alias = sim.getObjectAlias(h) or ''
            if any(alias.startswith(p) for p in owned_prefixes):
                try:
                    sim.removeObject(h)
                except Exception:
                    pass
        except Exception:
            pass

def hide_mirobot_demo_clutter(sim, tip_h=None):
    always_hide = ('pickupPart', 'pickPos')
    maybe_hide  = ('vacuumGripper',)

    tip_ancestor_handles = set()
    if tip_h is not None and tip_h != -1:
        h = tip_h
        while h != -1:
            tip_ancestor_handles.add(h)
            try:
                h = sim.getObjectParent(h)
            except Exception:
                break

    def hide_one(h):
        try:
            sim.setObjectInt32Param(
                h, sim.objintparam_visibility_layer, 0)
        except Exception:
            pass
        try:
            sim.setObjectInt32Param(
                h, sim.shapeintparam_respondable, 0)
        except Exception:
            pass
        try:
            sim.setObjectPosition(h, [100.0, 100.0, 100.0], -1)
        except Exception:
            pass

    scene_objs = sim.getObjectsInTree(
        sim.handle_scene, sim.handle_all, 0)
    for h in scene_objs:
        try:
            alias = sim.getObjectAlias(h) or ''
            if alias in always_hide:
                hide_one(h)
            elif alias in maybe_hide:
                if h in tip_ancestor_handles:

                    print(f'  Keeping {alias!r} (tip is anchored '
                          f'through it).')
                else:
                    hide_one(h)
        except Exception:
            pass

def ensure_mirobot_anatomy(sim):
    try:
        mirobot = sim.getObject('/Mirobot')
    except Exception:
        raise RuntimeError(
            'Cannot find /Mirobot in scene.  Add it from the Model\n'
            'browser:  robots → non-mobile → Mirobot.')

    children = sim.getObjectsInTree(mirobot, sim.handle_all, 0)

    def find_by_alias(target_alias):
        for h in children:
            try:
                if sim.getObjectAlias(h) == target_alias:
                    return h
            except Exception:
                pass
        return None

    tip    = find_by_alias('tip')
    target = find_by_alias('target')

    end_effector = None
    end_effector_name = None
    for candidate in ('connection', 'tcp', 'tool_tip', 'end_effector', 'flange'):
        h = find_by_alias(candidate)
        if h is not None:
            end_effector = h
            end_effector_name = candidate
            break

    anchor_parent = None
    anchor_world_pose = None
    if end_effector is not None:
        try:
            anchor_parent = sim.getObjectParent(end_effector)
            anchor_world_pose = sim.getObjectPose(end_effector, -1)
        except Exception:
            anchor_parent = None

    if anchor_parent is not None and tip is not None:
        tip_world = sim.getObjectPosition(tip, -1)
        base_world = sim.getObjectPosition(mirobot, -1)
        tip_dist_from_base = math.sqrt(
            (tip_world[0] - base_world[0])**2 +
            (tip_world[1] - base_world[1])**2 +
            (tip_world[2] - base_world[2])**2)

        if tip_dist_from_base > 1.0:
            anchor_parent_alias = sim.getObjectAlias(anchor_parent) or '?'
            try:
                tip_parent_alias = (
                    sim.getObjectAlias(sim.getObjectParent(tip))
                    if sim.getObjectParent(tip) != -1 else 'scene')
            except Exception:
                tip_parent_alias = '?'
            print(f'  /Mirobot tip is {tip_dist_from_base:.1f}m from base '
                  f'(parented to {tip_parent_alias!r}); re-anchoring to '
                  f'{anchor_parent_alias!r} at {end_effector_name!r} pose.')
            sim.setObjectParent(tip, anchor_parent, False)
            sim.setObjectPose(tip, anchor_world_pose, -1)
        else:
            print(f'  /Mirobot tip is at sensible world location '
                  f'({tip_world[0]:+.3f}, {tip_world[1]:+.3f}, '
                  f'{tip_world[2]:+.3f}); leaving as-is.')

    if tip is not None and target is not None:
        print(f'  /Mirobot anatomy: tip and target ready.')
        return tip, target

    print('  /Mirobot hierarchy:')
    for h in children:
        try:
            alias = sim.getObjectAlias(h) or '<no alias>'
            print(f'    handle={h:<4}  alias={alias!r}')
        except Exception:
            pass

    if tip is None:
        if anchor_parent is not None:

            tip = sim.createDummy(0.005)
            sim.setObjectAlias(tip, 'tip')
            sim.setObjectParent(tip, anchor_parent, False)
            sim.setObjectPose(tip, anchor_world_pose, -1)
            anchor_parent_alias = sim.getObjectAlias(anchor_parent) or '?'
            print(f'  Created tip dummy as child of {anchor_parent_alias!r}, '
                  f'positioned at {end_effector_name!r} (physical end of arm).')
        else:

            last_link = None
            last_num = -1
            for h in children:
                try:
                    alias = sim.getObjectAlias(h) or ''
                    if alias.startswith('link') and alias[4:].isdigit():
                        n = int(alias[4:])
                        if n > last_num:
                            last_num = n
                            last_link = h
                except Exception:
                    pass

            if last_link is None:
                raise RuntimeError(
                    'Cannot find a link, connection, or TCP to attach\n'
                    'the tip dummy to.  The Mirobot variant in your\n'
                    'scene is unusual — try a fresh one from the Model\n'
                    'browser (robots → non-mobile → Mirobot).')

            parent_alias = sim.getObjectAlias(last_link) or '?'
            tip = sim.createDummy(0.005)
            sim.setObjectAlias(tip, 'tip')
            sim.setObjectParent(tip, last_link, False)
            sim.setObjectPosition(tip, [0.0, 0.0, 0.0], last_link)
            sim.setObjectOrientation(tip, [0.0, 0.0, 0.0], last_link)
            print(f'  Created tip dummy as child of {parent_alias!r} '
                  f'(fallback — may be offset from actual end of arm).')

    if target is None:
        target = sim.createDummy(0.005)
        sim.setObjectAlias(target, 'target')
        sim.setObjectParent(target, mirobot, False)
        tip_pose = sim.getObjectPose(tip, -1)
        sim.setObjectPose(target, tip_pose, -1)
        print(f'  Created target dummy at tip pose.')

    return tip, target

def build_table(sim, alias, cx, cy, sx, sy, top_z,
                top_colour, leg_colour=(0.30, 0.22, 0.15)):
    top_thickness = 0.005
    leg_w = 0.012
    top_center_z = top_z - top_thickness / 2
    leg_top_z = top_z - top_thickness
    leg_height = max(leg_top_z - 0.0, 0.001)
    leg_center_z = leg_height / 2

    top = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [sx, sy, top_thickness], 0)
    sim.setObjectAlias(top, alias)
    sim.setObjectPosition(top, [cx, cy, top_center_z], -1)
    sim.setObjectInt32Param(top, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(top, sim.shapeintparam_respondable, 0)
    sim.setObjectColor(top, 0, sim.colorcomponent_ambient_diffuse,
                       list(top_colour))

    inset = leg_w / 2 + 0.002
    corners = [
        (+sx/2 - inset, +sy/2 - inset),
        (+sx/2 - inset, -sy/2 + inset),
        (-sx/2 + inset, +sy/2 - inset),
        (-sx/2 + inset, -sy/2 + inset),
    ]
    for i, (dx, dy) in enumerate(corners):
        leg = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [leg_w, leg_w, leg_height], 0)
        sim.setObjectAlias(leg, f'{alias}_leg{i}')
        sim.setObjectPosition(leg,
            [cx + dx, cy + dy, leg_center_z], -1)
        sim.setObjectInt32Param(leg, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(leg, sim.shapeintparam_respondable, 0)
        sim.setObjectColor(leg, 0, sim.colorcomponent_ambient_diffuse,
                           list(leg_colour))

def build_conveyor(sim):

    build_table(sim, 'Belt', PICK_X, PICK_Y,
                0.080, 0.050, BELT_TOP_Z,
                top_colour=(0.15, 0.15, 0.17))

    rail_h = 0.008
    rail_t = 0.004
    rail_z = BELT_TOP_Z + rail_h / 2
    for side, y_off in (('L', -0.025 - rail_t / 2),
                        ('R', +0.025 + rail_t / 2)):
        rail = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [0.080, rail_t, rail_h], 0)
        sim.setObjectAlias(rail, f'BeltRail_{side}')
        sim.setObjectPosition(rail,
            [PICK_X, PICK_Y + y_off, rail_z], -1)
        sim.setObjectInt32Param(rail, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(rail, sim.shapeintparam_respondable, 0)
        sim.setObjectColor(rail, 0, sim.colorcomponent_ambient_diffuse,
                           [0.55, 0.55, 0.60])

def build_workmat_and_grid(sim):

    build_table(sim, 'WorkMat', GRID_CX, GRID_CY,
                0.080, 0.080, MAT_TOP_Z,
                top_colour=(0.93, 0.91, 0.86))

    grid_z = MAT_TOP_Z + 0.001
    line_t = 0.0008
    line_w = 6 * CELL + 0.004
    for i in range(7):
        off = (i - 3) * CELL
        h = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [line_w, line_t, 0.001], 0)
        sim.setObjectAlias(h, f'GridLine_h{i}')
        sim.setObjectPosition(h, [GRID_CX, GRID_CY + off, grid_z], -1)
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
        sim.setObjectColor(h, 0, sim.colorcomponent_ambient_diffuse,
                           [0.2, 0.2, 0.2])
        v = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [line_t, line_w, 0.001], 0)
        sim.setObjectAlias(v, f'GridLine_v{i}')
        sim.setObjectPosition(v, [GRID_CX + off, GRID_CY, grid_z], -1)
        sim.setObjectInt32Param(v, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(v, sim.shapeintparam_respondable, 0)
        sim.setObjectColor(v, 0, sim.colorcomponent_ambient_diffuse,
                           [0.2, 0.2, 0.2])

def build_two_finger_gripper(sim, tip):

    try:
        vac = sim.getObject('/Mirobot/vacuumGripper')
        sim.setObjectInt32Param(vac, sim.objintparam_visibility_layer, 0)
        sim.setObjectInt32Param(vac, sim.shapeintparam_respondable, 0)
        sim.setObjectPosition(vac, [100.0, 100.0, 100.0], -1)
    except Exception:
        pass

    tip_world = sim.getObjectPosition(tip, -1)
    print(f'  Building 2-finger gripper at tip world ({tip_world[0]:+.3f}, '
          f'{tip_world[1]:+.3f}, {tip_world[2]:+.3f})')

    if abs(tip_world[0]) > 1.0 or abs(tip_world[1]) > 1.0 or tip_world[2] > 1.0:
        raise RuntimeError(
            f'Tip is at {tip_world} — clearly not at the end of the arm.\n'
            'ensure_mirobot_anatomy failed to re-anchor the tip correctly.\n'
            'Check the /Mirobot hierarchy: remove any stale tip dummies,\n'
            'then rerun.')

    base_thickness = 0.014
    base = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        [0.030, 0.020, base_thickness], 0)
    sim.setObjectAlias(base, 'gripper2f_base')
    sim.setObjectParent(base, tip, True)
    sim.setObjectPosition(
        base, [0.0, 0.0, +GRIPPER_DROP / 2], tip)
    sim.setObjectOrientation(base, [0.0, 0.0, 0.0], tip)
    sim.setObjectInt32Param(base, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(base, sim.shapeintparam_respondable, 0)
    sim.setObjectColor(base, 0, sim.colorcomponent_ambient_diffuse,
                       [1.0, 0.0, 1.0])

    for name, sign in (('leftFinger', -1), ('rightFinger', +1)):
        f = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [0.003, 0.014, 0.012], 0)
        sim.setObjectAlias(f, f'gripper2f_{name}')
        sim.setObjectParent(f, tip, True)
        sim.setObjectPosition(
            f, [sign * 0.010, 0.0, +GRIPPER_DROP], tip)
        sim.setObjectOrientation(f, [0.0, 0.0, 0.0], tip)
        sim.setObjectInt32Param(f, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(f, sim.shapeintparam_respondable, 0)
        sim.setObjectColor(f, 0, sim.colorcomponent_ambient_diffuse,
                           [0.0, 1.0, 0.0])

    for name in ('gripper2f_base', 'gripper2f_leftFinger',
                 'gripper2f_rightFinger'):
        for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
            try:
                if sim.getObjectAlias(h) == name:
                    pos = sim.getObjectPosition(h, -1)
                    print(f'    {name}: world=({pos[0]:+.3f}, '
                          f'{pos[1]:+.3f}, {pos[2]:+.3f})')
                    break
            except Exception:
                pass

_block_counter = 0

def spawn_block(sim, colour_name, pos):
    global _block_counter
    _block_counter += 1
    b = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [BLOCK, BLOCK, BLOCK], 0)
    sim.setObjectAlias(b, f'Block_{colour_name}_{_block_counter}')
    sim.setObjectPosition(b, list(pos), -1)
    sim.setObjectInt32Param(b, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(b, sim.shapeintparam_respondable, 0)
    sim.setObjectColor(b, 0, sim.colorcomponent_ambient_diffuse,
                       list(PALETTE[colour_name]))
    return b

def _snap_block_to_cell(sim, block_handle, cell_world):
    px, py, pz = cell_world
    try:
        sim.setObjectPosition(block_handle, [px, py, pz], -1)
        sim.setObjectOrientation(block_handle, [0.0, 0.0, 0.0], -1)
    except Exception:
        pass

def parse_pattern(name_or_path):
    if name_or_path in PATTERNS:
        rows = PATTERNS[name_or_path]
        cells = []
        for r, line in enumerate(rows):
            for c, ch in enumerate(line):
                if ch in CHAR_TO_COLOUR:
                    cells.append((r, c, CHAR_TO_COLOUR[ch]))
        cells.sort(key=lambda x: (x[2], x[0], x[1]))
        return cells

    try:
        from PIL import Image
    except ImportError:
        print(f'Pattern "{name_or_path}" unknown and PIL not installed.')
        print(f'Choose from: {list(PATTERNS)}')
        sys.exit(1)

    img = Image.open(name_or_path).convert('RGB').resize((6, 6), Image.LANCZOS)
    cells = []
    for i, (r, g, b) in enumerate(list(img.getdata())):
        row, col = divmod(i, 6)
        if r > 230 and g > 230 and b > 230:
            continue
        rgb_n = (r / 255.0, g / 255.0, b / 255.0)
        best = min(PALETTE.items(),
                   key=lambda kv: sum((a-b)**2 for a, b in zip(rgb_n, kv[1])))
        cells.append((row, col, best[0]))
    cells.sort(key=lambda x: (x[2], x[0], x[1]))
    return cells

def cell_to_world(row, col):
    x = GRID_CX + (col - 2.5) * CELL
    y = GRID_CY + (row - 2.5) * CELL
    return (x, y, PLACE_Z)

def shell_midpoint(x0, y0, x1, y1, shell_r=SHELL_R):
    theta0 = math.atan2(y0, x0)
    theta1 = math.atan2(y1, x1)
    dtheta = theta1 - theta0

    if dtheta > math.pi:
        dtheta -= 2 * math.pi
    elif dtheta < -math.pi:
        dtheta += 2 * math.pi
    theta_mid = theta0 + dtheta / 2
    return (shell_r * math.cos(theta_mid),
            shell_r * math.sin(theta_mid))

def print_install_instructions():
    print()
    print('━' * 68)
    print('  The Mirobot controller (Lua) is not installed or did not')
    print('  start correctly.')
    print()
    print(f'  1. Open  {CONTROLLER_FILE}  in a text editor.')
    print('  2. Right-click /Mirobot in CoppeliaSim\'s scene hierarchy.')
    print('  3. Edit (or Add → Associated child script → Non-threaded).')
    print('  4. Select all existing code, delete, paste this file.')
    print('  5. Save (Ctrl-S) and close the editor.')
    print('  6. Re-run:  python3 mirobot_planner.py smiley')
    print('━' * 68)
    print()

def find_mirobot_script(sim):
    try:
        mirobot = sim.getObject('/Mirobot')
    except Exception as e:
        print(f'[!] Cannot find /Mirobot in the scene: {e}')
        return None
    try:
        script_h = sim.getScript(sim.scripttype_childscript, mirobot)
        if script_h is None or script_h == -1:
            return None
        return script_h
    except Exception:
        return None

def verify_controller(sim, script_h, timeout_s=5.0):
    deadline = time.time() + timeout_s
    last_err = None
    while time.time() < deadline:
        try:
            result = sim.callScriptFunction('ping', script_h)
            if result == 'ready':
                print(f'Controller ping: ready ✓')
                return True
            if isinstance(result, str) and result.startswith('init_failed'):
                print(f'Controller ping: {result}')
                return False
            last_err = f'unexpected response: {result!r}'
        except Exception as e:
            last_err = str(e)
        time.sleep(0.25)
    print(f'[!] ping() did not respond in {timeout_s}s.  Last: {last_err}')
    return False

_SCRIPT_H = None

def rpc(sim, func, *args):
    return sim.callScriptFunction(func, _SCRIPT_H, *args)

def wait_to_reach(sim, target_xyz, tol=0.015, timeout=3.5):
    deadline = time.time() + timeout
    last_d = float('inf')
    while time.time() < deadline:
        try:
            p = rpc(sim, 'get_tip_pos')
            if isinstance(p, (list, tuple)) and len(p) == 3:
                dx = p[0] - target_xyz[0]
                dy = p[1] - target_xyz[1]
                dz = p[2] - target_xyz[2]
                last_d = math.sqrt(dx*dx + dy*dy + dz*dz)
                if last_d < tol:
                    return True, last_d
        except Exception:
            pass
        time.sleep(0.05)
    return False, last_d

def move_and_wait(sim, x, y, z, tol=0.018, verbose=True):
    rpc(sim, 'goto_pose', x, y, z)
    reached, d = wait_to_reach(sim, (x, y, z), tol=tol)
    if not reached and verbose:
        try:
            p = rpc(sim, 'get_tip_pos')
            print(f'    cmd=({x:+.3f}, {y:+.3f}, {z:+.3f})  '
                  f'tip=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})  '
                  f'err={d:.3f}')
        except Exception:
            pass
    return reached, d

def park_above_pick(sim):
    park_z = PICK_Z + GRIPPER_DROP + SURFACE_CLEARANCE + APPROACH + 0.010
    print(f'\nParking above pick at ({PICK_X:.3f}, {PICK_Y:.3f}, {park_z:.3f})...')
    rpc(sim, 'goto_pose', PICK_X, PICK_Y, park_z)
    reached, d = wait_to_reach(sim, (PICK_X, PICK_Y, park_z),
                                tol=0.015, timeout=4.0)
    print(f'  parked (d={d:.3f})')

def main():
    global _SCRIPT_H

    pattern = sys.argv[1] if len(sys.argv) > 1 else 'smiley'

    print('Connecting to CoppeliaSim...')
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print('Connected.')

    cells = parse_pattern(pattern)
    colours = {}
    for _, _, c in cells:
        colours[c] = colours.get(c, 0) + 1
    print(f'Pattern "{pattern}": {len(cells)} blocks  {colours}')

    try:
        sim.stopSimulation()
    except Exception:
        pass
    time.sleep(0.6)
    for _ in range(20):
        try:
            if sim.getSimulationState() == sim.simulation_stopped:
                break
        except Exception:
            break
        time.sleep(0.1)

    print('Looking for Mirobot controller script...')
    script_h = find_mirobot_script(sim)
    if script_h is None:
        print('[!] /Mirobot has no child script.')
        print_install_instructions()
        sys.exit(1)
    print(f'  Found child script on /Mirobot (handle {script_h})')
    _SCRIPT_H = script_h

    print('Building scene...')
    force_mirobot_to_origin(sim)
    tip, target = ensure_mirobot_anatomy(sim)
    clear_old_scene_items(sim)
    hide_mirobot_demo_clutter(sim, tip_h=tip)
    build_conveyor(sim)
    build_workmat_and_grid(sim)
    build_two_finger_gripper(sim, tip)
    print('Scene ready.')

    print('\nScene inventory:')
    print('  handle  alias                      world-pos')
    all_objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    seen = set()
    for h in all_objs:
        if h in seen:
            continue
        seen.add(h)
        try:
            alias = sim.getObjectAlias(h) or f'<h={h}>'
            pos = sim.getObjectPosition(h, -1)
            parent_h = sim.getObjectParent(h)
            par = 'root' if parent_h == -1 else f'<h={parent_h}>'
            print(f'  {h:>4}   {alias:<26} '
                  f'({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})   parent={par}')
        except Exception:
            pass
    print()

    sim.startSimulation()
    time.sleep(1.0)

    if not verify_controller(sim, script_h, timeout_s=6.0):
        print_install_instructions()
        try:
            sim.stopSimulation()
        except Exception:
            pass
        sys.exit(1)

    print('Homing arm...')
    rpc(sim, 'home')
    reached, d_home = wait_to_reach(sim, HOME_XYZ, tol=0.020, timeout=4.0)
    print(f'  d_home={d_home:.3f}')

    park_above_pick(sim)

    for i, (row, col, colour) in enumerate(cells, 1):
        bar_len = int(28 * i / len(cells))
        bar = '█' * bar_len + '░' * (28 - bar_len)
        print(f'\n[{bar}] {i}/{len(cells)} {colour}')

        block_h = spawn_block(sim, colour, (PICK_X, PICK_Y, PICK_Z))

        time.sleep(0.25)
        px, py, pz = cell_to_world(row, col)

        pick_tip_z    = PICK_Z + GRIPPER_DROP + SURFACE_CLEARANCE
        place_tip_z   = pz + GRIPPER_DROP + SURFACE_CLEARANCE
        approach_pick = pick_tip_z + APPROACH
        approach_plc  = place_tip_z + APPROACH

        move_and_wait(sim,
            PICK_X, PICK_Y, approach_pick, tol=0.018, verbose=False)
        _, d_p = move_and_wait(sim,
            PICK_X, PICK_Y, pick_tip_z, tol=0.018)
        print(f'  PICK  d_pick={d_p:.3f}')

        gripped = rpc(sim, 'grip')
        if not gripped:
            print('  [!] grip() returned 0 — skipping; check console.')
            move_and_wait(sim,
                PICK_X, PICK_Y, approach_pick, tol=0.020, verbose=False)
            continue

        move_and_wait(sim,
            PICK_X, PICK_Y, approach_pick, tol=0.020, verbose=False)
        mx, my = shell_midpoint(PICK_X, PICK_Y, px, py)

        transit_z = max(approach_pick, approach_plc)
        move_and_wait(sim,
            mx, my, transit_z, tol=0.025, verbose=False)
        move_and_wait(sim,
            px, py, approach_plc, tol=0.020, verbose=False)
        _, d_pl = move_and_wait(sim, px, py, place_tip_z, tol=0.020)
        print(f'  PLACE ({px:.3f}, {py:.3f}, {pz:.3f})  d={d_pl:.3f}')
        rpc(sim, 'release')

        _snap_block_to_cell(sim, block_h, (px, py, pz))

        move_and_wait(sim,
            px, py, approach_plc, tol=0.020, verbose=False)
        mx2, my2 = shell_midpoint(px, py, PICK_X, PICK_Y)
        move_and_wait(sim,
            mx2, my2, transit_z, tol=0.025, verbose=False)

    rpc(sim, 'home')
    wait_to_reach(sim, HOME_XYZ, tol=0.020, timeout=4.0)

    print('\nFinal block positions:')
    scene_objs = sim.getObjectsInTree(
        sim.handle_scene, sim.object_shape_type, 0)
    block_handles = sorted(
        [h for h in scene_objs
         if (sim.getObjectAlias(h) or '').startswith('Block_')],
        key=lambda h: int((sim.getObjectAlias(h) or '_0').split('_')[-1])
    )
    for h in block_handles:
        alias = sim.getObjectAlias(h) or ''
        pos = sim.getObjectPosition(h, -1)
        parent = sim.getObjectParent(h)
        parent_tag = 'scene'
        if parent != -1:
            try:
                parent_tag = sim.getObjectAlias(parent) or f'[{parent}]'
            except Exception:
                parent_tag = f'[{parent}]'
        print(f'  {alias:24s} ({pos[0]:+.3f}, {pos[1]:+.3f}, '
              f'{pos[2]:+.3f})  parent={parent_tag}')

    print(f'\n✓ Done. Processed {len(cells)} blocks.')

if __name__ == '__main__':
    main()
