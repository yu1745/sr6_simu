"""Replace four secondary links in sr6_mini.stp; mm, source file is preserved."""
import hashlib
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
from cad_replacements import fit_motor_base, fit_pitch_link, add_opposite_pitch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'cad'
OUT.mkdir(exist_ok=True)
SOURCE = ROOT.parent / 'sr6_mini.stp'
TARGET = OUT / 'sr6_mini_complete.stp'
SHAFT_LENGTH = 210.0
ENGAGEMENT = 12.0
EYE_HEIGHT = 36.0
LINK_LENGTH = SHAFT_LENGTH + 2 * (EYE_HEIGHT - ENGAGEMENT)
NUT_HEIGHT = 5.0
BALL_WIDTH = 9.0


def vec(v):
    return np.asarray(v, dtype=float)


def transform(shape, rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    trsf = gp_Trsf()
    trsf.SetValues(*[float(n) for row in m[:3] for n in row])
    return shape.transformShape(cq.Matrix(trsf))


def frame(direction):
    z = vec(direction) / np.linalg.norm(direction)
    x = vec([0, 0, 1]) - z * z[2]
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def cylinder(radius, length, origin=(0, 0, 0), direction=(0, 0, 1)):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(*direction))


def rz(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


print('Import source STEP', flush=True)
solids = cq.importers.importStep(str(SOURCE)).val().Solids()
assert len(solids) == 54
parts = list(solids)
names = {i: f'source_{i:02d}' for i in range(len(parts))}
print('Fit supplied U_body_8108 motor base', flush=True)
base_fit = fit_motor_base(parts, transform)
names[1] = 'U_body_8108_replacement'

# Supplier SA6T/K: preserve exact independent housing/ball solids.
bearing_source = OUT/'inputs/SA6TK-jlcfa.step'
supplier_eye, ball = cq.importers.importStep(str(bearing_source)).val().Solids()
# Supplier stem points -Y and bore +Z; canonical stem +Z and bore +X.
eye = transform(supplier_eye, np.array([[0,0,1],[-1,0,0],[0,-1,0]]), vec([0,0,0]))
assert eye.isValid() and ball.isValid()
# Stock steel sleeves: ID6 OD8, lengths5 and10; nominal dimensions.
washer = cylinder(4,5).cut(cylinder(3,5)).clean()
spacer = cylinder(4,10).cut(cylinder(3,10)).clean()
assert washer.isValid() and spacer.isValid()

# User requested no helical teeth. The simplified thread envelope is diameter6.05
# for mating clearance, NOT the manufacturing tap-drill diameter (nominal5).
print('Build D10 L210 shaft with simplified M6 depth12 end holes', flush=True)
shaft = cylinder(5, SHAFT_LENGTH)
reverse = np.diag([1, -1, -1])
for rot, origin in [(np.eye(3), vec([0, 0, 0])), (reverse, vec([0, 0, SHAFT_LENGTH]))]:
    bore = cylinder(3.025, ENGAGEMENT).fuse(cylinder(2.5, 14))
    shaft = shaft.cut(transform(bore, rot, origin))
shaft = shaft.clean()
assert shaft.isValid() and len(shaft.Solids()) == 1

# DIN934 M6: width across flats10, height5. Thread represented by its bore here.
nut = cq.Workplane('XY').polygon(6, 10 / math.cos(math.pi / 6)).extrude(NUT_HEIGHT)
nut = nut.faces('>Z or <Z').edges().chamfer(0.3).val().cut(cylinder(3, NUT_HEIGHT)).clean()

RECEIVER_X, RECEIVER_Y = 100.062295, 270.186359
RIGHT_FACE, LEFT_FACE = 29.5, -154.671573
# Lower/upper refer to firmware linkage names; order at the receiver is lower
# nearest the receiver, upper outside it. Every pair shares a single bolt axis.
links = [
    dict(motor=0, crank=15, shaft=20, lower_ball=21, upper_ball=18, lower_eye=22, upper_eye=19,
         pivot=[50, 58.999984, -147], upper=[RECEIVER_X, RECEIVER_Y, LEFT_FACE - 9.5], initial=math.pi),
    dict(motor=1, crank=16, shaft=25, lower_ball=26, upper_ball=23, lower_eye=27, upper_eye=24,
         pivot=[145, 58.999984, -147], upper=[RECEIVER_X, RECEIVER_Y, LEFT_FACE - 28.5], initial=math.atan2(55.285477-58.999984, 249.934277-145)),
    dict(motor=4, crank=14, shaft=30, lower_ball=31, upper_ball=28, lower_eye=32, upper_eye=29,
         pivot=[145, 58.999984, 39], upper=[RECEIVER_X, RECEIVER_Y, RIGHT_FACE + 28.5], initial=0),
    dict(motor=5, crank=0, shaft=35, lower_ball=36, upper_ball=33, lower_eye=37, upper_eye=34,
         pivot=[50, 58.999984, 39], upper=[RECEIVER_X, RECEIVER_Y, RIGHT_FACE + 9.5], initial=math.pi),
]
report_links = []
for link in links:
    shift = base_fit['shifts']['main_left' if link['motor'] < 2 else 'main_right']
    link['pivot'] = (vec(link['pivot']) + vec(shift)).tolist()
    pivot, upper = vec(link['pivot']), vec(link['upper'])
    delta = upper - pivot
    cosine = (np.dot(delta, delta) + 105**2 - LINK_LENGTH**2) / (210 * np.linalg.norm(delta[:2]))
    assert abs(cosine) <= 1
    heading, offset = math.atan2(delta[1], delta[0]), math.acos(cosine)
    angle = min([heading+offset, heading-offset], key=lambda a: abs(math.atan2(math.sin(a-link['initial']), math.cos(a-link['initial']))))
    lower = pivot + vec([105 * math.cos(angle), 105 * math.sin(angle), 0])
    direction = (upper - lower) / LINK_LENGTH
    rotation = frame(direction)
    parts[link['shaft']] = transform(shaft, rotation, lower + direction * (EYE_HEIGHT-ENGAGEMENT))
    parts[link['lower_eye']] = transform(eye, rotation, lower)
    parts[link['upper_eye']] = transform(eye, frame(-direction), upper)
    # Inner balls retain the mounting pin's Z axis while the housing swivels.
    for key, center in [('lower_ball', lower), ('upper_ball', upper)]:
        parts[link[key]] = ball.translate(tuple(center))
    crank_rotation = rz(angle - link['initial'])
    parts[link['crank']] = transform(parts[link['crank']], crank_rotation, pivot - crank_rotation @ pivot)
    for key in ['crank', 'shaft', 'lower_eye', 'upper_eye', 'lower_ball', 'upper_ball']:
        names[link[key]] = f'M{link["motor"]+1}_{key}'
    report_links.append({**link, 'lower': lower.tolist(), 'length': LINK_LENGTH,
                         'crank_angle_deg': math.degrees(angle),
                         'shaft_start': (lower + direction * 24).tolist(),
                         'shaft_end': (upper - direction * 24).tolist()})

# Single relief washer and double-ended relief spacer on each receiver side.
for side, face in [(1, RIGHT_FACE), (-1, LEFT_FACE)]:
    for level, distance in enumerate([0, 5 + BALL_WIDTH]):
        origin = vec([RECEIVER_X, RECEIVER_Y, face + side * distance])
        rotation = np.eye(3) if side == 1 else reverse
        names[len(parts)] = f'receiver_{"right" if side == 1 else "left"}_spacer_{level+1}_M6'
        parts.append(transform(washer.rotate((0,0,0),(1,0,0),180).translate((0,0,5)) if level == 0 else spacer, rotation, origin))

# Source receiver and crank mounting holes were diameter5; clear them to6.5
# so the requested M6 bearing pin/bolt can pass through all mounting parts.
parts[38] = parts[38].cut(cylinder(3.25, 240, (RECEIVER_X, RECEIVER_Y, -190))).clean()
for link in report_links:
    x, y, _ = link['lower']
    parts[link['crank']] = parts[link['crank']].cut(cylinder(3.25, 240, (x, y, -190))).clean()

print('Fit supplied pitch secondary arm and its two rod ends', flush=True)
pitch_fit = fit_pitch_link(parts, eye, base_fit, transform, frame, cylinder, ball)
names[49] = 'pitch_secondary_arm_replacement'
for key in ['crank','lower_ball','upper_ball','lower_eye','upper_eye']:
    names[pitch_fit['link'][key]] = f'M4_{key}'
print('Add mirrored left pitch linkage on actual mounting pins', flush=True)
left_pitch = add_opposite_pitch(parts, names, pitch_fit, transform, cylinder)
replacements = {'motor_base':base_fit, 'pitch_arm':pitch_fit, 'left_pitch_arm':left_pitch}
(OUT/'replacement-fit.json').write_text(json.dumps(replacements,indent=2),encoding='utf-8')

# Two complete clamps for the stacked main rod ends, M6x50 screws.
# Head outwards for hex-key access; locking nut on the inside of the receiver.
cap_head = cq.Workplane('XY').circle(5).extrude(6).edges('%CIRCLE').chamfer(0.3).val().translate((0,0,-6))
socket = cq.Workplane('XY',origin=(0,0,-6.01)).polygon(6,5/math.cos(math.pi/6)).extrude(3.01).val()
cap_bolt = cap_head.cut(socket).fuse(cylinder(3,50)).clean()
assert cap_bolt.isValid() and len(cap_bolt.Solids()) == 1
fasteners = []
for side, face in [(1,RIGHT_FACE),(-1,LEFT_FACE)]:
    rotation = reverse if side == 1 else np.eye(3)
    bolt_id = len(parts)
    head_seat = [RECEIVER_X,RECEIVER_Y,face+side*38]
    names[bolt_id] = f'receiver_{"right" if side == 1 else "left"}_M6x50_socket_cap_bolt'
    parts.append(transform(cap_bolt,rotation,vec(head_seat)))
    nut_id = len(parts)
    nut_seat = [RECEIVER_X,RECEIVER_Y,face-side*4]
    names[nut_id] = f'receiver_{"right" if side == 1 else "left"}_M6_locking_nut'
    # Put a hex flat toward the nearby receiver bend to clear its inside radius.
    parts.append(transform(nut.rotate((0,0,0),(0,0,1),30),rotation,vec(nut_seat)))
    fasteners.append({'side':'right' if side == 1 else 'left','bolt_id':bolt_id,'nut_id':nut_id,
        'designation':'M6x50 full-thread socket head cap screw','head_diameter':10,'head_height':6,
        'hex_key':5,'hex_socket_depth':3,'head_seat':head_seat,'nut_seat':nut_seat,
        'clamped_stack':42,'locking_nut_height':5,'tip_protrusion':3,'thread_model':'simplified'})

# The pitch arm end plates are tapped directly; the existing rod-end shanks
# screw into them. Only the two remaining receiver pivots need through bolts.
pitch_bolt = cap_head.cut(socket).fuse(cylinder(3,30)).clean()
for side, face, link in [(1,RIGHT_FACE,pitch_fit['link']),(-1,LEFT_FACE,left_pitch['link'])]:
    rotation = reverse if side == 1 else np.eye(3)
    upper = vec(link['upper'])
    head_seat = (upper + vec([0,0,side*9.5])).tolist()
    nut_seat = [float(upper[0]),float(upper[1]),face-side*4]
    bolt_id = len(parts)
    names[bolt_id] = f'receiver_pitch_{"right" if side == 1 else "left"}_M6x30_socket_cap_bolt'
    parts.append(transform(pitch_bolt,rotation,vec(head_seat)))
    nut_id = len(parts)
    names[nut_id] = f'receiver_pitch_{"right" if side == 1 else "left"}_M6_locking_nut'
    parts.append(transform(nut,rotation,vec(nut_seat)))
    fasteners.append({'side':'right' if side == 1 else 'left','mount':'pitch',
        'bolt_id':bolt_id,'nut_id':nut_id,'connected_parts':[38,link['upper_ball'],link['upper_eye']],
        'designation':'M6x30 socket head cap screw','head_diameter':10,'head_height':6,
        'hex_key':5,'hex_socket_depth':3,'head_seat':head_seat,'nut_seat':nut_seat,
        'clamped_stack':23,'locking_nut_height':5,'tip_protrusion':2,'thread_model':'simplified'})

# Additional external washers, appended to preserve existing motion IDs.
washer_mounts = []
for side, face in [(1,RIGHT_FACE),(-1,LEFT_FACE)]:
    origin = vec([RECEIVER_X,RECEIVER_Y,face+side*33])
    washer_mounts.append((side,origin,'main_outer'))
for side, link in [(1,pitch_fit['link']),(-1,left_pitch['link'])]:
    center = vec(link['upper'])
    washer_mounts.extend([(side,center+vec([0,0,side*4.5]),'pitch_outer'),
                          (-side,center-vec([0,0,side*4.5]),'pitch_inner')])
for side,origin,label in washer_mounts:
    names[len(parts)] = f'receiver_{label}_{len(parts)}_steel_sleeve_M6'
    parts.append(transform(washer,np.eye(3) if side == 1 else reverse,origin))

print('Validate and export assembly', flush=True)
inherited_invalid = [i for i, s in enumerate(solids) if not s.isValid()]
modified_ids = [1,17,38,49,50,51,52,53, *range(54, len(parts))]
for link in links:
    modified_ids.extend(link[key] for key in ['crank','shaft','lower_ball','upper_ball','lower_eye','upper_eye'])
assert all(parts[i].isValid() and len(parts[i].Solids()) == 1 for i in modified_ids)
assert all(len(p.Solids()) == 1 for p in parts)
print('Source motor solids with pre-existing B-rep issues:', inherited_invalid, flush=True)
print('Check new mounting interfaces and simplified thread engagement', flush=True)
checks = []
# Canonical joint: lower eye at0; shaft end at24; shank tip at36 ->12 insertion.
thread_common = eye.intersect(shaft.translate((0,0,24)))
assert thread_common.isValid()
thread_overlap = sum(abs(s.Volume()) for s in thread_common.Solids())
assert thread_overlap < 1e-3, f'Thread interference: {thread_overlap}'
for link in report_links:
    length = float(np.linalg.norm(vec(link['upper']) - vec(link['lower'])))
    assert abs(length - LINK_LENGTH) < 1e-7
    checks.append({'motor':link['motor']+1, 'eye_to_eye':length})
interface_pairs = [(18,23),(19,24),(28,33),(29,34),(51,38),(53,17),(49,51),(49,53)]
lp = left_pitch['link']
assert abs(np.linalg.norm(vec(lp['upper'])-vec(lp['lower']))-lp['length']) < 1e-7
checks.append({'motor':3,'eye_to_eye':lp['length']})
interface_pairs.extend([(lp['upper_eye'],38),(lp['upper_ball'],38),
    (lp['lower_eye'],lp['crank']),(lp['shaft'],lp['upper_eye']),(lp['shaft'],lp['lower_eye'])])
for hardware in fasteners:
    connected = [38,28,29,33,34,54,55] if hardware['side'] == 'right' else [38,18,19,23,24,56,57]
    connected = hardware.get('connected_parts', connected)
    for part in connected:
        interface_pairs.append((hardware['bolt_id'],part))
        interface_pairs.append((hardware['nut_id'],part))
    interface_pairs.append((hardware['bolt_id'],hardware['nut_id']))
for link in links:
    nut_ids = [56,57] if link['motor'] < 2 else [54,55]
    interface_pairs.extend((link['upper_eye'], j) for j in [38,*nut_ids])
    interface_pairs.extend((link['upper_ball'], j) for j in [38,*nut_ids])
    interface_pairs.append((link['lower_eye'],link['crank']))
overlaps = []
# Every added washer is also checked against the fixed clamp stack at home.
for a in range(72,78):
    interface_pairs.extend((a,b) for b in [38,18,19,23,24,28,29,33,34,50,51,60,61,54,55,56,57,*range(64,a)] if a != b)
for a,b in interface_pairs:
    volume = abs(parts[a].intersect(parts[b]).Volume())
    if volume > 1e-3:
        overlaps.append({'parts':[a,b], 'volume_mm3':volume})
(OUT/'validation.json').write_text(json.dumps({'link_dimensions':checks,
    'simplified_thread_envelope_overlap_mm3':thread_overlap,'mounting_interface_overlaps':overlaps,
    'modified_solids_valid':True,'source_invalid_motor_solids':inherited_invalid},indent=2),encoding='utf-8')
assert not overlaps, f'Mounting interference: {overlaps}'
assembly = cq.Assembly(name='sr6_mini_complete')
for i, part in enumerate(parts):
    color = cq.Color(0.22,0.25,0.28) if 'socket_cap_bolt' in names[i] else cq.Color(0.75, 0.79, 0.83) if i in [20,25,30,35] or 'nut' in names[i] or 'spacer' in names[i] else cq.Color(0.5, 0.65, 0.72)
    assembly.add(part, name=names[i], color=color)
assembly.export(str(TARGET), exportType='STEP')
cq.exporters.export(shaft, str(OUT / 'shaft_D10_L210_M6_depth12.stp'), exportType='STEP')
cq.exporters.export(washer, str(OUT/'steel_sleeve_6x8x5.step'))
cq.exporters.export(spacer, str(OUT/'steel_sleeve_6x8x10.step'))
cq.exporters.export(eye, str(OUT / 'SA6TK_M6x1_housing.stp'), exportType='STEP')

manifest = {
    'source': SOURCE.name, 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'output': TARGET.name, 'sha256': hashlib.sha256(TARGET.read_bytes()).hexdigest(),
    'shaft': {'diameter':10, 'length':210, 'thread':'M6x1 RH', 'thread_depth':12, 'pilot_depth':14,
              'thread_representation':'simplified cylindrical envelope, no helical teeth',
              'modeled_hole_diameter':6.05, 'nominal_tap_drill_diameter':5},
    'bearing': {'model':'SA6T/K JLCFA supplier STEP', 'bore':6, 'head_diameter':20,
                'ball_diameter':12.7, 'ball_width':9, 'housing_width':6.75,
                'center_to_tip':36, 'male_thread_length':21, 'engagement':12, 'exposed_thread':9},
    'nuts': {'model':'DIN934 M6', 'height':5, 'across_flats':10, 'spacer_quantity':0,'locking_quantity':4,'quantity':4},
    'pitch_arm_connection':'Rod-end male shanks screw directly into tapped arm end plates; no additional bolts or nuts; threads simplified',
    'receiver_fasteners':fasteners,
    'stack':'receiver -> 5 mm steel sleeve -> 9 mm ball -> 10 mm steel sleeve -> 9 mm ball -> 5 mm steel sleeve',
    'receiver_ball_spacing':19, 'steel_sleeves':{'bore':6,'outer_diameter':8,'height':5,'single_quantity':8,'double_height':10,'double_quantity':2,'status':'nominal stock sleeve geometry; joint interference begins near 19 degrees; verify supplier bore tolerance'}, 'eye_to_eye':LINK_LENGTH,
    'inherited_invalid_source_solids': inherited_invalid,
    'links':sorted([*report_links,pitch_fit['link'],left_pitch['link']],key=lambda l:l['motor']),
    'part_names':names, 'replacements':replacements,
    'references': ['https://www.ikont.co.jp/global_data/download/pdf_catalog/cat5502dPHS.pdf',
      'https://eshop.wuerth.de/Product-categories/DIN-934-stainless-steel-A2-plain/14013512120105.cyid/1401.cgid/en/US/EUR/'],
}
(OUT / 'assembly-spec.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')

# Export from this same ordered part list: keeps IDs stable for motion bindings.
print('Tessellate preview assets', flush=True)
meshes = []
for i, part in enumerate(parts):
    vertices, triangles = part.tessellate(0.15, 0.2)
    b = part.BoundingBox()
    meshes.append({'id':i, 'name':names[i], 'center':part.Center().toTuple(),
        'bounds':{'min':[b.xmin,b.ymin,b.zmin], 'max':[b.xmax,b.ymax,b.zmax]},
        'positions':[round(n,5) for v in vertices for n in v.toTuple()],
        'indices':[n for t in triangles for n in t]})
(ROOT/'public'/'sr6-cad.json').write_text(json.dumps({
    'units':'mm', 'source':TARGET.name, 'sha256':manifest['sha256'], 'spec':manifest, 'parts':meshes,
}, separators=(',',':')), encoding='utf-8')
print(f'{TARGET}\n{len(parts)} solids; eye-to-eye {LINK_LENGTH} mm', flush=True)
