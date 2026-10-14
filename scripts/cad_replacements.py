"""Rigid placement of user-supplied replacement STEP parts, with fit reports."""
import math
from pathlib import Path
import cadquery as cq
import numpy as np

INPUTS = Path(__file__).resolve().parents[1] / 'cad' / 'inputs'


def circle_centers(shape, radius):
    found = {}
    for edge in shape.Edges():
        if edge.geomType() == 'CIRCLE' and abs(edge.radius() - radius) < 1e-5:
            point = edge.arcCenter().toTuple()
            found[tuple(round(v, 5) for v in point)] = np.array(point)
    return list(found.values())


def fit_motor_base(parts, transform):
    base = cq.importers.importStep(str(INPUTS/'motor-base-replacement-0.stp')).val().Solids()[0]
    centers = sorted(circle_centers(base, 33.5), key=lambda p:p[0])
    assert len(centers) == 12
    rows = [np.array(centers[i:i+4]) for i in range(0,12,4)]
    longitudinal = rows[1].mean(axis=0) - rows[0].mean(axis=0)
    longitudinal /= np.linalg.norm(longitudinal)
    vertical = np.array([0.,0.,1.])
    lateral = np.cross(vertical, longitudinal)
    lateral /= np.linalg.norm(lateral)
    longitudinal = np.cross(lateral, vertical)
    rotation = np.array([longitudinal, -vertical, lateral])
    origin = rotation @ rows[0].mean(axis=0)
    # Align the first motor row and lateral center, and seat the bottom plate at Y=0.
    translation = np.array([50-origin[0], base.BoundingBox().zmax, -54-origin[2]])
    placed = [np.array([rotation@p+translation for p in row]) for row in rows]
    height = float(placed[0][:,1].mean())
    shift_y = height - 58.999984
    main_right = float(placed[0][:,2].max())
    main_left = float(placed[0][:,2].min())
    pitch_right = float(placed[2][:,2].max())
    pitch_left = float(placed[2][:,2].min())
    shifts = {
        'main_right': [0,shift_y,main_right],
        'main_left': [0,shift_y,main_left+108],
        'pitch_right': [0,shift_y,pitch_right],
        'pitch_left': [0,shift_y,pitch_left+108],
    }
    for group, ids in [('main_right',[0,2,3,4,5,14]),('main_left',[8,9,10,11,15,16]),
                       ('pitch_right',[6,7,17]),('pitch_left',[12,13])]:
        for i in ids:
            parts[i] = parts[i].translate(tuple(shifts[group]))
    parts[1] = transform(base, rotation, translation)
    return {'input':'U_body_8108.zip / U_body_8108.stp', 'part_id':1,
        'rotation':rotation.tolist(), 'translation':translation.tolist(), 'shifts':shifts,
        'motor_center_height':height, 'old_motor_center_height':58.999984,
        'row_spacing':[float(np.linalg.norm(rows[i+1].mean(axis=0)-rows[i].mean(axis=0))) for i in range(2)],
        'main_outer_width':main_right-main_left, 'pitch_outer_width':pitch_right-pitch_left,
        'old_outer_width':108, 'opening_diameter':67, 'old_opening_diameter':80,
        'part_scale':1}


def fit_pitch_link(parts, eye, base_fit, transform, frame, cylinder, ball):
    shape = cq.importers.importStep(str(INPUTS/'pitch-replacement-0.stp')).val().Solids()[0]
    holes = sorted(circle_centers(shape,3), key=lambda p:p[1])
    assert len(holes) == 4
    flange0, flange1 = (holes[0]+holes[1])/2, (holes[2]+holes[3])/2
    axis = holes[1]-holes[0]
    axis /= np.linalg.norm(axis)
    # Keep the existing two eye-to-flange distances (source assembly is asymmetric).
    extensions = [22.499996925141044,24.626859904889894]
    local_lower = flange0-axis*extensions[0]
    local_upper = flange1+axis*extensions[1]
    span = local_upper-local_lower
    length = float(np.linalg.norm(span))
    old_lower = np.array([345,58.999984,-29])
    upper = np.array([183.062295,270.186359,39])
    pivot = np.array([240,58.999984,-29])+np.array(base_fit['shifts']['pitch_right'])
    delta = upper-pivot
    cosine = (np.dot(delta,delta)+105**2-length**2)/(210*np.linalg.norm(delta[:2]))
    assert abs(cosine) <= 1, 'Replacement pitch link cannot reach the receiver'
    heading, offset = math.atan2(delta[1],delta[0]),math.acos(cosine)
    angle = min([heading+offset,heading-offset],key=lambda a:abs(math.atan2(math.sin(a),math.cos(a))))
    lower = pivot+np.array([105*math.cos(angle),105*math.sin(angle),0])
    target_axis = (upper-lower)/length
    # Choose the handedness that puts the bent-arm offset outward toward the
    # receiver, keeping the eye shanks almost in the motor's rotation plane.
    target_x = np.cross(target_axis,np.array([0,0,1]))
    target_x /= np.linalg.norm(target_x)
    source_axis = span/length
    source_x = np.array([1,0,0])
    source_frame = np.column_stack([source_x,np.cross(source_axis,source_x),source_axis])
    target_frame = np.column_stack([target_x,np.cross(target_axis,target_x),target_axis])
    rotation = target_frame @ source_frame.T
    translation = lower-rotation@local_lower
    parts[49] = transform(shape,rotation,translation)
    stem = rotation@axis
    parts[53] = transform(eye,frame(stem),lower)
    parts[51] = transform(eye,frame(-stem),upper)
    parts[52] = ball.translate(tuple(lower))
    parts[50] = ball.translate(tuple(upper))
    r = np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    parts[17] = transform(parts[17],r,pivot-r@pivot)
    parts[17] = parts[17].cut(cylinder(3.25,240,(lower[0],lower[1],-190))).clean()
    parts[38] = parts[38].cut(cylinder(3.25,240,(upper[0],upper[1],-190))).clean()
    offset = np.linalg.norm((flange1-flange0)-axis*np.dot(flange1-flange0,axis))
    return {'input':'pitch-replacement-0.stp', 'part_id':49, 'rotation':rotation.tolist(),
        'translation':translation.tolist(), 'part_scale':1, 'hole_diameter':6,'old_hole_diameter':5,
        'flange_axis_span':float(np.dot(flange1-flange0,axis)), 'old_flange_axis_span':219,
        'lateral_offset':float(offset),'old_lateral_offset':63,
        'eye_to_flange_distances':extensions,'eye_to_eye':length,
        'old_eye_to_eye':float(np.linalg.norm(upper-old_lower)),
        'stem_direction':stem.tolist(), 'lower_flange':(rotation@flange0+translation).tolist(),
        'upper_flange':(rotation@flange1+translation).tolist(),
        'link':{'motor':3,'crank':17,'shaft':49,'lower_ball':52,'upper_ball':50,
          'lower_eye':53,'upper_eye':51,'pivot':pivot.tolist(),'lower':lower.tolist(),
          'upper':upper.tolist(),'length':length,'crank_angle_deg':math.degrees(angle)}}


def add_opposite_pitch(parts, names, pitch_fit, transform, cylinder):
    """Mirror the six right-hand solids, then close them on the actual left pins."""
    right = pitch_fit['link']
    mirror_z = -54.0  # Motor-base symmetry plane; receiver has a different center.
    def point(p):
        return np.array([p[0], p[1], 2*mirror_z-p[2]])
    source_lower, source_upper = point(right['lower']), point(right['upper'])
    pivot = point(right['pivot'])
    # Outer left receiver face plus half the existing nine-mm ball width.
    upper = np.array([right['upper'][0], right['upper'][1], -154.671573-9.5])
    length = right['length']
    delta = upper-pivot
    cosine = (np.dot(delta,delta)+105**2-length**2)/(210*np.linalg.norm(delta[:2]))
    assert abs(cosine) <= 1, 'Mirrored pitch link cannot reach the left receiver'
    heading, offset = math.atan2(delta[1],delta[0]), math.acos(cosine)
    initial = math.radians(right['crank_angle_deg'])
    angle = min([heading+offset,heading-offset],
        key=lambda a:abs(math.atan2(math.sin(a-initial),math.cos(a-initial))))
    lower = pivot+np.array([105*math.cos(angle),105*math.sin(angle),0])
    a, b = (source_upper-source_lower)/length, (upper-lower)/length
    cross = np.cross(a,b)
    skew = np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
    rod_rotation = np.eye(3)+skew+skew@skew/(1+np.dot(a,b))
    crank_delta = angle-initial
    crank_rotation = np.array([[math.cos(crank_delta),-math.sin(crank_delta),0],
        [math.sin(crank_delta),math.cos(crank_delta),0],[0,0,1]])
    ids = {}
    for key in ['crank','shaft','upper_ball','upper_eye','lower_ball','lower_eye']:
        shape = parts[right[key]].mirror('XY',(0,0,mirror_z))
        if key == 'crank':
            placed = transform(shape,crank_rotation,pivot-crank_rotation@pivot)
        elif key == 'lower_ball':
            placed = shape.translate(tuple(lower-source_lower))
        elif key == 'upper_ball':
            placed = shape.translate(tuple(upper-source_upper))
        else:
            placed = transform(shape,rod_rotation,lower-rod_rotation@source_lower)
        ids[key] = len(parts)
        names[len(parts)] = f'M3_{key}'
        parts.append(placed)
    parts[38] = parts[38].cut(cylinder(3.25,240,(upper[0],upper[1],-190))).clean()
    return {'source_motor':4, 'motor_base_mirror_plane_z':mirror_z,
        'receiver_center_plane_z':(-154.671573+29.5)/2,
        'receiver_offset_from_mirrored_right_mm':float(upper[2]-source_upper[2]),
        'part_scale':1, 'part_ids':ids,
        'link':{'motor':2,**ids,'pivot':pivot.tolist(),'lower':lower.tolist(),
            'upper':upper.tolist(),'length':length,'crank_angle_deg':math.degrees(angle)}}
