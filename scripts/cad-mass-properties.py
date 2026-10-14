"""Reproducible mass properties from the same STEP revision used by the viewer.

STEP coordinates are mm. OCCT volume properties have units mm^3 / mm^5;
multiply by rho*1e-9 / rho*1e-15 for kg / kg m^2, respectively.
The tensor is about each part's centroid, expressed in source STEP XYZ axes.
"""
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'cad/sr6_mini_complete.stp'
MATERIALS = {
    'al6061': {'name': '6061 铝合金', 'densityKgM3': 2700, 'color': '#58b8d8',
               'source': 'https://www.cctprecision.com/materials/aluminum-data-sheet/'},
    'steel': {'name': '钢', 'densityKgM3': 7850, 'color': '#b5c3d4',
              'source': 'https://www.ssab.com/en/support/product-material-data/steel/20-questions'},
    'ss304': {'name': '304 不锈钢', 'densityKgM3': 7900, 'color': '#c7ad83',
              'source': 'https://atlassteels.com.au/documents/atlas_wire_bar.pdf'},
    'motor': {'name': 'R0 电机总成（实测质量）', 'densityKgM3': None, 'color': '#aa92dc'},
    'unknown': {'name': '材质待确认', 'densityKgM3': None, 'color': '#d6a45d'},
    'fixed': {'name': '固定总成', 'densityKgM3': None, 'color': '#405260'},
}


def assignment(parts):
    result = {p['id']: {'id': p['id'], 'name': p['name'], 'label': p['name'],
                        'material': 'fixed', 'included': False, 'motion': 'fixed',
                        'reason': '固定组件，不计入运动质量'} for p in parts}

    def assign(ids, label, material, motion, included=True, reason=''):
        for part_id in ids:
            result[part_id].update(label=label, material=material, motion=motion,
                                   included=included, reason=reason)

    assign([1], '固定底座钣金', 'al6061', 'fixed', False, '固定组件，不计入运动质量')
    links = [
        (1, 15, 20, 21, 18, 22, 19), (2, 16, 25, 26, 23, 27, 24),
        (3, 58, 59, 62, 60, 63, 61), (4, 17, 49, 52, 50, 53, 51),
        (5, 14, 30, 31, 28, 32, 29), (6, 0, 35, 36, 33, 37, 34),
    ]
    for motor, crank, rod, lower_ball, upper_ball, lower_eye, upper_eye in links:
        assign([crank], f'M{motor} 一级摇臂', 'al6061', f'crank{motor}')
        assign([rod], f'M{motor} ' + ('俯仰二级钣金臂' if motor in [3, 4] else '二级光轴'),
               'al6061' if motor in [3, 4] else 'steel', f'rod{motor}')
        assign([lower_ball], f'M{motor} 下鱼眼内球', 'steel', f'crank{motor}')
        assign([upper_ball], f'M{motor} 上鱼眼内球', 'steel', 'platform')
        assign([lower_eye], f'M{motor} 下鱼眼外壳', 'steel', f'rod{motor}')
        assign([upper_eye], f'M{motor} 上鱼眼外壳', 'steel', f'rod{motor}')
    assign([38], '接收器主体钣金', 'al6061', 'platform')
    assign([46], '接收器内侧钣金', 'al6061', 'platform')
    assign([39], 'R0 电机 · 388 g', 'motor', 'platform', True, '质量实测；重心及惯量按均匀圆柱外形近似')
    assign([40], 'R0 电机附属件', 'motor', 'platform', False, '质量统一计入 R0 电机 388 g 总成，避免重复')
    assign([41, 45], 'R0 圆盘法兰', 'al6061', 'twist')
    assign([47, 48], 'R0 输出夹持条', 'al6061', 'twist')
    assign([42, 43, 44], 'R0 小螺钉', 'ss304', 'twist')
    assign([54, 55, 56, 57], '鱼眼钢套', 'steel', 'platform')
    for i in range(64, 72):
        assign([i], '接收器连接' + ('螺栓' if i % 2 == 0 else '螺母'),
               'ss304', 'platform')
    assign(list(range(72,78)), '鱼眼钢套', 'steel', 'platform')
    return result


def properties(shape, density):
    volume = shape.Volume()
    center = np.array(cq.Shape.centerOfMass(shape).toTuple())
    tensor = np.array(cq.Shape.matrixOfInertia(shape)) * density * 1e-15
    return volume, center, volume*density*1e-9, tensor


def validate_units():
    # Independent analytical box, including translation invariance of centroidal I.
    box = cq.Workplane('XY').box(20, 30, 40).val()
    _, center, mass, tensor = properties(box, 2700)
    expected = np.diag([.03**2+.04**2, .02**2+.04**2, .02**2+.03**2])*mass/12
    assert np.allclose(center, 0, atol=1e-9)
    assert abs(mass-.0648) < 1e-10
    assert np.allclose(tensor, expected, rtol=1e-9, atol=1e-14)
    assert np.allclose(properties(box.translate((100, 200, 300)), 2700)[3], expected, rtol=1e-9, atol=1e-14)


def main():
    validate_units()
    data = json.loads((ROOT/'public/sr6-cad.json').read_text(encoding='utf-8'))
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert digest == data['sha256'], 'Mass data must use the displayed STEP revision'
    solids = cq.importers.importStep(str(SOURCE)).val().Solids()
    assert len(solids) == len(data['parts']) == 78
    catalog = assignment(data['parts'])
    for p, shape in zip(data['parts'], solids):
        row = catalog[p['id']]
        # Export/import can change integration results slightly, but not IDs.
        assert np.linalg.norm(np.array(shape.Center().toTuple())-p['center']) < .01
        row['volumeMm3'] = shape.Volume()
        row['centerMm'] = list(cq.Shape.centerOfMass(shape).toTuple())
        row['massKg'] = None
        row['inertiaCentroidKgM2'] = None
        row['principalKgM2'] = None
        row['method'] = '未计算'
        rho = MATERIALS[row['material']]['densityKgM3']
        if rho:
            assert shape.isValid(), f'Invalid material solid {p["id"]}'
            volume, center, mass, tensor = properties(shape, rho)
            row['method'] = 'STEP 实体体积积分，均匀材质'
        elif p['id'] == 39:
            # Source motor geometry is invalid. Its measured mass is reliable,
            # but CAD-envelope cylinder inertia is explicitly an approximation.
            bounds = shape.BoundingBox()
            radius = max(bounds.ylen, bounds.zlen)*.0005
            length = bounds.xlen*.001
            center = np.array([(bounds.xmin+bounds.xmax)/2,
                               (bounds.ymin+bounds.ymax)/2, (bounds.zmin+bounds.zmax)/2])
            mass = .388
            tensor = np.diag([mass*radius*radius/2,
                              mass*(3*radius*radius+length*length)/12,
                              mass*(3*radius*radius+length*length)/12])
            row['method'] = '388 g 实测质量 × 均匀圆柱外形近似'
            row['approximation'] = {'shape': 'cylinder', 'axis': 'STEP X',
                                    'radiusM': radius, 'lengthM': length}
        else:
            continue
        principal = np.linalg.eigvalsh(tensor)
        assert mass > 0 and np.all(principal > 0)
        assert principal[-1] <= principal[0]+principal[1]+1e-10
        row.update(centerMm=center.tolist(), massKg=mass,
                   inertiaCentroidKgM2=tensor.tolist(), principalKgM2=principal.tolist())
    included = [p for p in catalog.values() if p['included']]
    assert all(p['massKg'] is not None for p in included)
    result = {
        'version': 1, 'source': SOURCE.name, 'sha256': digest,
        'coordinateFrame': 'Original STEP XYZ; tensors about each part centroid',
        'materials': MATERIALS, 'parts': list(catalog.values()),
        'includedMassKg': sum(p['massKg'] for p in included),
        'includedCount': len(included), 'virtualStickMassKg': 0,
        'payloadIsAdditional': True,
    }
    target = ROOT/'public/sr6-mass-properties.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    report = [
        '# SR6 材质、质量与转动惯量', '',
        f'源文件：`{SOURCE.name}`；SHA-256：`{digest}`。', '',
        f'共 78 个 CAD 实体，{len(included)} 件计入动力学；机构运动质量合计 **{result["includedMassKg"]:.6f} kg**。', '',
        '按用户最后确认，所有铝件（包括原先指定的 5052 钣金件）统一按 6061、2700 kg/m³ 计算并使用同一种材质颜色。', '',
        '以下惯量是关于各零件质心的三个主惯量，按从小到大排列，单位 kg·m²。'
        '主轴一般不与 STEP XYZ 或接收器 Fwd / Thrust / Side 重合；完整质心坐标与 STEP 轴系张量见 '
        '[JSON 数据](../public/sr6-mass-properties.json)。', '',
        'R0 电机质量取用户实测 388 g，因原 CAD 电机实体无效，重心与惯量按其圆柱外形近似；'
        '附属件 #40 不重复加质量。其他计入零件取均匀材质的 STEP 体积积分。'
        '螺纹使用 CAD 的简化几何，结果是模型估算，不是实物称量。', '',
        '固定底座与六个底部电机不计入运动质量。按用户最后确认，运动螺钉、螺母计入。'
        '虚拟杆质量为零；输入重力为杆尖额外集中负载，不重复包含机构质量，未假设负载自身的转动惯量。', '',
        '| 零件 | 材质 | 质量 g | I₁ | I₂ | I₃ | 计入 |',
        '|---|---|---:|---:|---:|---:|---|',
    ]
    for p in catalog.values():
        moments = p['principalKgM2']
        moment_text = ' | '.join(f'{v:.6e}' for v in moments) if moments else '— | — | —'
        mass_text = f'{p["massKg"]*1000:.3f}' if p['massKg'] is not None else '—'
        report.append(f'| #{p["id"]} {p["label"]} | {MATERIALS[p["material"]]["name"]} | {mass_text} | {moment_text} | {"是" if p["included"] else p["reason"]} |')
    report.extend(['', '## 密度依据', '', '| 材料 | 密度 kg/m³ | 来源 |', '|---|---:|---|'])
    for material in MATERIALS.values():
        if material.get('source'):
            report.append(f'| {material["name"]} | {material["densityKgM3"]} | [供应商资料]({material["source"]}) |')
    report.extend(['', '## 动力学实现', '',
        '同一刚体上的零件使用平行轴定理合成为 14 个运动组件（6 根一级臂、6 组二级臂、接收器平台、R0 输出组件）。'
        '逐组件计算质心平动惯性 m·a，以及转动惯性 I·α + ω×(I·ω)。'
        '经质心位置/姿态雅可比用虚功映射到接收器的五个广义坐标，再按现有阻尼最小范数规则分配给六个电机。', '',
        '坐标采用原始 STEP 轴系，重力补偿方向为 STEP +X。输入平移以 mm 为单位，角度以 rad 为单位；'
        '惯量和力始终采用 SI 单位。R0 输出运动会计入六电机承担的结构反作用，未输出独立 R0 电机扭矩。', '',
        '当前刚体姿态服从 CAD 运动约束；不额外求解鱼眼连杆的自由轴向自旋，也未加入电机内部转子的独立转动、'
        '摩擦、传动效率、柔性变形或预紧内力。R0 电机采用外形均匀分布估算，实际质量分布不同会改变结果。', '',
        '以播放实际时间采样，速度和加速度两级 10 Hz 低通；质心线加速度各分量限幅 ±20 m/s²。'
        '暂停仅重置下一次采样历史，保留最后一帧的动力学结果；定位与手动输入清空加速度。', '',
        '重新生成：`pnpm cad:mass`。验证：`pnpm test:dynamics`。', '',
    ])
    (ROOT/'docs/MASS-PROPERTIES.md').write_text('\n'.join(report), encoding='utf-8')
    print(f'{len(catalog)} parts; {len(included)} included; {result["includedMassKg"]:.6f} kg', flush=True)
    for p in included:
        print(f'#{p["id"]:02d} {p["label"]}: {p["massKg"]*1000:.3f} g; I principal = {p["principalKgM2"]}', flush=True)


if __name__ == '__main__':
    main()
