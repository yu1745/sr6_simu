# 220 mm 补偿的安装轴线搜索

范围：L1 ±30 mm、L0 ±60 mm、L2=0、Roll/Pitch 各 ±12.5°。固定球心，保留现有共轴双鱼眼布局，仅倾斜 M3/M4。原接收器参与碰撞检查；不包含 110 mm 预览副本中新画的凸台。

625 个网格样本中 567 个可达、58 个不可达。倾斜安装轴线不改变可达性。

| 方案 | 显著干涉样本 | 占可达样本比例 | 最大单对交集 mm³ | 最大鱼眼摆角 |
|---|---:|---:|---:|---:|
| 原安装轴线 | 230 | 40.56% | 6.80612 | 35.0688° |
| 直接沿用 110 mm 数量方案 | 220 | 38.80% | 6.02492 | 34.2185° |
| 220 mm 最坏摆角方案 | 216 | 38.10% | 2.10562 | 28.7725° |
| 220 mm 数量方案 | 216 | 38.10% | 2.15059 | 28.8575° |

采用交集 >0.001 mm³ 判据。两组 220 mm 候选都达到当前网格的 216 个不可调主支链干涉样本下限；优先选择最坏摆角方案。有限网格不能证明连续空间全局最优或无干涉。

## 候选方向

原轴线为接收器局部 STEP +Z。α=atan2(nx,nz)、β=atan2(ny,nz)，n=normalize(tan α,tan β,1)，不是界面的 Roll/Pitch。

| 安装位 | α | β | 总倾角 | 最大摆角：原 → 优化 |
|---|---:|---:|---:|---|
| M3 | −4.31° | +5.50° | 6.9713° | 35.0688° → 28.7725° |
| M4 | +0.86° | −2.79° | 2.9191° | 29.9609° → 27.9732° |

数量方案是 M3 (−4°, +6°)、M4 (−1°, −6°)，实际交集结果未优于上表。

4 个代表位姿经过原始 STEP B-rep 复核，>0.001 mm³ 干涉零件对与网格结果一致。最大交集代表位姿的 STEP 体积约 2.05807 mm³，网格体积约 2.10562 mm³。零位和选定已消除干涉位姿的 STEP 交集均为零。

- [搜索参数](mount-axis-search.json)，键 `220`、`count-220`。
- [推荐候选完整网格](workspace-220mm-mount-220.json)
- [直接沿用 110 mm 倾角的对照](workspace-220mm-mount-count-110.json)
- [STEP 复核](mount-axis-220-brep-verification.json)

```powershell
node scripts/search-mount-axes.mjs
node scripts/scan-compensated-workspace.mjs 5 cad/workspace-220mm-mount-220.json 220 side-off 220
node scripts/mount-design-witnesses.mjs 220 220 work/workspace-scan/mount-witnesses-220.json
python scripts/verify-workspace-contacts.py work/workspace-scan/mount-witnesses-220.json cad/mount-axis-220-brep-verification.json
```

仅研究参数，未更改线上模型或此前的 110 mm CAD 预览。
