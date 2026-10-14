# SR6 SIMU

SR6 机构运动学与 STEP 装配模拟器。Three.js 显示真实装配，支持实测 CAD 正逆解、Funscript 六轴控制、虚拟杆尖补偿，以及 Manifold WASM 干涉检测。

## 本地运行

```sh
pnpm install
pnpm dev
```

打开 http://localhost:5173。空格播放/暂停；各轴可设置 0–1 对应的总行程或总转角。虚拟杆补偿默认关闭，杆长默认 220 mm。

## 干涉检测

按已确认的机构约束，仅检查接收器主体、六个上端鱼眼（内球与外壳）及两根俯仰二级摇臂，共 15 件。交集显示红色，涉及零件显示特征边。WASM 运行于 Web Worker，默认并行数为浏览器报告的逻辑处理器数减一，至少一个。

这是运动学与网格干涉可视化，不是动力学仿真。排除件不参与干涉判断，详细约束见 [SIMULATOR.md](SIMULATOR.md)。

## 构建与部署

STEP 模式可下载固定在装配零位的完整 STEP（ZIP 内含 `.stp`，78 个实体）。该姿态与网页“恢复装配零位”一致，下载内容不随当前动画变化。构建时会核对 STEP 与网页 CAD 的 SHA-256，再生成 ZIP；压缩仅为满足静态资源单文件大小限制，不修改实体几何。

```sh
pnpm build
node node_modules/wrangler/bin/wrangler.js deploy --config dist/server/wrangler.json
```

构建脚本会将 CAD JSON 无损分块，满足 Cloudflare 单文件限制。部署前需完成 Wrangler 登录。

- 当前模型：[cad/sr6_mini_complete.stp](cad/sr6_mini_complete.stp)
- 建模说明：[cad/README.md](cad/README.md)
- 性能实测：[docs/performance/RESTRICTED-SCOPE.md](docs/performance/RESTRICTED-SCOPE.md)
- GPU 原型评估：[docs/performance/WEBGPU-REPORT.md](docs/performance/WEBGPU-REPORT.md)

测试原始日志和 WebGPU 实验依赖存放在忽略的 `work/perf/`，未纳入正式运行路径。
