# Unitree Go2 MuJoCo 本体包

本项目提供 Robonix Go2 仿真本体包，支持浏览器 MuJoCo WASM 和 native Python
MuJoCo 两种后端。启动时 ONNX 策略默认关闭，两种后端都可在界面中 Load/Disable
随包提供的 go2_rl_gym MoE 策略，无需重新训练。

![Go2 在 SceneSmith House 185 中](docs/media/native-scenesmith_house_185.png)

## 安装和配置

本机已安装 Robonix，源码位于 ~/robonix。环境需要 Docker Compose、Node.js 20+、
Python 3.10+、uv、rbnx。native 配置使用 WSLg/X11 和硬件 OpenGL；
普通 Linux 机器需要调整 sim/compose.native.yaml 的显示设备挂载。

```bash
cd ~/robot-unitree-go2_mujoco
cp .env.example .env
# 在 .env 中配置 ROBONIX_SOURCE_PATH、VLM_BASE_URL、VLM_API_KEY、VLM_MODEL。
bash scripts/bootstrap.sh
```

已经配置好本地 .env 的机器不要重复覆盖。凭据文件被版本控制忽略，
不会由网页静态服务器提供。仿真本身不需要 VLM；Pilot 和 Scene 使用这些配置。
运行资源和 ONNX 已包含在包内，普通启动不需要下载原始 SceneSmith 数据集。

## 人工启动

终端一启动仿真，以下方式选择一种：

```bash
cd ~/robot-unitree-go2_mujoco

# Web 物理后端，自动打开浏览器。
bash sim/start.sh --backend web --environment scenesmith_house_185

# 或 native 物理后端，打开 MuJoCo 原生窗口。
bash sim/start.sh --backend native --viewer --environment scenesmith_house_185

# 或 native 无窗口；保留 RGB-D 相机渲染。
bash sim/start.sh --backend native --headless --environment scenesmith_house_185
```

等待 ready 提示。Web 仿真在启动脚本打开的浏览器窗口运行。
普通控制入口为 http://127.0.0.1:5181/ ，native 控制页为
http://127.0.0.1:5181/?backend=native ，健康检查为
http://127.0.0.1:8766/health 。普通控制页连接现有仿真，不重建机器人。

终端二启动本体包：

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx boot --no-update-check
```

终端三查看能力并交互：

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx caps -v
rbnx chat
```

可输入以下任务：

- 拍摄前方相机，描述当前房间。
- 以每秒 0.18 米探索房间，最长运行 120 秒，返回任务 ID。
- 查询探索进度，然后取消该探索任务。
- 列出 Scene 已观察到的物体。
- 使用 Scene 查找最近的桌子，生成安全接近位姿，然后导航过去。

探索是异步任务，启动后用任务 ID 查询状态。探索、手动驾驶和导航不要并发控制本体。
“桌子”等语义目标必须先被相机观察到；Scene 结合 RGB-D、地图和本体轮廓生成目标，
没有用 XML 真实物体坐标冒充视觉感知。看不到的区域应先探索。

## 策略和操作

Web 端策略选择和 `Load / Disable Policy` 位于右上角 `Simulation > Policy`，
加载结果以实际后端确认和状态为准。默认生产模式不注册 Web 键盘运动监听，
native viewer 也不接收本地运动按键，运动命令由 Robonix 下发。

仅调试本地驾驶时显式添加 `--dev`：

```bash
bash sim/start.sh --backend web --dev --environment go2_rl_stairs
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
```

dev 模式中 W/S 前后、A/D 转向、Q/E 横移、Space 停止、X 重置；
native viewer 额外支持 L 切换策略。`--dev` 不影响 Robonix Twist 接口。

当前策略来自 go2_rl_gym 的
go2_moe_cts_high_slope_thre_164k_0.6715。使用 5x45 维历史、50 Hz 推理、
0.002 秒物理步长和原始 PD/动作缩放，详见
[权重来源](assets/robots/go2/policy/moe_rough/UPSTREAM.md)。
此次没有换成前面讨论的 v4.2。

卸载 ONNX 后使用确定性基础步态，适合室内平地。楼梯和障碍赛道应加载 RL 策略。
加载/卸载保留机身位置和朝向；Reset 是另外的操作，建图或导航运行期间不要使用。

## 场景

| 环境 ID | 内容 |
| --- | --- |
| scenesmith_house_185 | 新的 House 185：客厅和浴室，默认场景 |
| scenesmith_house_186 | 新的 House 186：卧室和浴室 |
| go2_rl_stairs | go2_rl_gym 原始阶梯几何 |
| go2_rl_track | go2_rl_gym 原始赛道几何 |

两个住宅都是单层、多房间数据集场景，区别于之前的 House 187。
四个场景均支持两种后端。切换场景前先停止 Robonix 和仿真，
再使用另一个 --environment 重启，避免沿用旧地图和语义位置。

只测策略时可只开仿真，不启动 Robonix：

```bash
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
# 在界面中 Load 后使用调试按键驾驶。
```

[场景来源与离线重新导入](docs/scenes.md)记录了源版本、校验和及命令。
楼梯/赛道不承担二维 Scene/Nav2 跨层语义导航验收。

## 验证与停止

```bash
npm test
python3 -m unittest discover -s primitives/tests

# 仿真和 Robonix 启动后检查传感器、本体注册和地图。
bash scripts/acceptance.sh --require-stack --require-map

# 导航目标必须根据当前地图选择。
bash scripts/acceptance.sh --navigate 5.6 2.4
bash scripts/acceptance.sh --semantic --object-id scene.object.table_001
bash scripts/acceptance.sh --explore --explore-duration 150 --explore-timeout 240 --explore-speed 0.18

# 独立加载四个 native 场景并验证相机、激光和策略切换。
docker exec mujoco_go2_sim python3 /workspace/sim/tests/native_smoke.py --environment all --policy
```

验收会移动仿真机器人，请先取消其他运动任务。完整结果、运行限制和系统变更见
[实现与验收报告](docs/IMPLEMENTATION_REPORT.md)。

停止：

```bash
source scripts/env.sh
rbnx shutdown
bash sim/stop.sh
```

每个启动终端中的 Ctrl-C 也能停止对应生命周期。
仿真日志在 .runtime，Robonix 日志在 rbnx-boot/logs。

## 实现范围

四个原语为 go2_chassis、mid360_lidar、mid360_imu、front_camera；
上层复用 Mapping、Navigation、Scene；本地 skills/explore 在原有接口上
补充了本体轮廓检查、连通性检查、失败目标暂避、速度限制和取消确认。
soma.yaml 与 urdf/go2.urdf 描述本体尺寸、轮廓和安装变换。
本包不包含真实 Go2 硬件通信、语音或机械臂。

参考来源及许可证保留在 NOTICE、LICENSE、LICENSE.Apache-2.0 和各资源目录中。
更多细节见 [架构说明](docs/ARCHITECTURE.md) 与
[Robonix 接入说明](docs/ROBONIX_INTEGRATION.md)。
