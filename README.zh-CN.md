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
- 在 `scenesmith_multilevel_house` 中上楼、下楼、去一楼或去二楼。

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
基础步态的原地旋转同时生成前后和横向切向落足，并在机身大幅倾斜时抑制运动指令；
正反向持续旋转、六方向行走和速度反转均有 native 物理回归测试。

## 场景

| 环境 ID | 内容 |
| --- | --- |
| scenesmith_house_185 | 新的 House 185：客厅和浴室，默认场景 |
| scenesmith_house_186 | 新的 House 186：卧室和浴室 |
| go2_rl_stairs | go2_rl_gym 原始阶梯几何 |
| go2_rl_track | go2_rl_gym 原始赛道几何 |
| scenesmith_multilevel_house | House 191 + House 188 与人工标定直楼梯的双层场景 |

两个住宅都是单层、多房间数据集场景，区别于之前的 House 187。
五个场景均可由统一环境清单选择。双层换层闭环使用 native 后端完成验收。
切换场景前先停止 Robonix 和仿真，
再使用另一个 --environment 重启，避免沿用旧地图和语义位置。

只测策略时可只开仿真，不启动 Robonix：

```bash
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
# 在界面中 Load 后使用调试按键驾驶。
```

[场景来源与离线重新导入](docs/scenes.md)记录了源版本、校验和及命令。
楼梯/赛道不承担二维 Scene/Nav2 跨层语义导航验收。

双层 demo 推荐 native 后端：

```bash
bash scripts/install-prebuilt-maps.sh
bash sim/start.sh --backend native --environment scenesmith_multilevel_house
```

它使用独立的一、二楼地图和部署级 `floor_transition` skill，不修改系统 Scene、
Mapping 或 Navigation 服务。设计、标注字段、scope 和人工验收步骤见
[双楼层换层 Demo](docs/MULTI_FLOOR_DEMO.zh-CN.md)。

## 上下楼技能

`floor_transition` 是部署级、固定场景换层技能，用于让 Go2 在已经建图并完成楼梯
标定的双层环境中执行以下命令：

- `UP`：从一楼经已标定楼梯到二楼；
- `DOWN`：从二楼经同一楼梯回到一楼；
- `GO_TO_FLOOR(1|2)`：根据当前楼层决定是否换层；目标就是当前楼层时幂等成功，
  不移动机器人；
- `status` 和 `cancel`：按 `run_id` 查询阶段或请求停止当前换层任务。

该技能不修改 Robonix 的系统服务。它通过 Atlas 使用现有 chassis、IMU、Mapping 和
Navigation capability，通过 simulator bridge 管理 `moe_rough` 策略，并只读加载场景
拥有的 [`multifloor.yaml`](assets/environments/scenesmith_multilevel_house/multifloor.yaml)。
Scene 服务不是换层执行的关键依赖；楼梯坐标也不会写入 Scene 数据库。

### 适用范围

当前实现适用于满足以下条件的仿真或受控部署：

- 环境 ID 为 `scenesmith_multilevel_house`，楼层固定为 1 和 2；
- 使用人工测量并验证过的单段直楼梯，入口、中心线、目标高度和落地点稳定；
- 每个楼层有独立的二维占据地图和 RTAB-Map 定位数据库；
- 机器人能够在楼梯入口前使用平地步态定位，在楼梯段使用已验证的粗糙地形策略；
- 楼梯区域无人、无动态障碍，且台阶尺寸、坡度、摩擦和平台空间与标定环境一致；
- 任务只要求换层或到达指定楼层，不要求跨楼层语义物体导航。

典型用途包括固定楼宇 Demo、算法联调、双层仿真回归、地图切换验证，以及在已知楼梯
上的策略测试。将它移植到另一个 MuJoCo 场景时，应将其视为需要重新标定和重新验收的
场景专用 skill，而不是直接复用坐标。

### 前置条件

启动换层任务前必须满足：

1. 使用 native MuJoCo 启动正确场景；当前闭环只在 native 后端完成过验收。
2. 两层地图存在于 `assets/maps/`，并通过 `scripts/install-prebuilt-maps.sh` 安装到
   Mapping 地图目录。地图文件只有被纳入项目发布制品或提交到远程仓库后，其他使用者
   才能在克隆项目后直接安装；脚本本身不会上传地图。当前完整地图 ID 为
   `scenesmith_multilevel_floor_1_v2` 和 `scenesmith_multilevel_floor_2_v2`，分别包含
   439 和 787 个 RTAB-Map 节点。仓库以 xz 压缩包保存数据库，安装脚本会在运行时
   地图目录中将其展开为 `rtabmap.db`。
3. `multifloor.yaml` 中的 `map_id`、楼梯入口、中心线、目标 X、楼层高度、速度和安全
   阈值必须与当前场景一致。
4. chassis odom、IMU、Mapping pose/load-map 和 Nav2 navigate/status/cancel capability
   均为可用状态，simulator bridge 健康且能够确认策略加载/卸载。
5. 物理楼层必须与 skill 持久化的 `current_floor` 一致。更换场景、重置机器人位置或
   手工把机器人放到另一层后，不能沿用旧楼层状态和旧定位会话。
6. Explore、普通 Navigation、手动驾驶和其他速度发布者均已停止。换层 skill 假设
   自己独占 `/cmd_vel` 和楼梯策略生命周期。

### 执行流程和策略生命周期

Skill 激活时会验证场景，卸载可能已经加载的 `moe_rough`，等待 odom，然后以实际
odom 位姿加载持久化楼层地图。每次换层按以下顺序执行：

1. 确保 `moe_rough` 已卸载，使用确定性平地步态和 Nav2 到达楼梯入口附近；
2. 进入连接区后停止使用 Nav2，由 skill 低速闭环对准人工标注的入口朝向；
3. 校验入口位置和朝向后加载 `moe_rough`；
4. 沿标定中心线发布有界 Twist，并持续检查横向偏差、机身直立度、行进方向、终点 X
   和落地高度；
5. 到达平台后立即停止并卸载 `moe_rough`，使用平地步态转到目标楼层标准朝向；
6. 使用当前实际 odom 作为定位初值，加载目标楼层地图；地图加载成功后才更新并持久化
   `current_floor`。

如果 `moe_rough` 在任务开始前已经加载，skill 会先将其卸载，再在楼梯阶段重新加载。
任一次策略切换没有得到后端确认，任务都会失败并停止，不会盲目继续运动。任务结束时
策略应为 unloaded；不要在换层过程中从 UI 或其他节点并发切换策略。

### 地图、Nav2 和 Scene 的换层语义

- **Mapping**：由 skill 显式调用 `load_map` 切换到目标楼层的预建地图，模式为
  `localization`。这是地图加载，不是重新建图；新观测不会自动写回随包地图。
- **Nav2**：不重启。Mapping 发布新 `/map` 和定位关系后，Nav2 的 global/local
  costmap static layer 会接收新地图，后续导航目标应使用目标楼层坐标。换层期间不得保留
  另一个活动导航任务。当前 Demo 没有额外调用 `clear_costmap`，移植到更复杂环境时建议
  增加新地图确认和 global/local costmap 清理屏障。
- **Scene**：不会自动切换楼层语义对象。当前 skill 只读取场景文件中的楼梯标注，
  不更新 Scene object graph，也不提供“到二楼某个物体附近”。若需要该能力，必须为对象
  增加楼层归属，并在换层后切换或过滤对应楼层的语义数据。

### 安全边界和不支持的场景

当前版本明确不支持：

- 未知楼梯自动检测、视觉发现楼梯或在线估计台阶几何；
- 未标定场景、旋转楼梯、多段楼梯、多个候选楼梯、电梯和自动选择换层连接；
- 楼梯上的动态避障、人群通行、湿滑或显著改变的摩擦条件；
- 一张二维地图覆盖多层、跨层连续 Nav2 路径规划；
- 换层后自动探索、自动更新预建地图或跨楼层物体导航；
- 真实 Go2 硬件安全认证。当前安全阈值和测试结果仅覆盖该 MuJoCo 场景。

位置、航向、中心线、直立度、目标高度、超时或地图加载任一检查失败时，skill 会发布
零速度并返回失败；只有物理换层和目标地图加载都成功后才提交楼层状态。

### 使用和验收

自然语言交互可输入“上楼”“下楼”“去二楼”或“去一楼”。不依赖 Pilot/VLM 的确定性
验收入口为：

```bash
bash scripts/floor-transition.sh up
bash scripts/floor-transition.sh down
bash scripts/floor-transition.sh floor 2
bash scripts/floor-transition.sh floor 1
```

脚本会动态发现正式 MCP 端点，打印 `run_id` 和阶段变化，并等待终态。验收时应同时确认
机器人实际到达目标平台、`current_floor` 正确、Mapping active map 与楼层一致、Nav2 保持
ACTIVE，以及任务结束后 `moe_rough.loaded=false`。完整状态机、标注字段和实测结果见
[双楼层换层 Demo](docs/MULTI_FLOOR_DEMO.zh-CN.md)。

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
[架构说明](docs/ARCHITECTURE.md)与
[双楼层换层 Demo](docs/MULTI_FLOOR_DEMO.zh-CN.md)。

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
补充了本体轮廓检查、连通性检查、失败目标暂避、速度限制和取消确认；
`skills/floor_transition` 负责固定双层场景的安全换层和地图切换。
soma.yaml 与 urdf/go2.urdf 描述本体尺寸、轮廓和安装变换。
本包不包含真实 Go2 硬件通信、语音或机械臂。

参考来源及许可证保留在 NOTICE、LICENSE、LICENSE.Apache-2.0 和各资源目录中。
更多细节见 [架构说明](docs/ARCHITECTURE.md) 与
[双楼层换层 Demo](docs/MULTI_FLOOR_DEMO.zh-CN.md)。
