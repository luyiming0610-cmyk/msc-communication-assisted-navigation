# 实验目录命名规则

自 2026-07-19 起，对外可见的实验目录统一采用：

`实验编号-本实验第几次.实验名称`

例如：`3-2.统一遭遇避障实验` 表示第 3 类实验的第 2 次控制方案实验。

## 当前目录映射

| 新目录名 | 原目录名 | 说明 |
|---|---|---|
| `1-1.实体机器人避障实验` | `real_robot_avoidance_v1` | Windows 主副本；Pi 和 WSL 上已经部署的副本仍保留原路径 |
| `2-1.仿真通信实验` | `simulation_comm_experiment_v1` | Webots world 与 launch 工作目录 |
| `3-1.局部避障锁存实验` | `controller_v2_local_latch_20260717` | 控制器开发证据第一阶段 |
| `3-2.统一遭遇避障实验` | `controller_v3_unified_encounter_20260717` | 控制器开发证据第二阶段 |
| `3-3.全传感器避障实验` | `controller_v4_full_sensor_bypass_20260717` | 当前冻结控制器及正式 Phase 4 证据 |

## 约束

- 新建的对外实验目录不再使用 `v1`、`v2` 等版本字样。
- 同一实验的重复运行使用第二个数字区分，例如 `4-1`、`4-2`。
- 协议版本、ROS 消息版本、控制器源码内部版本及既有 bag/trial ID 不重命名。这些是可复现性标识，不是展示目录名称。
- 历史 rosbag、日志和分析快照中的原始路径不回写；它们记录的是实验发生时的真实环境。需要定位时使用上表映射。
- 任何后续目录改名都必须同步更新启动脚本、`experiment_registry.csv`、`path_manifest.csv`、`EXPERIMENT_INDEX.md` 和 `PROJECT_HANDOFF.md`，并在提交前运行测试。

## 已知陷阱：WSL 构建镜像与 Windows 仓库的路径漂移（2026-07-20）

`~/epuck_ws/src/` 是 `src/` 在 WSL 原生文件系统下的**构建镜像副本**（colcon 只在原生 ext4 路径下编译/测试，不直接对 `/mnt/c/...` 编译），与 Windows 仓库的 git 检出是两份独立文件。2026-07-19 的目录改名只更新了 Windows 仓库（git 跟踪的源），没有同步刷新这份镜像；`test_analyze_static_v4_task.py` 在镜像中残留 2026-07-17 的旧版本（仍引用 `controller_v4_full_sensor_bypass_20260717/...`），导致 `colcon test` 在镜像里跑出 4 项"路径不存在"失败——但 Windows 仓库里的同一份文件当时已经是正确的 `3-3.全传感器避障实验/...`路径（含两级候选路径回退逻辑），不需要再次修改。

根因确认后用 `rsync -av`（不带 `--delete`）把 `src/` 重新同步进 `~/epuck_ws/src/`，重建 `epuck2_comm_interfaces`/`epuck2_comm`，194 项测试（165 现有 + 29 N2 新增）全部通过。**规则：任何目录改名后，除了上面第 25 条列出的文档/脚本，还必须重新同步 `~/epuck_ws/src/`，否则 WSL 侧的测试会针对一份过期镜像运行。**

