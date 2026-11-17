# 提交前验证（2026-11-02）

- Windows、Python 3.12.14：13 项控制状态机测试、11 项重放缓冲测试通过。
- 同次 unittest 发现 26 项，2 项 ROS 消息边界测试因未安装 ROS 2 消息库而跳过。见 [原始输出](unit-tests.log)与[环境记录](local-checks.json)。
- 当前 Python 有 NumPy、没有 OpenCV，因此没有运行 3 项视觉图像测试。不能将其计为通过。
- 对三组归档日志重新执行 `scripts/audit_replay_run.py`，核对配置、图像时间戳关联、实际延迟、首次停止状态与之后的指令。汇总与原始日志字节哈希共同保存在 summaries 目录。
- 16 个 Python 文件通过语法解析，9 个 shell 脚本通过 Git Bash `bash -n`；场景 XML 解析和文档相对链接检查通过。32 个对应的部署文件与共享目录内容一致；12 份归档原始文件哈希匹配。根 README 的 Git blob 与 main 一致，见 [静态检查](static-checks.json)。
- 本次发布整理没有在 Windows 重跑 ROS 构建或 Gazebo。仿真证据来自操作者在 Ubuntu 导出的原始日志，接触与无接触观察另存 observations.json。

Ubuntu 安装 ROS/OpenCV 后可运行完整测试：

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
bash scripts/build_baseline.sh
PYTHONPATH="$PWD/src/tb3_visual_baseline" python3 -m unittest discover -s src/tb3_visual_baseline/test -v
```
