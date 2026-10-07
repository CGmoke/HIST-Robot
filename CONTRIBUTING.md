# 贡献指南

感谢你的兴趣！这份文档说明**怎么提**以及**我们看什么**。

## 你可以做的四件事

| 类型 | 入口 | 说明 |
| --- | --- | --- |
| 🐛 报 bug | [Bug report](https://github.com/CGmoke/robot-forge/issues/new?template=bug_report.yml) | 请带上复现命令、期望结果、实际结果、环境版本 |
| 💡 提需求 | [Feature request](https://github.com/CGmoke/robot-forge/issues/new?template=feature_request.yml) | 说清"要解决什么问题"，而不是"要加什么函数" |
| 🤖 加新项目 | [New project](https://github.com/CGmoke/robot-forge/issues/new?template=new_project.yml) | **先读 [docs/adding-a-project.md](docs/adding-a-project.md)** |
| 🔧 提 PR | 直接开 PR | 小改动（文档、修错别字）可以直接提；大改动请先开 issue 对齐 |

不确定要不要做，**先开 issue 聊**，避免白写。

## 开发流程

```bash
# 1) fork 后克隆
git clone git@github.com:<你的用户名>/robot-forge.git
cd robot-forge

# 2) 建分支：feat/  fix/  docs/  refactor/  build/  test/
git checkout -b feat/add-xxx

# 3) 本地验证（提交前必须过）
cd packages/reusable_model && python -m pytest -q && cd ../..
```

ROS 2 项目的验证：

```bash
cd projects/tianyi25-sim/ros2_ws
colcon build --packages-select tianyi25_urdf tianyi25_sim --symlink-install
source install/setup.bash
ros2 launch tianyi25_sim gazebo.launch.py     # 确认能起来、/scan 与 /odom 有数据
```

## 提交信息规范

本仓库使用 [Conventional Commits](https://www.conventionalcommits.org/)，与已有历史保持一致：

```
feat: 新增 xxx 功能
fix: 修复 xxx 问题
docs: 更新 xxx 文档
refactor: 重构 xxx
build: 调整构建/依赖
test: 补充 xxx 测试
```

一条提交只做一件事。避免 `update`、`修改` 这类无信息量的描述。

## PR 检查清单

PR 模板会自动带上这份清单，请逐条确认：

- [ ] 本地测试通过（`python -m pytest` / `colcon build`）
- [ ] 新功能带测试，修 bug 带回归测试
- [ ] 文档已同步（受影响的 README 都改了）
- [ ] 没有提交绝对路径、私有数据、密钥、`build/` `install/` `log/` 等产物
- [ ] 涉及新项目时，根 `README.md` 的项目索引表和 `CHANGELOG.md` 已更新

## 代码规范

### Python

- 硬依赖只允许标准库 + `numpy`；重依赖（`opencv` / `pyserial` / `PyYAML`）**惰性导入**，不装也能用其余部分
- 不使用 `print()`，统一 `logging.getLogger(__name__)`
- 不写绝对路径、不读环境变量、不依赖任何具体项目的配置 —— `packages/` 下的代码必须与项目解耦
- 公开 API 写 `参数 / 返回 / 异常` 格式的 docstring，并尽量附可执行 doctest

### ROS 2

- `package.xml` 依赖完整且用 `exec_depend`（避免未安装时 CMake `find_package` 失败）
- launch 参数一律给默认值并写进 README 的参数表
- 每个功能包一个 `README.md`：定位、依赖、构建、运行、话题/TF、排错

### 文档

- 以中文为主，关键位置补英文摘要
- 命令必须**可直接复制执行**，并给出"怎么确认成功了"

## 不接受的内容

- 与机器人开发无关的代码或资源
- 无许可证或与 Apache-2.0 不兼容的第三方代码、模型、网格资源
- 只有一次大提交、无法复现的"代码倾倒"
- 直接提交二进制大文件（网格资源请说明来源与许可）

## 行为准则

保持专业与友善：讨论技术，不攻击个人。任何形式的骚扰、歧视性言论会被直接移出讨论。
