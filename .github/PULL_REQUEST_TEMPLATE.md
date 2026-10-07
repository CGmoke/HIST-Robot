<!-- 标题建议：<类型>(<范围>): <一句话>   例如：fix(reusable_model): 修复 gridmap 距离场边界越界 -->

## 这个 PR 做了什么

<!-- 一两句话说清动机和结果，不要只贴 diff -->

关联 issue：Closes #

## 改动类型

- [ ] 🐛 Bug 修复
- [ ] ✨ 新功能
- [ ] 📖 文档
- [ ] ♻️ 重构（不改变行为）
- [ ] 🔧 构建 / CI / 依赖
- [ ] 🤖 新增机器人项目

## 验证方式

<!-- 你是**怎么确认**它是对的？贴命令和实际输出，不要只写"本地测过" -->

```bash

```

## 检查清单

- [ ] 本地测试通过（`cd packages/reusable_model && python -m pytest`）
- [ ] ROS 2 改动已 `colcon build` 通过，且启动过相关 launch
- [ ] 新功能带测试；修 bug 带能复现原问题的回归测试
- [ ] 受影响的 README / 文档已同步更新
- [ ] 新项目已按 [docs/adding-a-project.md](../docs/adding-a-project.md) 自检
- [ ] 根 `README.md` 项目索引表与 `CHANGELOG.md` 已更新（如适用）
- [ ] 没有提交绝对路径、密钥、私有数据、`build/` `install/` `log/` 产物

## 备注

<!-- 破坏性变更、需要 reviewer 特别关注的点、后续 TODO -->
