# C6 TUM off_1 地图合并阶段无进展

本轮使用冻结的 C6 CPU 计时二进制，`RGBD.LocalMapMode: off`，属于正式 A-B/B-A/A-B 的第一个 A。它保留为失败轮，不能以之后成功的回放替换。最终轨迹没有正常导出；CSV 缓冲区只有 4693 条完整帧记录和一条未完成尾行，不能据此声称只执行了 4693 帧。

在帧文件十分钟以上没有进展后，两次 GDB 采样显示 LoopClosing 位于 `Optimizer::OptimizeEssentialGraph → MapPoint::EraseObservation`，Tracking 等待地图锁，LocalMapping 已停止。显式加载冻结二进制/共享库符号后的完整栈位于 IDE `artifacts/local_map_budget_20260924/c6/merge_timeout_stack.txt`。首次未加载符号的采样及全部原始日志保留在 outer `memory/lm01-c6cpu-tum-matrix/off_1/`。

`Optimizer.cc` 的该参考 KF 清理循环与 C0 `d80009e9` 相同：先以 `pRefKF->isBad()` 判断循环，再调用 `EraseObservation` 并重新读取参考 KF；空指针检查位于解引用之后。一个需要后续独立复现的机制是：擦除观测让 MP 退休、清空观测关系，却保留坏的参考 KF，随后再次擦除不再推进。当前线程栈支持“卡在此清理过程”，但没有冻结并导出该 MP 的完整对象状态，不能把这一机制写成已证实根因，也不能证明新增同步对触发时机没有影响。

本次未修改全局合并算法。已停止这个自有测试进程，保存 SIGTERM 记录及部分轨迹；没有生成 Atlas 文件。测试驱动现在支持严格校验二进制/配置后的续跑，每轮最多 900 秒、帧输出连续 300 秒无进展即失败，保留退出码和不完整尾行，继续其余独立轮次。原始 runner 哈希和恢复版本哈希均保留；运行中的脚本没有被修改。

这属于阻止生产验收的开放失败。已有局部事务 fixture/TSan 通过不能覆盖完整 System 的地图合并进度保证。
