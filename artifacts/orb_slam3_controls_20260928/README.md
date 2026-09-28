# GF04 复现入口

源代码与数据保存在 `/home/ywl/Project/RDK_X5_orb_slam/orb_slam3_controls_20260928`。
本目录保存协议、工具与证据。已完成轮次不得原地重跑，使用新目录并相应设置
`harness/run.py` 的 ROOT 常量后重新冻结。`setup.py <新目录>` 从固定 Git 对象
导出算法源码，插入诊断 friend 声明及两版相同的一行关闭回环保护；
不下载或修改生产源码。初始原样关闭回环测试崩溃，详见 PLAN.md 修订。

构建命令（在隔离根目录运行，两版各独立完整构建）：

```sh
cmake -S harness -B build-official -DU="$PWD/official" -DVARIANT=official
cmake --build build-official --target replay -- -j2
cmake -S harness -B build-delivered -DU="$PWD/delivered" -DVARIANT=delivered
cmake --build build-delivered --target replay -- -j2
python3 harness/test_accuracy.py
python3 harness/run.py --phase smoke
python3 harness/run.py --phase formal
python3 harness/analyze.py
```

正式测试须在所有编译结束之后进行。测试工具会拒绝覆盖旧轮次和已有正式
freeze；失败轮保留并单列。GF03 桥接使用原冻结 executable/lib，不重编旧实验。
数据/依赖/解释边界见 PLAN.md 和最终 REPORT.md。
