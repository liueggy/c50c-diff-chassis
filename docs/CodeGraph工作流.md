# C50C / Eggy 的 CodeGraph 工作流

本仓库使用本机 CodeGraph CLI 1.6.2，索引位于 `.codegraph/`，由 `.gitignore` 排除。Git保存源码与流程，CodeGraph提供符号、调用关系和影响范围查询。新协作者需在自己的工作副本重新建立索引。

## 初始化与刷新

在仓库根目录执行：

```powershell
codegraph init --yes .   # 第一次建立索引
codegraph status --json
codegraph sync .         # 源码变动、切换分支后刷新
```

若索引损坏或工具提示需要重建，使用 `codegraph index .`。本轮没有安装新的全局工具、修改全局MCP设置或添加自动Git钩子；本地CLI即可完成以下流程。

## 开发前：查找与影响分析

```powershell
git status
git branch --show-current
codegraph sync .
codegraph query Robot_Select --limit 10
codegraph node Robot_Select --file CarType/diff_robot_init.c
codegraph callers Robot_Select --file CarType/diff_robot_init.c
codegraph callees Robot_Select --file CarType/diff_robot_init.c
codegraph impact Set_Pwm --file BALANCE/balance_task.c --depth 2
codegraph node --file CarType/Inc/eggy_chassis_config.h --limit 70
```

`Robot_Select`有多个车型定义，必须指定文件。优先从当前Diff_Car选用的文件出发，不把旧BALANCE/robot_select_init.c或其他车型命中的函数当成当前实现。

## 改完后：刷新、测试、编译、提交

```powershell
codegraph sync .
codegraph impact Set_Pwm --file BALANCE/balance_task.c --depth 2
codegraph affected CarType/diff_robot_init.c BALANCE/balance_task.c
python tests/test_eggy_profile.py
git -c core.whitespace=cr-at-eol diff --check
git diff --stat
```

在Keil选择Diff_Car并构建，读取新生成的日志，确认错误/警告和内存占用。CLI构建也可在仓库根目录使用：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b '.\USER\WHEELTEC.uvprojx' -t Diff_Car -o '..\OBJ\Eggy_build.txt'
Get-Content '.\OBJ\Eggy_build.txt' -Tail 15
```

UV4启动方式可能提前返回；应等待日志出现本次完整构建结尾，并结合HEX/AXF修改时间核对，不能只看启动命令退出码。测试与编译通过后，暂存明确文件、检查差异并提交，再执行 `codegraph sync .`。

## 本仓库的解析边界

- CodeGraph会索引不同车型、旧文件和预处理分支，不等同于解析Keil实际编译目标。`Deep_SelfCheck`虽在Eggy配置中关闭，仍会出现在图中。
- 宏引用、寄存器操作、函数指针和中断入口可能解析不完整。`query EGGY_TRACK_WIDTH_M`当前返回包含它的宏，但不保证提供完整的宏依赖关系；必要时用 `rg` 和原文件核对。
- GBK/GB18030中文注释在工具输出中可能乱码。符号查询仍可用，阅读中文应按原文件编码解码，不能为修正工具显示而批量改源码编码。
- `tests/test_eggy_profile.py`动态读取C源码，`affected`实测没有发现对应测试。因此修改底盘参数、停机和PWM逻辑时明确运行这个测试，不因“无受影响测试”跳过。
- 本轮只调用本地CLI，不依赖当前会话是否加载CodeGraph MCP工具。

## 初始化核验记录

2026-10-12首次初始化：248个文件、4797个节点、9115条关系，索引状态complete、pendingRefs=0、pendingChanges均为0。`Robot_Select`查询找到差速定义；`Set_Pwm`调用者查询找到ResponseControl、UnResponseControl、Deep_SelfCheck；指定balance_task.c的两层影响分析包含Balance_task。后续数量会随源码和工具版本变化。
