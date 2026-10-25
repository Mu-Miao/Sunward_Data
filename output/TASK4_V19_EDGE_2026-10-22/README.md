# V19 EDGE 下载包

下载文件：`Sunward_V19_EDGE.hex`；对应调试文件：`Sunward_V19_EDGE.elf`。屏幕标题应为 `TASK 4 V19 EDGE`，低速接管阶段显示EDGE。

捕线连续30ms后允许最外侧单探头接管低速巡线，70mm/s基速、最外侧20/120；内侧连续60ms且阶段至少200ms后渐升。低速阶段受120mm/2.5秒/45°约束；全白时降为20/80并保留原500ms及位移/角度保护。正常直线270、半圆240mm/s及正常换段连续运动保留。

22组ASan/UBSan、50个ARM单元及HEX/向量/版本校验通过。V19未烧录、未实车验证，pyOCD保持关闭。

本地分支：`codex/task4-v19-edge`；提交：`17bb15e9089c0836eb33ffc675cd115941fbb477`。尚未推送。V14、V18分支保持原提交。

源码工作树：`/Users/mzh/.codex/worktrees/task4-v14-rolling/Sunward`。完整说明、验证日志、944字节instance布局和源码/测试/固件SHA256见同目录其他文件。

`V18_to_V19_EDGE.patch`针对干净V18提交`f2802b6`，包含源码、测试及现场依据；不包含二进制归档。应用前可用 `git apply --check`。

下次仅在需要采集时启动附带脚本并显式指定此ELF：

```sh
"/Users/mzh/Library/Application Support/pipx/venvs/pyocd/bin/python" pyocd_capture.py --elf Sunward_V19_EDGE.elf --hz 20
```

该命令不会烧录或启动电机，连接后仍会先核对FLASH。本轮没有运行此命令。
