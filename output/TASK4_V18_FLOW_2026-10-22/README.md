# V18 FLOW 下载包

基于用户指定的 V14 分支 `242b647ed1a8470ea77b5fc2576e4198fb08c912`；本地分支 `codex/task4-v14-rolling`，提交 `e79193d6f195df920783c3761ea51f8dcea636ee`。未推送。

下载文件：`Sunward_V18_FLOW.hex`。ELF：`Sunward_V18_FLOW.elf`。下载后标题应为 `TASK 4 V18 FLOW`。

C/D 在原窗口见线即低速预转；正常换段和前三圈 A 点直接衔接；第四圈终点停稳。转弯外轮最高100、内轮20mm/s，任务四转向位移预算从80改为120mm，4秒时限保留。巡航速度270/240mm/s、输出上限2200。

20组ASan/UBSan回归及50个ARM编译单元通过。正常四圈17次提示、3次移动接圈、最终停车和故障停车已软件验证。未烧录或实车验证。

源码工作树：`/Users/mzh/.codex/worktrees/task4-v14-rolling/Sunward`。在该目录运行 `python3 tools/tests/run_tests.py --sanitize`、`python3 tools/build_firmware.py` 可重测重编译。不要覆盖原 dev 的未提交改动。

`V14_to_V18_FLOW.patch` 为基线到新提交的完整源码差异；在干净的 V14 检出上可先 `git apply --check`，再按需应用。其他文件为修改说明、构建/回归日志和源文件/固件SHA256清单。
