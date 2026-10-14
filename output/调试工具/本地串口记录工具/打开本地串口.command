#!/bin/zsh
task_script_dir="${0:A:h}"
/opt/miniconda3/bin/python3 "$task_script_dir/serial_console.py"
print "按回车关闭此窗口。"
read
