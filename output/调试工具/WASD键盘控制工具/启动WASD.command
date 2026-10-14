#!/bin/zsh
task_script_dir="${0:A:h}"
cd "$task_script_dir"
./c10b-wasd
task_exit_code=$?
stty sane
if (( task_exit_code != 0 )); then
  print "工具退出，状态码：$task_exit_code"
fi
print "按回车关闭窗口。"
read
