#coding=UTF-8
import bluetooth
import sys
import time
from pynput.keyboard import Key, Listener

# 需要连接的设备名字和地址
target_name = 'WHEELTEC'
target_addr = '02:12:34:28:90:10'

# 扫描附近的蓝牙设备
nearby_devices = bluetooth.discover_devices(lookup_names=True)
print("附近所有可连的蓝牙设备:")
for device in nearby_devices:
    print(device)

# 查找指定名称和地址的设备
addr = None
for device in nearby_devices:
    if device[1].strip() == target_name and device[0] == target_addr:
        addr = device[0]
        print("device found!", target_name, " address is: ", addr)
        break

if addr is None:
    print("device not exist")
    sys.exit()  # 如果没有找到设备，退出程序

# 尝试连接到设备的各个端口
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
i = 1  # 从端口1开始尝试，因为端口0通常是保留的
while i < 255:  # 遍历端口号，进行连接
    try:
        sock.connect((addr, i))
        print("连接成功，端口：", i)
        break
    except Exception as e:
        print("端口：", i, "连接失败", e)
        i += 1

if i == 255:
    print("所有端口连接失败")
    sys.exit()  # 如果所有端口都连接失败，退出程序

# 实时读取键盘输入并发送数据
print("按下对应按键，控制移动方向：")
print("左前  前  右前             u  i  o")
print("左          右             j     l")
print("左后  后  右后             m  ，  .")
print("加速：w      减速: e")
print("退出程序：q/ctrl+c")

# 定义按键映射
key_map = {
    'i': 0x41,
    'o': 0x42,
    'l': 0x43,
    '.': 0x44,
    ',': 0x45,
    'm': 0x46,
    'j': 0x47,
    'u': 0x48,
    'w': 0x58,
    'e': 0x59,
}

def on_press(key):
    try:
        if key.char in key_map:
            sock.send(bytes([key_map[key.char]]))
        else:
            sock.send(bytes([0X5A]))  # 发送0x5A
    except AttributeError:
        pass

def on_release(key):
    sock.send(bytes([0X5A]))  # 发送0x5A
    if key == Key.esc or key.char == 'q':
        print("退出程序")
        return False  # 停止监听器

# 开始监听键盘事件
with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# 最后，关闭socket
sock.close()