"""Operator-guided button mapping and native combo check. NO CAN, no policy."""
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time

C=Path('/home/tony/open_sprite_runtime/candidates/xbox_native_120_20261019')
device='/dev/input/js0'
def read_buttons(label):
    input(f'{label}: 松开所有按键，按回车后，只按下并松开{label}。采集6秒。')
    fd=os.open(device,os.O_RDONLY|os.O_NONBLOCK)
    events=[];pressed=set();end=time.monotonic()+6
    try:
        while time.monotonic()<end:
            if not select.select([fd],[],[],.05)[0]:continue
            b=os.read(fd,8)
            if len(b)!=8:raise RuntimeError('joystick disconnected')
            _,value,kind,number=struct.unpack('<IhBB',b)
            if kind&1 and not kind&128:
                events.append([number,value]);print(label,number,value,flush=True)
                if value:pressed.add(number)
    finally:os.close(fd)
    if len(pressed)!=1:raise RuntimeError(f'{label}: expected exactly one button; saw {pressed}')
    button=pressed.pop()
    if [button,0] not in events:raise RuntimeError('release not observed')
    return button,events

if __name__=='__main__':
    print('只读取手柄，无CAN、无使能、无policy。',flush=True)
    lb,le=read_buttons('LB');rb,re=read_buttons('RB')
    if lb==rb:raise RuntimeError('LB and RB must differ')
    input('下一步松开LB/RB，回车后再同时按住两键至少3秒。')
    env=os.environ.copy();env.update(SPRITE_GAMEPAD_LB=str(lb),SPRITE_GAMEPAD_RB=str(rb))
    subprocess.run([str(C/'gamepad_disable_probe'),device],env=env,check=True)
    result={'passed':True,'device':device,'lb':lb,'rb':rb,'lb_events':le,'rb_events':re,'native_combo_passed':True,'hardware_tx':0}
    (C/'buttons_verified.json').write_text(json.dumps(result,indent=2)+'\n')
    subprocess.run([sys.executable,str(C/'check_powered_seal.py'),'--seal'],check=True)
    print('按键确认完成；尚未使能任何电机。')
