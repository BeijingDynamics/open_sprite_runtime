"""Guided dual-IMU recording. No CAN, motor control or sensor configuration writes."""
from dataclasses import asdict
import argparse
import gc
from datetime import datetime
import json
from pathlib import Path
import threading
import time

from open_sprite_runtime.bno055 import BNO055, I2CRegisters
from open_sprite_runtime.yahboom_imu import YahboomStreamDecoder

STAGES = [(10, 'STILL_START', 'Keep completely still'),
          (10, 'TURN_LEFT', 'Slowly turn LEFT about robot vertical Z to the 90-degree marker'),
          (10, 'HOLD_LEFT', 'Hold still at the left marker'),
          (10, 'RETURN', 'Slowly return to the EXACT starting marker'),
          (15, 'STILL_END', 'Keep completely still at the starting marker')]


def main():
    import serial
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stationary-seconds',type=int,choices=range(10,121))
    parser.add_argument('--start-now',action='store_true')
    parser.set_defaults(defer_gc=True)
    parser.add_argument('--defer-gc',dest='defer_gc',action='store_true',help='Default: defer cyclic GC for the bounded capture')
    parser.add_argument('--allow-cyclic-gc',dest='defer_gc',action='store_false',help='Diagnostic only: reproduce automatic GC pauses')
    args=parser.parse_args()
    stages=STAGES if args.stationary_seconds is None else [(args.stationary_seconds,'STATIONARY','Keep completely still; do not touch the robot')]
    print('SENSOR ONLY: no CAN, no motor enable, no policy, no configuration writes.')
    print('Both IMUs must be rigidly attached to the SAME body.')
    if args.stationary_seconds is None:
        print('Mark start and left90 directions. Use ROBOT vertical Z, not sensor Z. Keep cables free.')
    if not args.start_now:
        input('Press ENTER when ready; remain still and follow stage instructions: ')
    folder=Path(__file__).resolve().parents[1]/'reports'/datetime.now().strftime('dual_imu_guided_%Y%m%d_%H%M%S')
    folder.mkdir(parents=True,exist_ok=False)
    stop=threading.Event()
    ready={n:threading.Event() for n in ('bno055','yahboom')}
    last={n:0 for n in ready}
    errors=[]; samples=[]; batches=[]; events=[]
    configurations=[]
    gc_events=[]
    gc_enabled=gc.isenabled()
    def gc_trace(phase,info):
        gc_events.append((time.monotonic_ns(),phase,info['generation']))
    gc.collect()
    gc.callbacks.append(gc_trace)
    if args.defer_gc:
        gc.disable()

    def bno():
        io=None
        try:
            io=I2CRegisters(7,0x28); sensor=BNO055(io)
            def check():
                cfg=sensor.inspect(); configurations.append(cfg)
                if not cfg['matches'] or cfg['system_error'] or cfg['system_status']!=5:
                    raise RuntimeError('BNO055 not configured for IMUPLUS/P0; run the configuration script first')
            check(); deadline=time.monotonic(); next_check=deadline+1
            while not stop.is_set():
                s=sensor.read_sample(); row=asdict(s); row['body']=s.body_state('p0-output-axes')
                samples.append(row); last['bno055']=s.read_end_ns; ready['bno055'].set()
                if time.monotonic()>=next_check:
                    check(); next_check=time.monotonic()+1
                deadline+=.01
                if deadline<time.monotonic(): deadline=time.monotonic()
                stop.wait(max(0,deadline-time.monotonic()))
        except Exception as exc:
            errors.append('bno055: '+repr(exc)); stop.set()
        finally:
            if io is not None: io.close()

    def yahboom():
        try:
            decoder=YahboomStreamDecoder(); seen=set()
            port=serial.Serial(port=None,baudrate=115200,timeout=.02,exclusive=True)
            port.dtr=False;port.rts=False;port.port='/dev/sprite0825-imu'
            with port:
                while not stop.is_set():
                    begin=time.monotonic_ns()
                    payload=port.read(min(max(port.in_waiting,1),4096))
                    end=time.monotonic_ns()
                    packets=[dict(type=type(p).__name__,values=asdict(p)) for p in decoder.feed(payload)]
                    if payload: batches.append(dict(read_start_ns=begin,read_end_ns=end,hex=payload.hex(),packets=packets))
                    for packet in packets:
                        seen.add(packet['type'])
                    if packets: last['yahboom']=end
                    if {'YahboomRawImu','YahboomQuaternion'}<=seen: ready['yahboom'].set()
        except Exception as exc:
            errors.append('yahboom: '+repr(exc));stop.set()

    threads=[threading.Thread(target=f,daemon=True) for f in (bno,yahboom)]
    try:
        for thread in threads: thread.start()
        timeout=time.monotonic()+5
        while not all(e.is_set() for e in ready.values()):
            if stop.is_set(): raise RuntimeError('Sensor startup failed')
            if time.monotonic()>timeout: raise RuntimeError('Both IMUs did not become ready within5s')
            time.sleep(.02)
        print('BOTH IMUs READY. Follow the countdown below.',flush=True)
        for duration,name,instruction in stages:
            start=time.monotonic_ns()
            events.append(dict(stage=name,start_ns=start,duration_s=duration))
            print('\n\a'+name+': '+instruction,flush=True)
            for remaining in range(duration,0,-1):
                print(f'{name}: {remaining:2d}s remaining',flush=True)
                deadline=start/1e9+(duration-remaining+1)
                while time.monotonic()<deadline:
                    if stop.wait(.02): raise RuntimeError('Acquisition failed')
                    if any(time.monotonic_ns()-v>500_000_000 for v in last.values()):
                        raise RuntimeError('Sensor stream missing for more than500ms; stop moving')
            events[-1]['end_ns']=time.monotonic_ns()
        print('\n\aDONE. You may release the robot; acquisition is stopping.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        errors.append(repr(exc));print('\nSTOP: '+repr(exc),flush=True)
    finally:
        stop.set()
        for thread in threads:
            if thread.ident is not None: thread.join(3)
        if any(thread.is_alive() for thread in threads): errors.append('Reader shutdown timeout')
        gc.callbacks.remove(gc_trace)
        if gc_enabled:
            gc.enable()
        (folder/'bno055.json').write_text(json.dumps(dict(samples=samples,errors=errors,configurations=configurations)))
        (folder/'yahboom.json').write_text(json.dumps(dict(batches=batches,errors=errors)))
        (folder/'manifest.json').write_text(json.dumps(dict(events=events,errors=errors,
            can_tx_count=0,configuration_writes=0,gc_events=gc_events,gc_deferred=args.defer_gc,
            clock='host monotonic, not sensor sample time'),indent=2))
        print('REPORT DIRECTORY:',folder,flush=True)
    raise SystemExit(bool(errors))


if __name__=='__main__': main()
