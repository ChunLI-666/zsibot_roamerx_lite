#!/usr/bin/env python3
"""SSH stdin relay: hard-wired SIM loopback destination with age/watchdog checks."""
import json,math,select,socket,sys,time

def validated_axes(msg, now_ns):
    if not isinstance(msg,dict):raise ValueError('object required')
    age=(now_ns-int(msg['issued_unix_ns']))*1e-9
    if age<-.05 or age>.20:raise ValueError('stale/future command')
    f,y=float(msg['forward']),float(msg['yaw'])
    if not math.isfinite(f) or not math.isfinite(y):raise ValueError('nonfinite command')
    if not 0<=f<=.25 or abs(y)>.5:raise ValueError('uncalibrated command bound')
    return dict(version=1,action='walk',forward=f,lateral=0.,yaw=y)

def main():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);target=('127.0.0.1',7447)
    zero=dict(version=1,action='walk',forward=0.,lateral=0.,yaw=0.)
    current=zero;last_valid=0.;running=True;accepted=0;rejected=0
    print('SIM_RELAY_READY 127.0.0.1:7447',flush=True)
    try:
        while running:
            readable,_,_=select.select([sys.stdin],[],[],.05)
            if readable:
                line=sys.stdin.readline()
                if not line:break
                try:current=validated_axes(json.loads(line),time.time_ns());last_valid=time.monotonic();accepted+=1
                except (ValueError,TypeError,KeyError,json.JSONDecodeError):current=zero;rejected+=1
            if time.monotonic()-last_valid>.25:current=zero
            sock.sendto(json.dumps(current,separators=(',',':')).encode(),target)
    finally:
        for _ in range(5):sock.sendto(json.dumps(zero).encode(),target);time.sleep(.02)
        print(json.dumps(dict(accepted=accepted,rejected=rejected,stopped_with_zero=True)),flush=True);sock.close()

if __name__=='__main__':main()
