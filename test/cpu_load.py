#!/usr/bin/env python3
"""CPU 负载发生器: N 个纯自旋进程。用法: cpu_load.py start N | stop"""
import subprocess, sys, os, time, signal

PIDFILE = '/tmp/cpu_load.pids'

def start(n):
    pids = []
    for _ in range(n):
        p = subprocess.Popen([sys.executable, '-c', 'while True: pass'])
        pids.append(p.pid)
    open(PIDFILE, 'w').write('\n'.join(map(str, pids)))
    print(f'started {n} spinners: {pids[:3]}...')

def stop():
    if not os.path.exists(PIDFILE):
        print('no load running')
        return
    pids = [int(x) for x in open(PIDFILE).read().split()]
    for pid in pids:
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
    os.remove(PIDFILE)
    print(f'stopped {len(pids)}')

if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'start':
        start(int(sys.argv[2]))
    elif cmd == 'stop':
        stop()
