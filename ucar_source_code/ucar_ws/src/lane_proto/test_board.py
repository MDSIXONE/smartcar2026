import numpy as np, math, sys
sys.path.insert(0,'/home/claude/restore/scripts')
from lane_common import scan_xy, board_detect
N, AMIN, AINC = 720, -math.pi, 2*math.pi/720
def ray(segs, ang):
    best=16.0; dx,dy=math.cos(ang),math.sin(ang)
    for (x1,y1,x2,y2) in segs:
        ex,ey=x2-x1,y2-y1; den=dx*ey-dy*ex
        if abs(den)<1e-9: continue
        t=(x1*ey-y1*ex)/den; u=(x1*dy-y1*dx)/den
        if t>0.05 and 0.0<=u<=1.0: best=min(best,t)
    return best
def run(name, segs, expect, **kw):
    np.random.seed(1)
    a=AMIN+AINC*np.arange(N)
    r=np.array([ray(segs,t) for t in a])+np.random.normal(0,0.004,N)
    xs,ys=scan_xy(r,AMIN,AINC,0.08,16.0,lidar_x=0.0)
    hit,info=board_detect(xs,ys,**kw)
    ok = hit==expect
    print("%s%-32s -> %-4s d=%5.2f w=%5.2f n=%-4d 簇=%-3d %s"
          % ("OK " if ok else "!! ", name, "板子" if hit else "放行",
             info["d"], info["w"], info["n"], info["nclus"], info["why"]))
    return ok
FAR=(3.0,-2.5,3.0,2.5)
T=[]
T.append(run("拦路板 0.5m 正前方", [(0.5,-0.21,0.5,0.21),FAR], True))
T.append(run("围挡 0.5m 铺满", [(0.5,-2.5,0.5,2.5)], False))
T.append(run("围挡 1.2m 铺满", [(1.2,-2.5,1.2,2.5)], False))
for deg in (10,15,20,30,45):
    th=math.radians(deg)
    T.append(run("围挡斜 %d° (窄纵深陷阱)"%deg,
        [(0.6-2.5*math.sin(th),-2.5*math.cos(th),0.6+2.5*math.sin(th),2.5*math.cos(th))], False))
for deg in (0,15,25,35):
    th=math.radians(deg); L=0.21
    T.append(run("板子斜 %d° 看"%deg,
        [(0.55-L*math.sin(th),-L*math.cos(th),0.55+L*math.sin(th),L*math.cos(th)),FAR], True))
T.append(run("红绿灯箱体 20cm", [(0.6,-0.10,0.6,0.10),FAR], False))
T.append(run("板子后 25cm 是围挡", [(0.5,-0.21,0.5,0.21),(0.75,-2.5,0.75,2.5)], True))
T.append(run("板子 + 左 0.8m 平行围挡",
    [(0.5,-0.21,0.5,0.21),(-1,0.8,3,0.8),FAR], True))
T.append(run("空场地(围挡 2m 外)",
    [(2.2,-2.5,2.2,2.5),(-1,1.5,3,1.5),(-1,-1.5,3,-1.5)], False))
T.append(run("走廊(两侧 0.6m 平行围挡, 无板)",
    [(-1,0.6,3,0.6),(-1,-0.6,3,-0.6),FAR], False))
T.append(run("场地拐角(两面围挡成直角)",
    [(1.0,-2.5,1.0,0.7),(1.0,0.7,3.0,0.7)], False))
print("\n%d/%d 通过" % (sum(T), len(T)))
