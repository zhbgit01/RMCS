"""Run from repository root: python3 yaw-pid-angle8-velocity11/report/compare.py."""
import io,json
from pathlib import Path
import numpy as np
out=Path('yaw-pid-angle8-velocity11/report'); grid=np.arange(0,.501,.005)
allcurves={};results={}
for name,filename in [('Kp13','yaw-pid-baseline/yaw_pid_monitor_2026-10-16_08-05-40.csv'),('Kp11','yaw-pid-kp11/yaw_pid_monitor_2026-10-16_08-25-54.csv'),('Angle8_Vel11','yaw-pid-angle8-velocity11/yaw_pid_monitor_2026-10-16_08-44-07.csv')]:
 lines=Path(filename).read_text().splitlines()
 if len(lines[-1].split(','))!=len(lines[0].split(',')):lines.pop()
 a=np.genfromtxt(io.StringIO('\n'.join(lines)),delimiter=',',names=True,deletechars='');c=lambda k:a['/gimbal/yaw/'+k]
 t=c('pid_test/time');s=c('pid_test/stage');off=np.rad2deg(c('pid_test/offset'));inds=np.flatnonzero(s==1);starts=inds[np.r_[True,abs(np.diff(off[inds]))>1]]
 curves={k:[] for k in ['velocity_imu','control_torque','control_angle_error']};reverse=[];wide_reverse=[]
 for i in starts:
  sign=np.sign(off[i]-off[i-1]);m=(t>=t[i])&(t<t[i]+.6);tt=t[m]-t[i]
  for key in curves:
   scale=180/np.pi if key!='control_torque' else 1
   curves[key].append(np.interp(grid,tt,c(key)[m])*scale*sign)
  # 25ms moving mean; reverse speed magnitude during 100-300ms after each step.
  v=np.convolve(curves['velocity_imu'][-1],np.ones(5)/5,mode='valid');vt=grid[2:-2]
  reverse.append(float(max(0,-v[(vt>=.1)&(vt<=.3)].min())))
  wide_reverse.append(float(max(0,-v[(vt>=.1)&(vt<=.48)].min())))
 allcurves[name]={k:np.array(v) for k,v in curves.items()}
 results[name]={'steps':len(starts),'reverse_speed_25ms_mean_100_to_300ms_deg_s':reverse,'mean_reverse_speed_deg_s':float(np.mean(reverse)),'worst_reverse_speed_deg_s':max(reverse),'mean_reverse_speed_100_to_480ms_deg_s':float(np.mean(wide_reverse))}
(out/'rebound_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="900"><rect width="1100" height="900" fill="white"/><g font-family="sans-serif" font-size="13"><text x="70" y="25" font-size="18">12 steps aligned, direction normalized: A10/V13 blue, A10/V11 orange, A8/V11 green</text>']
for j,(key,label) in enumerate([('velocity_imu','IMU velocity (deg/s)'),('control_torque','Command torque (Nm)'),('control_angle_error','Angle error (deg)')]):
 top=65+j*270;bottom=top+205;lo=min(0,*[v[key].mean(axis=0).min() for v in allcurves.values()]);hi=max(v[key].mean(axis=0).max() for v in allcurves.values());span=hi-lo;lo-=span*.08;hi+=span*.08
 svg.append(f'<text x="70" y="{top-15}">{label}, average across 12 steps</text>')
 for tick in np.linspace(lo,hi,5):
  y=bottom-205*(tick-lo)/(hi-lo);svg.append(f'<path d="M70 {y} H1060" stroke="#ddd"/><text x="5" y="{y+4}">{tick:.2f}</text>')
 for t in [0,.1,.2,.3,.4,.5]:
  x=70+990*t/.5;svg.append(f'<path d="M{x} {top} V{bottom}" stroke="#ddd"/><text x="{x}" y="{bottom+20}">{t:.1f}s</text>')
 for name,color in [('Kp13','#2474b5'),('Kp11','#e07020'),('Angle8_Vel11','#238c48')]:
  v=allcurves[name][key].mean(axis=0);points=' '.join(f'{70+990*t/.5:.2f},{bottom-205*(z-lo)/(hi-lo):.2f}' for t,z in zip(grid,v));svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
svg.append('</g></svg>');(out/'comparison.svg').write_text(''.join(svg))
print(json.dumps(results,indent=2))
