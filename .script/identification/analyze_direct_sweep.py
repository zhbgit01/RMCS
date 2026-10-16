#!/usr/bin/env python3
"""Fit a strict J/B model to a complete direct torque sweep; 0-40 s train, 40-60 s holdout."""
import argparse, csv, json
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv',type=Path)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=True)

def optimize(residual, initial, lo, hi, iterations=80):
    x = np.clip(initial, lo, hi)
    r = residual(x)
    cost = float(r @ r)
    damping = .01
    history = []
    for iteration in range(iterations):
        columns = []
        for i in range(len(x)):
            trial = x.copy()
            trial[i] += 1e-3
            columns.append((residual(trial) - r) / 1e-3)
        jac = np.column_stack(columns)
        H = jac.T @ jac
        accepted = False
        for _ in range(10):
            delta = np.linalg.solve(H + damping * np.diag(np.maximum(np.diag(H), 1e-8)), -jac.T @ r)
            delta = np.clip(delta, -.5, .5)
            trial = np.clip(x + delta, lo, hi)
            rr = residual(trial)
            newcost = float(rr @ rr)
            if newcost < cost:
                change = cost - newcost
                x, r, cost = trial, rr, newcost
                damping = max(damping / 3, 1e-8)
                accepted = True
                break
            damping *= 5
        history.append(cost)
        if iteration % 10 == 0:
            print('iteration', iteration, 'objective', cost, flush=True)
        if not accepted or change < 1e-10:
            break
    return x, history

with args.csv.open() as f:
 r=csv.reader(f);next(r);a=np.array([list(map(float,z)) for z in r])
a=a[np.isfinite(a).all(axis=1)];dt=.005;t=np.arange(0,a[-1,1],dt);u=np.interp(t,a[:,1],a[:,3]);v=np.interp(t,a[:,1],a[:,4]);q=np.interp(t,a[:,1],np.unwrap(a[:,5]));q-=q[0]
def sim(u,J,B,theta,omega):
 n=len(u);r=B/J;z=r*dt;decay=np.exp(-z);aa=-np.expm1(-z)/r;bb=dt-aa;cc=dt*dt/2-bb/r;slope=np.diff(u)/dt
 d=(1-decay)*u[:-1]/B+bb*slope/B
 size=1<<(2*n-1).bit_length();impulse=decay**np.arange(n)
 vel=omega*impulse+np.fft.irfft(np.fft.rfft(np.r_[0,d],size)*np.fft.rfft(impulse,size),size)[:n]
 angle=np.r_[theta,theta+np.cumsum(aa*vel[:-1]+bb*u[:-1]/B+cc*slope/B)]
 return np.column_stack([angle,vel])
m=t<40
res=lambda x:(sim(u[m],*np.exp(x),q[0],v[0])[::4,0]-q[m][::4])
bests=[]
for p in [[.05,1.45],[.1,3],[.02,.5]]:
 x,h=optimize(res,np.log(p),np.log([.001,.01]),np.log([2,30]));bests.append((res(x)@res(x),np.exp(x)))
best=min(bests,key=lambda z:z[0])[1];print('BEST',best)
for label,m in [('train',t<40),('validation',t>=40)]:
 pred=sim(u[m],*best,q[m][0],v[m][0]);e=np.rad2deg(pred-np.column_stack([q[m],v[m]]));print(label,'rms',np.sqrt(np.mean(e*e,axis=0)),'peakangle',max(abs(e[:,0])))

results={}; predictions=[]
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="650"><rect width="1100" height="650" fill="white"/><text x="65" y="28" font-size="18">Torque-to-angle J/B model: measured blue, predicted orange</text>']
for i,(label,mask) in enumerate([('Training 0-40 s',t<40),('Holdout validation 40-60 s',t>=40)]):
    pred=sim(u[mask],*best,q[mask][0],v[mask][0]); actual=np.column_stack([q[mask],v[mask]])
    e=np.rad2deg(pred-actual)
    results[label]={'angle_rmse_deg':float(np.sqrt(np.mean(e[:,0]**2))),'max_angle_error_deg':float(max(abs(e[:,0]))),'velocity_rmse_deg_s':float(np.sqrt(np.mean(e[:,1]**2))),'measured_angle_range_deg':float(np.rad2deg(np.ptp(q[mask]))),'centered_angle_fit_percent':float(100*(1-np.linalg.norm(e[:,0])/np.linalg.norm(np.rad2deg(q[mask]-q[mask].mean()))))}
    predictions.append(np.column_stack([t[mask],np.full(mask.sum(),i),u[mask],actual,pred]))
    top=70+i*280;bottom=top+200;tt=t[mask]
    yy=np.rad2deg(actual[:,0]);pp=np.rad2deg(pred[:,0]);lo=min(yy.min(),pp.min());hi=max(yy.max(),pp.max());span=hi-lo
    svg.append(f'<text x="65" y="{top-14}" font-size="16">{label}; angle RMSE {results[label]["angle_rmse_deg"]:.3f} deg</text><path d="M65 {top} V{bottom} H1040" fill="none" stroke="black"/>')
    for data,color in [(yy,'#1674bd'),(pp,'#df7426')]:
        points=' '.join(f'{65+975*(s-tt[0])/(tt[-1]-tt[0]):.2f},{bottom-200*(z-lo)/span:.2f}' for s,z in zip(tt[::3],data[::3]))
        svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.2"/>')
    svg.append(f'<text x="2" y="{top+10}" font-size="12">{hi:.2f} deg</text><text x="2" y="{bottom}" font-size="12">{lo:.2f}</text><text x="65" y="{bottom+22}">{tt[0]:.1f} s</text><text x="990" y="{bottom+22}">{tt[-1]:.1f} s</text>')
svg.append('</svg>');(args.output/'validation.svg').write_text(''.join(svg))
np.savetxt(args.output/'prediction.csv',np.vstack(predictions),delimiter=',',comments='',header='time_s,partition_0_train_1_validation,feedback_torque_Nm,measured_angle_rad,measured_velocity_rad_s,predicted_angle_rad,predicted_velocity_rad_s')
raw_dt=np.diff(a[:,1])
model={'source':str(args.csv),'status':'preliminary approximate model; independent temporal holdout checked; not a precision mechanical calibration','equation':'J*theta_ddot+B*theta_dot=feedback_torque','J_kg_m2':float(best[0]),'B_Nm_s_rad':float(best[1]),'transfer_denominator':[float(best[0]),float(best[1]),0.0],'training_s':[0,40],'validation_s':[40,60],'method':'continuous forward angle-error minimization; zero time shift; no smoothing; positive J/B; initial measured encoder angle and velocity used only once per partition; linear force interpolation and exact integration at 200 Hz','multistart_solutions':[{'objective':float(c),'J':float(p[0]),'B':float(p[1])} for c,p in bests],'metrics':results,'data_quality':{'finite_rows':len(a),'time_end_finite_s':float(a[-1,1]),'max_dt_s':float(raw_dt.max()),'unwrap_angle':True,'command_feedback_correlation':float(np.corrcoef(a[:,2],a[:,3])[0,1]),'encoder_velocity_peak_deg_s':float(np.rad2deg(max(abs(a[:,4]))))}}
(args.output/'model.json').write_text(json.dumps(model,indent=2)+'\n')
# Numerical implementation check against constant torque analytic solution.
test_t=np.arange(1001)*dt; check=sim(np.full(len(test_t),.1),.02,.5,0.,0.)
expected=.2*test_t-.008*(1-np.exp(-25*test_t))
assert max(abs(check[:,0]-expected))<1e-9
print(json.dumps(model,indent=2))
