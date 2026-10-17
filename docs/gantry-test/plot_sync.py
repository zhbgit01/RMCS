#!/usr/bin/env python3
"""Render the logged gantry ascent excerpts to standalone SVG/HTML and CSV.
Usage: python3 plot_sync.py /path/to/log.txt [output_directory]
Uses only Python's standard library; no hardware access.
"""
import csv
import html
import json
import re
import sys
from pathlib import Path

source = Path(sys.argv[1])
outdir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).parent
outdir.mkdir(parents=True, exist_ok=True)
rows = []
pending = None
for line in source.read_text().splitlines():
    timestamp = re.search(r'\[INFO\] \[([\d.]+)\]', line)
    if not timestamp:
        continue
    if '[gantry] ' in line:
        match = re.search(r'active=(\d).*fault=(\d).*stick\(up,side\)=\(([^)]+)\).*torque\(yaw,left,right\)=\(([^)]+)\).*skew=([-\d.]+)', line)
        if match:
            active, fault, stick, torque, skew = match.groups()
            pending = dict(timestamp=float(timestamp[1]), active=int(active), fault=int(fault),
                           stick=list(map(float, stick.split(','))),
                           torque=list(map(float, torque.split(','))), skew=float(skew))
    elif '[gantry motion yaw,left,right]' in line and pending:
        if abs(float(timestamp[1]) - pending['timestamp']) > 0.1:
            pending = None
            continue
        fields = {}
        for name in ('target_rad', 'position_rad', 'velocity_target', 'velocity_actual'):
            match = re.search(name + r'=\(([^)]+)\)', line)
            if match:
                fields[name] = list(map(float, match[1].split(',')))
        if len(fields) == 4:
            rows.append(dict(pending, **fields))
        pending = None

windows = [(1792222659, 1792222668), (1792222687, 1792222695)]
segments = [[r for r in rows if lo <= r['timestamp'] < hi] for lo, hi in windows]
assert [len(s) for s in segments] == [9, 8], 'Unexpected/missing samples: inspect source log'
summary = []
with (outdir / 'sync_samples.csv').open('w') as stream:
    writer = csv.writer(stream)
    writer.writerow(['segment', 'timestamp', 'elapsed_s', 'pitch_stick', 'active', 'fault',
                     'left_position_rad', 'right_position_rad', 'left_displacement_rad', 'right_displacement_rad',
                     'left_target_rad', 'right_target_rad', 'left_velocity_rad_s', 'right_velocity_rad_s',
                     'left_command_Nm', 'right_command_Nm', 'logged_skew_rad'])
    for index, segment in enumerate(segments, 1):
        first = segment[0]
        for r in segment:
            writer.writerow([index, r['timestamp'], r['timestamp']-first['timestamp'], r['stick'][0], r['active'], r['fault'],
                             *r['position_rad'][1:],
                             r['position_rad'][1]-first['position_rad'][1], r['position_rad'][2]-first['position_rad'][2],
                             *r['target_rad'][1:], *r['velocity_actual'][1:], *r['torque'][1:], r['skew']])
        summary.append(dict(segment=index, samples=len(segment), start_timestamp=first['timestamp'],
                            end_timestamp=segment[-1]['timestamp'],
                            max_abs_logged_skew_rad=max(abs(r['skew']) for r in segment),
                            mean_abs_logged_skew_rad=sum(abs(r['skew']) for r in segment)/len(segment),
                            fault_samples=sum(r['fault'] for r in segment),
                            displacement_reference='Each side subtracts its own first measured position in this excerpt.'))
(outdir / 'summary.json').write_text(json.dumps(summary, indent=2))

# SVG charts: shared scales, visible measured sample markers, no interpolation/smoothing.
W, H = 1300, 1080
parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         '<rect width="100%" height="100%" fill="#f5f7fb"/>',
         '<style>text{font-family:DejaVu Sans,Arial,sans-serif;fill:#233047} .small{font-size:12px} .tick{font-size:11px;fill:#526176}</style>']
def text(x,y,value,size=14,fill=None,**attrs):
    extra=' '.join(f'{k.replace("_","-")}="{v}"' for k,v in attrs.items())
    parts.append(f'<text x="{x}" y="{y}" font-size="{size}" {extra}'+(f' style="fill:{fill}"' if fill else '')+'>'+html.escape(str(value))+'</text>')
def line(x1,y1,x2,y2,color,width=1,dash=''):
    parts.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>')
text(40,42,'Gantry ascent under unilateral disturbance',27,font_weight='bold')
text(40,68,'Operator reports: right-side resistance first, left-side resistance later. Exact contact times were not logged.',14)
text(40,91,'Encoder evidence only: relative screw motion stays close; physical tilt was not measured.',14)
left_color, right_color = '#1269b0','#e17920'
line(43,119,70,119,left_color,3);text(78,124,'Left / ID 2',13)
line(209,119,236,119,right_color,3,'6 3');text(244,124,'Right / ID 3',13)
text(450,124,'Markers = logged samples (~1 Hz); straight lines connect samples.',13)

for col, segment in enumerate(segments):
    ox = 40 + col*640
    parts.append(f'<rect x="{ox}" y="145" width="610" height="835" rx="12" fill="white" stroke="#dce3ed"/>')
    title = ['A | Earlier ascent - right-side resistance reported','B | Later ascent - left-side resistance reported'][col]
    text(ox+16,174,title,16,font_weight='bold')
    text(ox+16,196,f"Unix timestamps {windows[col][0]}–{int(segment[-1]['timestamp'])}",12)
    maxerr=summary[col]['max_abs_logged_skew_rad']
    text(ox+16,219,f"Max sampled |skew|: {maxerr:.3f} rad   |   Fault samples: {summary[col]['fault_samples']}",14)
    elapsed=[r['timestamp']-segment[0]['timestamp'] for r in segment]
    xmax=9
    def chart(top,label,series,ylim,ticks):
        x0,y0,pw,ph=ox+68,top+27,516,161
        text(ox+16,top+8,label,14,font_weight='bold')
        def xy(t,v): return x0+t/xmax*pw,y0+ph-(v-ylim[0])/(ylim[1]-ylim[0])*ph
        for v in ticks:
            _,y=xy(0,v);line(x0,y,x0+pw,y,'#e7ecf3');text(x0-9,y+4,f'{v:g}',11,text_anchor='end')
        for t in range(0,10):
            x,_=xy(t,0);line(x,y0,x,y0+ph,'#edf1f6');text(x,y0+ph+17,t,11,text_anchor='middle')
        text(x0+pw/2,y0+ph+35,'Seconds since excerpt start',11,text_anchor='middle')
        for values,color,dash in series:
            coords=[xy(t,v) for t,v in zip(elapsed,values)]
            pts=' '.join(f'{x:.2f},{y:.2f}' for x,y in coords)
            parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.2" stroke-dasharray="{dash}"/>')
            for t,v,(x,y) in zip(elapsed,values,coords):
                parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3" fill="{color}"><title>t={t:.3f}s, value={v:.4f}</title></circle>')
        return xy
    displacement=[[r['position_rad'][i]-segment[0]['position_rad'][i] for r in segment] for i in (1,2)]
    chart(247,'1. Measured displacement from excerpt start (rad)',
          [(displacement[0],left_color,''),(displacement[1],right_color,'6 3')],(-2,80),[0,20,40,60,80])
    text(ox+68,481,'Each side uses its own starting position; overlap shows matched travel.',11)
    chart(501,'2. Controller torque commands (N m)',
          [([r['torque'][1] for r in segment],left_color,''),([r['torque'][2] for r in segment],right_color,'6 3')],(-0.45,0.45),[-0.4,-0.2,0,0.2,0.4])
    # An observed command imbalance is not an independently identified contact time.
    r = segment[6] if col == 0 else segment[-2]
    text(ox+68,735,f"Example t={r['timestamp']-segment[0]['timestamp']:.1f}s: left {r['torque'][1]:+.3f}, right {r['torque'][2]:+.3f} N m",11)
    chart(755,'3. Logged relative displacement difference: left - right (rad)',
          [([r['skew'] for r in segment],'#7750a3','')],(-0.015,0.015),[-0.01,0,0.01])
text(40,1010,'Interpretation: asymmetric commands accompany closely matched encoder travel in both ascent excerpts.',14,font_weight='bold')
text(40,1035,'Limits: rounded 1 Hz logs may miss peaks; screw lead, initial alignment and transmission slip are not measured.',13)
text(40,1057,'The figure supports synchronization under the reported disturbance; it does not prove zero physical tilt.',13)
parts.append('</svg>')
svg='\n'.join(parts)
(outdir / 'gantry_sync.svg').write_text(svg)
(outdir / 'gantry_sync.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>龙门架单侧受扰动同步测试</title><style>body{font-family:system-ui;margin:24px auto;max-width:1350px;line-height:1.7;color:#233047}svg{width:100%;height:auto}p{margin:12px 16px}</style><h1>龙门架单侧受扰动同步测试</h1><p>操作者确认先右侧施压、后左侧施压。图中采用两段完整上升片段，未把施压起止时间当作已测量数据。每列依次为左右位移、控制力矩、日志同步误差。</p>'''+svg+'''<p>位移图分别减去每侧在片段开始时的位置，显示运动增量；不能用于证明初始齐平。同步误差使用原日志 skew，不用位移图重新归零后的差值代替。力矩为控制命令，不是实测负载或实际力矩。</p><p>结论：结合操作者确认的扰动测试，两段日志显示左右行程接近一致，支持编码器层面同步有效。实际镖架是否无明显歪斜还需实物观察；约 1 Hz、三位小数的日志无法排除采样间峰值，也不能证明是同步修正单独产生的效果。</p><p><a href="sync_samples.csv">原始采样表 CSV</a> · <a href="summary.json">统计 JSON</a> · <a href="gantry_sync.svg">矢量图 SVG</a></p></html>''')
print(json.dumps(summary, indent=2))
