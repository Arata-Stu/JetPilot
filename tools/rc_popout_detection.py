"""Offline, causal activity baselines for annotated RC popout recordings."""
from __future__ import annotations
import argparse
import csv
import hashlib
import html
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evaluation_intervals(annotation):
    spans = sorted((float(x['start_s']), float(x['end_s']))
                   for x in annotation['intervals'] if x['label'] == 'evaluation')
    exclusions = [(float(x['start_s']), float(x['end_s']))
                  for x in annotation['intervals'] if x['label'] == 'exclude']
    if not spans or any(not (math.isfinite(a) and math.isfinite(b) and 0 <= a < b) for a,b in spans+exclusions):
        raise ValueError('invalid or missing evaluation intervals')
    if any(b > c for (a,b),(c,d) in zip(spans,spans[1:])):
        raise ValueError('overlapping evaluation intervals')
    for x,y in exclusions:
        spans = [(c,d) for a,b in spans for c,d in ((a,min(b,x)),(max(a,y),b)) if c<d]
    if not spans:
        raise ValueError('no evaluation time after exclusions')
    return spans


def crossings(rows, threshold):
    """One trigger per contiguous above-threshold episode, reset per interval."""
    result=[]; active=False; last=None
    for interval,t,value in rows:
        if interval != last:
            active=False
        high=value >= threshold
        if high and not active:
            result.append(t)
        active=high; last=interval
    return result


def trailing_counts(counts, window_bins, np):
    cumulative=np.concatenate(([0], np.cumsum(counts)))
    ends=np.arange(window_bins, len(counts)+1)
    return cumulative[ends]-cumulative[ends-window_bins]


def analyze(config, entry, args):
    import numpy as np
    import cv2
    from multi_sensor_calibration.io import load_yaml
    from multi_sensor_calibration.calibration_overlay import _camera
    from multi_sensor_calibration.scenario_overlay import _projection_homography, _rgb_frames, _selected_rgb_times
    from multi_sensor_calibration.evs_sources import MetavisionFileSource
    from multi_sensor_calibration.models import ClockEstimate

    ann_path=Path(entry['annotation']); ann=json.loads(ann_path.read_text())
    if ann['session'] != entry['session']:
        raise ValueError('session mismatch')
    spans=evaluation_intervals(ann)
    spatial=config['spatial']; roi=config['roi']; chain=Path(spatial['camchain'])
    if spatial['view_frame']!='evs' or digest(chain)!=spatial['camchain_sha256']:
        raise ValueError('EVS coordinates and matching calibration hash required')
    sync=Path(ann['time_sync'])
    if digest(sync)!=ann['time_sync_sha256']:
        raise ValueError('time sync changed since annotation')
    calibration=load_yaml(chain)
    evs=_camera(calibration,'cam0',np); rgb=_camera(calibration,'cam1',np)
    size=tuple(spatial['output_size']); w,h=size
    if size!=tuple(evs['size']): raise ValueError('EVS size mismatch')
    transform=np.asarray(calibration['cam1']['T_cn_cnm1'],dtype=float)
    H=np.linalg.inv(_projection_homography(evs,rgb,transform,spatial['projection'],spatial['depth_m'],np))
    if not np.allclose(H,spatial['rgb_to_view_homography'],rtol=1e-7,atol=1e-7):
        raise ValueError('projection matrix mismatch')
    maps=[]; supports=[]
    for camera,warp in ((rgb,H),(evs,np.eye(3))):
        mapping=cv2.initUndistortRectifyMap(camera['matrix'],camera['distortion'],None,camera['matrix'],camera['size'],cv2.CV_32FC1)
        maps.append(mapping)
        support=cv2.remap(np.ones((camera['size'][1],camera['size'][0]),np.float32),*mapping,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        supports.append(cv2.warpPerspective(support,warp,size,flags=cv2.INTER_LINEAR)>=.999)
    common=supports[0]&supports[1]
    coords=[roi[k] for k in ('x','y','width','height')]
    if any(isinstance(v,bool) or not isinstance(v,int) for v in coords): raise ValueError('integer ROI required')
    x,y,rw,rh=coords
    if not (0<=x<x+rw<=w and 0<=y<y+rh<=h): raise ValueError('ROI out of bounds')
    mask=np.zeros((h,w),bool); mask[y:y+rh,x:x+rw]=True
    if roi.get('mask_policy')!='intersect_common': raise ValueError('intersect_common policy required')
    mask &= common
    if not mask.any(): raise ValueError('empty common ROI')
    session=ann_path.parents[3]/ann['session']
    raw=list(session.glob('*.raw'))
    if len(raw)!=1: raise ValueError('exactly one RAW required')
    source_kind=ann.get('rgb_timestamp_source','bag')
    origin,indices,times=_selected_rgb_times(session,args.rgb_topic,source_kind,start_s=0,duration_s=None,every_n=1,max_frames=None)
    if abs(origin-ann['reference_origin_s'])>1e-6: raise ValueError('RGB time origin changed')
    if any(b>times[-1]-origin+1e-6 for a,b in spans): raise ValueError('interval beyond RGB coverage')
    rgb_rows=[]; previous=None; previous_interval=None; previous_t=None
    for timestamp,image in _rgb_frames(session,args.rgb_topic,source_kind,indices):
        t=timestamp-origin
        if t>=spans[-1][1]: break
        interval=next((i for i,(a,b) in enumerate(spans) if a<=t<b),None)
        if interval is None: previous=None;previous_interval=None;continue
        if image.shape[:2]!=(rgb['size'][1],rgb['size'][0]): raise ValueError('RGB dimensions mismatch')
        corrected=cv2.remap(image,*maps[0],cv2.INTER_LINEAR)
        corrected=cv2.warpPerspective(corrected,H,size)
        gray=cv2.cvtColor(corrected,cv2.COLOR_BGR2GRAY).astype(np.int16)
        if previous is not None and interval==previous_interval:
            if t<=previous_t: raise ValueError('RGB timestamps not increasing')
            score=float(np.mean(np.abs(gray[mask]-previous[mask])>=args.rgb_pixel_delta))
            rgb_rows.append((interval,t,score))
        previous=gray;previous_interval=interval;previous_t=t
    # Classify each raw event in undistorted EVS coordinates; keep multiplicity.
    yy,xx=np.indices((h,w)); points=np.stack((xx,yy),axis=-1).astype(np.float32).reshape(-1,1,2)
    undistorted=cv2.undistortPoints(points,evs['matrix'],evs['distortion'],P=evs['matrix']).reshape(h,w,2)
    ux=np.rint(undistorted[:,:,0]).astype(int); uy=np.rint(undistorted[:,:,1]).astype(int)
    inside=(ux>=0)&(ux<w)&(uy>=0)&(uy<h)
    lut=np.zeros((h,w),bool);lut[inside]=mask[uy[inside],ux[inside]]
    clock=ClockEstimate.from_dict(load_yaml(sync)['models']['evs'])
    if not math.isfinite(clock.drift) or 1+clock.drift<=0: raise ValueError('invalid clock scale')
    source=MetavisionFileSource(raw[0])
    if source.anchor is None: raise ValueError('RAW metadata missing')
    step=args.step_ms/1000; bins=args.window_bins
    hist=[np.zeros(int(math.floor((b-a)/step)),dtype=np.int64) for a,b in spans]
    first_event=None;last_event=None
    for batch in source.batches():
        if (source.width,source.height)!=(w,h): raise ValueError('RAW dimensions mismatch')
        events=batch.events
        provisional = source.anchor.reference_time_s + source.anchor.scale * (events['t'].astype(np.float64)-source.anchor.source_time_us)/1e6
        ts=clock.apply(provisional)-origin
        if len(ts)==0: continue
        if np.any(np.diff(ts)<0) or (last_event is not None and ts[0]<last_event): raise ValueError('nonmonotonic events')
        if first_event is None:first_event=float(ts[0])
        last_event=float(ts[-1])
        valid=(events['x']<w)&(events['y']<h)
        kept=np.zeros(len(events),bool);kept[valid]=lut[events['y'][valid],events['x'][valid]]
        for i,(a,b) in enumerate(spans):
            selected=ts[kept & (ts>=a) & (ts<b)]
            index=np.floor((selected-a)/step).astype(int)
            index=index[(index>=0)&(index<len(hist[i]))]
            hist[i]+=np.bincount(index,minlength=len(hist[i]))
        if ts[-1]>=spans[-1][1]:break
    if first_event is None or first_event>spans[0][0] or last_event<spans[-1][1]:
        raise ValueError('RAW event extent does not cover evaluation intervals; cannot treat missing data as silence')
    evs_rows=[]
    for i,((a,b),counts) in enumerate(zip(spans,hist)):
        for j,count in enumerate(trailing_counts(counts,bins,np),start=bins):
            t=a+j*step
            if t<b: evs_rows.append((i,t,int(count)))
    if not rgb_rows or not evs_rows: raise ValueError('insufficient samples')
    onset=ann.get('onset',{}).get('first_visible')
    onset_s=None if onset is None else onset['rgb_time_s']-origin
    result=dict(session=ann['session'],annotation=str(ann_path),annotation_sha256=digest(ann_path),
                roi=config['roi'],valid_pixels=int(mask.sum()),evaluation_seconds=sum(b-a for a,b in spans),
                intervals=spans,rgb_first_visible_s=onset_s,time_sync_sha256=digest(sync),
                camchain_sha256=digest(chain),annotation_kind='no_rgb_onset' if onset_s is None else 'rgb_onset',
                note='Offline activity threshold crossings, not vehicle classification or end-to-end latency.')
    for sensor,rows,threshold in [('rgb',rgb_rows,args.rgb_threshold),('evs',evs_rows,args.evs_threshold)]:
        triggers=crossings(rows,threshold)
        result[sensor]=dict(threshold=threshold,triggers_s=triggers,episodes=len(triggers),
            episodes_per_minute=len(triggers)*60/result['evaluation_seconds'],
            first_trigger_s=triggers[0] if triggers else None,
            first_trigger_minus_rgb_onset_ms=(triggers[0]-onset_s)*1000 if triggers and onset_s is not None else None)
    return result,rgb_rows,evs_rows


def write_plot(path, result, rgb, evs):
    """Min/max envelopes retain narrow peaks when reducing samples for display."""
    begin=result['intervals'][0][0]; end=result['intervals'][-1][1]
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="360" viewBox="0 0 1000 360"><rect width="1000" height="360" fill="white"/>']
    for panel,(sensor,rows) in enumerate((('rgb',rgb),('evs',evs))):
        threshold=result[sensor]['threshold']; top=25+panel*160
        maximum=max(threshold,max(v for _,t,v in rows))*1.1
        def x(t): return 60+900*(t-begin)/(end-begin)
        def y(v): return top+110-100*v/maximum
        parts.append(f'<text x="10" y="{top}" font-size="14">{sensor.upper()} score; threshold={threshold:g}</text>')
        buckets={}
        for interval,t,value in rows:
            key=(interval,int(x(t)))
            lo,hi=buckets.get(key,(value,value));buckets[key]=(min(lo,value),max(hi,value))
        for (_,px),(lo,hi) in buckets.items():
            parts.append(f'<path d="M{px},{y(lo):.2f} L{px},{y(hi):.2f}" stroke="#176da6" stroke-width="1.5"/>')
        parts.append(f'<path d="M60,{y(threshold):.2f} H960" stroke="#c44" stroke-dasharray="5 3"/>')
        onset=result['rgb_first_visible_s']
        if onset is not None:
            parts.append(f'<path d="M{x(onset):.2f},{top} v110" stroke="#6a3"/>')
        for a,b in result['intervals']:
            parts.append(f'<path d="M{x(a):.2f},{top+120} H{x(b):.2f}" stroke="black"/>')
    parts.append(f'<text x="60" y="350">Source time: {begin:.3f} to {end:.3f} s; green = first visible RGB</text></svg>')
    path.write_text(''.join(parts),encoding='utf-8')


def write_report(output, results):
    with (output/'summary.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['session','error','evaluation_seconds','rgb_first_visible_s','rgb_first_trigger_s','evs_first_trigger_s','rgb_episodes','evs_episodes'])
        for r in results:
            writer.writerow([r['session'],r.get('error',''),r.get('evaluation_seconds'),r.get('rgb_first_visible_s'),
                r.get('rgb',{}).get('first_trigger_s'),r.get('evs',{}).get('first_trigger_s'),
                r.get('rgb',{}).get('episodes'),r.get('evs',{}).get('episodes')])
    rows=[]
    for r in results:
        rows.append('<tr><td>'+('<a href="'+html.escape(r['session'],quote=True)+'/scores.svg">波形</a>' if 'error' not in r else '')+'</td>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in (
            r['session'],r.get('error','OK'),r.get('rgb_first_visible_s'),
            r.get('rgb',{}).get('first_trigger_s'),r.get('evs',{}).get('first_trigger_s')) )+'</tr>')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>RC activity baselines</title><h1>RC飛び出し・活動量ベースライン</h1><p>車の分類ではなく閾値超過の候補です。時刻は元記録の秒数。最初の候補には誤検知が含まれます。波形は各記録のCSVに保存。</p><table border="1"><tr><th>波形</th><th>記録</th><th>状態</th><th>RGB初出現</th><th>RGB最初の候補</th><th>EVS最初の候補</th></tr>'+''.join(rows)+'</table>',encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--rgb-threshold',required=True,type=float,help='fraction of ROI pixels changing by rgb-pixel-delta')
    p.add_argument('--evs-threshold',required=True,type=float,help='events within trailing window')
    p.add_argument('--rgb-pixel-delta',type=float,default=15)
    p.add_argument('--step-ms',type=float,default=1);p.add_argument('--window-bins',type=int,default=2)
    p.add_argument('--rgb-topic',default='/realsense/color/image_raw')
    args=p.parse_args()
    if (not 0<args.rgb_threshold<=1 or not math.isfinite(args.evs_threshold) or args.evs_threshold<=0
        or not 0<args.rgb_pixel_delta<=255 or not math.isfinite(args.step_ms) or args.step_ms<=0 or args.window_bins<1):p.error('invalid thresholds/windows')
    if args.output.exists():p.error('output already exists; choose a new directory')
    config=json.loads(args.config.read_text());args.output.mkdir(parents=True)
    (args.output/'run_config.json').write_text(json.dumps(dict(common=config,common_sha256=digest(args.config),parameters={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}),indent=2))
    results=[]
    for entry in config['sessions']:
        name=entry['session'];print(name,flush=True)
        try:
            if Path(name).name!=name or name in ('.','..'):raise ValueError('invalid session name')
            result,rgb,evs=analyze(config,entry,args)
            folder=args.output/name;folder.mkdir()
            for sensor,series in [('rgb',rgb),('evs',evs)]:
                with (folder/(sensor+'_scores.csv')).open('w') as f:
                    writer=csv.writer(f);writer.writerow(['interval','relative_time_s','score']);writer.writerows(series)
            (folder/'result.json').write_text(json.dumps(result,indent=2))
            write_plot(folder/'scores.svg',result,rgb,evs)
        except Exception as exc:
            result=dict(session=name,error=str(exc));print('FAILED:',exc,flush=True)
        results.append(result)
        (args.output/'summary.json').write_text(json.dumps(results,indent=2))
        write_report(args.output,results)
    return int(any('error' in r for r in results))

if __name__=='__main__':raise SystemExit(main())
