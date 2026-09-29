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


def spatial_scores(parts, n_bins, window_bins, width, height, tile_px, min_pixels, np):
    """Exact trailing-window counts and distinct pixels in fixed nonoverlapping tiles.

    Input order is irrelevant. Only occupied pixel/bin pairs are stored.
    """
    pixel_count=width*height
    tile_cols=(width+tile_px-1)//tile_px
    tile_count=tile_cols*((height+tile_px-1)//tile_px)
    if parts:
        data=np.concatenate(parts)
        keys,multiplicity=np.unique(data[:,0]*pixel_count+data[:,1],return_counts=True)
        bin_ids=keys//pixel_count;pixels=keys%pixel_count
    else:
        bin_ids=pixels=multiplicity=np.array([],dtype=np.int64)
    offsets=np.searchsorted(bin_ids,np.arange(n_bins+1))
    live={}; tile_events=np.zeros(tile_count,np.int64);tile_pixels=np.zeros(tile_count,np.int64)
    scores=[];peak_events=[];peak_pixels=[]
    def update(bin_index,sign):
        for j in range(offsets[bin_index],offsets[bin_index+1]):
            pixel=int(pixels[j]);amount=int(multiplicity[j])*sign
            tile=((pixel//width)//tile_px)*tile_cols+(pixel%width)//tile_px
            before=live.get(pixel,0);after=before+amount
            tile_events[tile]+=amount
            if before==0 and after>0:tile_pixels[tile]+=1
            if before>0 and after==0:tile_pixels[tile]-=1
            if after:live[pixel]=after
            else:live.pop(pixel,None)
    for i in range(n_bins):
        update(i,1)
        if i>=window_bins:update(i-window_bins,-1)
        if i+1>=window_bins:
            qualified=np.where(tile_pixels>=min_pixels,tile_events,0)
            scores.append(int(qualified.max()))
            peak_events.append(int(tile_events.max()))
            peak_pixels.append(int(tile_pixels.max()))
    return scores,peak_events,peak_pixels


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
    spatial_parts=[[] for _ in spans] if getattr(args,'spatial',False) else None
    first_event=None;last_event=None
    previous_raw_time=None; backward_steps=0; max_backward_us=0
    for batch in source.batches():
        if (source.width,source.height)!=(w,h): raise ValueError('RAW dimensions mismatch')
        events=batch.events
        provisional = source.anchor.reference_time_s + source.anchor.scale * (events['t'].astype(np.float64)-source.anchor.source_time_us)/1e6
        ts=clock.apply(provisional)-origin
        if len(ts)==0: continue
        if not np.all(np.isfinite(ts)):
            raise ValueError('nonfinite event timestamps')
        # Histogram addition is order independent. Preserve events and timestamps;
        # do not sort, clamp, or discard late events, including across batches.
        raw_times=events['t'].astype(np.int64)
        differences=np.diff(raw_times)
        if previous_raw_time is not None:
            differences=np.concatenate(([int(raw_times[0])-previous_raw_time],differences))
        negative=differences[differences<0]
        backward_steps+=int(len(negative))
        if len(negative):max_backward_us=max(max_backward_us,int(-negative.min()))
        previous_raw_time=int(raw_times[-1])
        batch_min,batch_max=float(ts.min()),float(ts.max())
        first_event=batch_min if first_event is None else min(first_event,batch_min)
        last_event=batch_max if last_event is None else max(last_event,batch_max)
        valid=(events['x']<w)&(events['y']<h)
        kept=np.zeros(len(events),bool);kept[valid]=lut[events['y'][valid],events['x'][valid]]
        for i,(a,b) in enumerate(spans):
            chosen=kept & (ts>=a) & (ts<b)
            index=np.floor((ts[chosen]-a)/step).astype(int)
            accepted=(index>=0)&(index<len(hist[i]))
            if spatial_parts is not None:
                ex,ey=events['x'][chosen][accepted],events['y'][chosen][accepted]
                pixels=uy[ey,ex]*w+ux[ey,ex]
                if len(pixels):spatial_parts[i].append(np.column_stack((index[accepted],pixels)))
            index=index[accepted]
            hist[i]+=np.bincount(index,minlength=len(hist[i]))
        # Read to EOF: a later batch can still contain in-range timestamps.
    if first_event is None or first_event>spans[0][0] or last_event<spans[-1][1]:
        raise ValueError('RAW event extent does not cover evaluation intervals; cannot treat missing data as silence')
    evs_rows=[];spatial_rows=[];spatial_details=[]
    for i,((a,b),counts) in enumerate(zip(spans,hist)):
        for j,count in enumerate(trailing_counts(counts,bins,np),start=bins):
            t=a+j*step
            if t<b: evs_rows.append((i,t,int(count)))
    if spatial_parts is not None:
        for i,((a,b),counts,parts) in enumerate(zip(spans,hist,spatial_parts)):
            scores,peaks,pixels=spatial_scores(parts,len(counts),bins,w,h,args.tile_px,args.min_active_pixels,np)
            spatial_parts[i]=None
            for j,(score,peak,occupied) in enumerate(zip(scores,peaks,pixels),start=bins):
                t=a+j*step
                if t<b:
                    spatial_rows.append((i,t,score))
                    spatial_details.append((i,t,score,peak,occupied))
    if not rgb_rows or not evs_rows: raise ValueError('insufficient samples')
    onset=ann.get('onset',{}).get('first_visible')
    onset_s=None if onset is None else onset['rgb_time_s']-origin
    result=dict(session=ann['session'],annotation=str(ann_path),annotation_sha256=digest(ann_path),
                roi=config['roi'],valid_pixels=int(mask.sum()),evaluation_seconds=sum(b-a for a,b in spans),
                intervals=spans,rgb_first_visible_s=onset_s,time_sync_sha256=digest(sync),
                event_ordering=dict(backward_steps=backward_steps,max_backward_step_us=max_backward_us,
                    policy='full-file timestamp histogram; no timestamp modification'),
                camchain_sha256=digest(chain),annotation_kind='no_rgb_onset' if onset_s is None else 'rgb_onset',
                note='Offline activity threshold crossings, not vehicle classification or end-to-end latency.')
    methods=[('rgb',rgb_rows,args.rgb_threshold),('evs',evs_rows,args.evs_threshold)]
    if spatial_parts is not None:methods.append(('evs_spatial',spatial_rows,args.spatial_threshold))
    for sensor,rows,threshold in methods:
        triggers=crossings(rows,threshold)
        result[sensor]=dict(threshold=threshold,triggers_s=triggers,episodes=len(triggers),
            episodes_per_minute=len(triggers)*60/result['evaluation_seconds'],
            first_trigger_s=triggers[0] if triggers else None,
            first_trigger_minus_rgb_onset_ms=(triggers[0]-onset_s)*1000 if triggers and onset_s is not None else None)
    result['score_distribution']={sensor:dict(zip(['p50','p95','p99','p99_9','max'],
        map(float,np.percentile([v for _,t,v in rows],[50,95,99,99.9,100])))) for sensor,rows,_ in methods}
    if spatial_parts is not None:
        result['spatial_parameters']=dict(tile_px=args.tile_px,min_active_pixels=args.min_active_pixels,
            note='fixed tiles, distinct undistorted EVS pixels; not a vehicle classifier')
        result['_spatial_rows']=spatial_details
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
        writer.writerow(['session','error','evaluation_seconds','rgb_first_visible_s','rgb_first_trigger_s','evs_first_trigger_s','rgb_episodes','evs_episodes','evs_spatial_first_trigger_s','evs_spatial_episodes'])
        for r in results:
            writer.writerow([r['session'],r.get('error',''),r.get('evaluation_seconds'),r.get('rgb_first_visible_s'),
                r.get('rgb',{}).get('first_trigger_s'),r.get('evs',{}).get('first_trigger_s'),
                r.get('rgb',{}).get('episodes'),r.get('evs',{}).get('episodes'),
                r.get('evs_spatial',{}).get('first_trigger_s'),r.get('evs_spatial',{}).get('episodes')])
    with (output/'background_distribution.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['session','annotation_kind','evaluation_seconds','method','p50','p95','p99','p99_9','max','episodes'])
        for r in results:
            if r.get('annotation_kind')!='no_rgb_onset':continue
            for method,dist in r.get('score_distribution',{}).items():
                writer.writerow([r['session'],r['annotation_kind'],r['evaluation_seconds'],method,
                    *[dist[k] for k in ('p50','p95','p99','p99_9','max')],r[method]['episodes']])
    rows=[]
    for r in results:
        rows.append('<tr><td>'+('<a href="'+html.escape(r['session'],quote=True)+'/spatial_scores.svg">局所波形</a> ' if 'evs_spatial' in r else '')+('<a href="'+html.escape(r['session'],quote=True)+'/scores.svg">波形</a>' if 'error' not in r else '')+'</td>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in (
            r['session'],r.get('error','OK'),r.get('rgb_first_visible_s'),
            r.get('rgb',{}).get('first_trigger_s'),r.get('evs',{}).get('first_trigger_s'),r.get('evs_spatial',{}).get('first_trigger_s')) )+'</tr>')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>RC activity baselines</title><h1>RC飛び出し・活動量ベースライン</h1><p>車の分類ではなく閾値超過の候補です。時刻は元記録の秒数。最初の候補には誤検知が含まれます。波形は各記録のCSVに保存。</p><table border="1"><tr><th>波形</th><th>記録</th><th>状態</th><th>RGB初出現</th><th>RGB最初の候補</th><th>EVS最初の候補</th><th>局所EVS最初の候補</th></tr>'+''.join(rows)+'</table>',encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--rgb-threshold',required=True,type=float,help='fraction of ROI pixels changing by rgb-pixel-delta')
    p.add_argument('--evs-threshold',required=True,type=float,help='events within trailing window')
    p.add_argument('--rgb-pixel-delta',type=float,default=15)
    p.add_argument('--step-ms',type=float,default=1);p.add_argument('--window-bins',type=int,default=2)
    p.add_argument('--spatial',action='store_true')
    p.add_argument('--tile-px',type=int,default=32)
    p.add_argument('--min-active-pixels',type=int,default=3)
    p.add_argument('--spatial-threshold',type=float,default=20)
    p.add_argument('--rgb-topic',default='/realsense/color/image_raw')
    args=p.parse_args()
    if (not 0<args.rgb_threshold<=1 or not math.isfinite(args.evs_threshold) or args.evs_threshold<=0
        or not 0<args.rgb_pixel_delta<=255 or not math.isfinite(args.step_ms) or args.step_ms<=0 or args.window_bins<1):p.error('invalid thresholds/windows')
    if args.tile_px<1 or args.min_active_pixels<1 or not math.isfinite(args.spatial_threshold) or args.spatial_threshold<=0:p.error('invalid spatial settings')
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
            details=result.pop('_spatial_rows',None)
            if details is not None:
                with (folder/'evs_spatial_scores.csv').open('w') as stream:
                    writer=csv.writer(stream);writer.writerow(['interval','relative_time_s','score','max_tile_events','max_tile_active_pixels']);writer.writerows(details)
                spatial_series=[(i,t,v) for i,t,v,_,_ in details]
                spatial_plot=dict(result,evs=result['evs_spatial'])
                write_plot(folder/'spatial_scores.svg',spatial_plot,rgb,spatial_series)
            (folder/'result.json').write_text(json.dumps(result,indent=2))
            write_plot(folder/'scores.svg',result,rgb,evs)
        except Exception as exc:
            result=dict(session=name,error=str(exc));print('FAILED:',exc,flush=True)
        results.append(result)
        (args.output/'summary.json').write_text(json.dumps(results,indent=2))
        write_report(args.output,results)
    return int(any('error' in r for r in results))

if __name__=='__main__':raise SystemExit(main())
