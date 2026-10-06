"""Development-only background-distribution experiment using existing tile arrays.

Use --bundle for the validated five-scene archive, or --tile-dir on the data host.
Requires NumPy; --plots additionally requires Matplotlib. No ROS/RAW processing.
"""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
from zipfile import ZipFile

import numpy as np

from analyze_rc_popout_development_bundle import load_bundle
from rc_popout_change_detection import DEFAULT_SPLIT, digest, select_sessions, write_json
from rc_popout_local_detection import load_scene, validate_data
from rc_popout_grid_background import (ALGORITHM, DEFAULTS, alarm_episodes, calibrate_threshold,
                                      fit_background, score_maps, validate_settings)


def write_csv(path,rows,fields=None):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields or list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def load_input(bundle,tile_dir):
    split=json.loads(DEFAULT_SPLIT.read_text())
    sessions=select_sessions(split,'development')
    if bundle:
        scenes,frozen,hashes,_=load_bundle(bundle)
        with ZipFile(bundle) as archive:
            for name,scene in scenes.items():
                path=f'tile_dev_trial01/{name}/result.json'
                raw=archive.read(path)
                if hashlib.sha256(raw).hexdigest() != hashes[path]:
                    raise ValueError('bundle changed while loading')
                scene['source_hashes']=json.loads(raw)['input_hashes']
        return scenes,frozen['input_definition'],dict(mode='archive',path=str(bundle.resolve()),
            bundle_sha256=digest(bundle),input_sha256=hashes,
            verification='Archived hashes checked; current RAW, sync and annotations not reopened.')
    run=json.loads((tile_dir/'run_config.json').read_text())
    if run['split'] != split or run['sessions'] != sessions or run['subset'] != 'development':
        raise ValueError('only the fixed development tile cache is accepted')
    statuses=json.loads((tile_dir/'summary.json').read_text())
    if any(sum(r['session']==s and r['status']=='complete' for r in statuses) != 1 for s in sessions):
        raise ValueError('tile extraction incomplete')
    source=run['source_config']
    if (digest(run['source_config_path']) != run['source_config_sha256']
            or json.loads(Path(run['source_config_path']).read_text()) != source):
        raise ValueError('source config changed since extraction')
    definition=dict(tile_px=run['tile_px'],roi=source['common']['roi'],spatial=source['common']['spatial'],
        **{k:source['parameters'][k] for k in ('rgb_topic','rgb_pixel_delta','step_ms','window_bins')})
    scenes={}; hashes={}
    for session in sessions:
        alignment,meta,checked=load_scene(tile_dir,session,run)
        arrays={}
        for sensor in ('rgb','evs'):
            p=tile_dir/session/f'{sensor}_tiles.npz'
            with np.load(p,allow_pickle=False) as data:
                arrays[sensor]={k:data[k] for k in data.files}
            validate_data(arrays[sensor],meta,sensor)
            checked[sensor+'_tiles_sha256']=digest(p)
        scenes[session]=dict(meta=meta,alignment=alignment,data=arrays,source_hashes=checked)
        hashes[session]=checked
    return scenes,definition,dict(mode='tile_cache',path=str(tile_dir.resolve()),
        tile_run_sha256=digest(tile_dir/'run_config.json'),input_sha256=hashes,
        verification='Current source/annotation/sync hashes checked; RAW not decoded.')


def validate_scenes(scenes):
    split=json.loads(DEFAULT_SPLIT.read_text())
    if list(scenes) != select_sessions(split,'development'):
        raise ValueError('unexpected development scenes')
    negatives=[s for g in split['groups'] if g['condition']=='none' for s in g['development']]
    if len(negatives) != 1:
        raise ValueError('this experiment requires one development negative')
    negative=negatives[0]; reference=scenes[negative]
    if reference['alignment'].get('rgb_first_visible_from_drive_s') is not None:
        raise ValueError('negative fitting recording has an onset annotation')
    for name,s in scenes.items():
        if s['meta'] != reference['meta'] or s['alignment']['session'] != name:
            raise ValueError('scene geometry/alignment mismatch')
        for sensor in ('rgb','evs'):
            validate_data(s['data'][sensor],s['meta'],sensor)
    return negative


def report_result(name,sensor,scene,r,calibration,settings):
    episodes,alarm,active=alarm_episodes(r,calibration['threshold_s'],settings['release_ratio'])
    a=scene['alignment']; zero=a['drive_start_s']; onset=a.get('rgb_first_visible_from_drive_s')
    times=r['time_s']-zero
    drive=next(p for p in a['phases'] if p['phase']=='drive')
    drive_mask=(times>=drive['start_from_drive_s']) & (times<drive['end_from_drive_s'])
    pre=np.zeros(len(times),bool) if onset is None else times < onset-settings['onset_guard_s']
    post=np.zeros(len(times),bool) if onset is None else (times>=onset)&(times<=onset+.250)
    # Duration uses the state already available at the LEFT endpoint, never
    # credits a new alarm to the preceding observation interval.
    drive_ready=drive_active=0.
    for i in range(1,len(times)):
        if r['reset'][i] or not (r['ready'][i] and r['ready'][i-1]):
            continue
        for lo,hi in drive['intervals']:
            dt=max(0.,min(r['time_s'][i],hi)-max(r['time_s'][i-1],lo))
            drive_ready+=dt
            drive_active+=dt if active[i-1] else 0.
    candidates=[]
    for j,e in enumerate(episodes,1):
        i=e['start_index']; start=e['start_time_s']-zero
        candidates.append(dict(session=name,method=sensor,candidate=j,
            start_relative_time_s=e['start_time_s'],start_from_drive_s=start,
            end_from_drive_s=e['end_time_s']-zero,end_reason=e['end_reason'],
            minus_onset_ms=None if onset is None else (start-onset)*1000,
            pair_tile_a=e['pair_tile_ids_at_start'][0],pair_tile_b=e['pair_tile_ids_at_start'][1],
            background_phase=('pre_drive','drive')[r['background_index'][i]],score_s=float(r['score'][i])))
    def peak(mask):
        v=r['score'][mask & r['ready']]
        return float(v.max()) if len(v) else None
    summary=dict(session=name,method=sensor,status='complete',samples=len(times),
        ready_samples=int(r['ready'].sum()),threshold_s=calibration['threshold_s'],
        candidates=len(candidates),drive_candidates=int(alarm[drive_mask].sum()),
        drive_ready_seconds=drive_ready,drive_active_seconds=drive_active,
        first_candidate_from_drive_s=candidates[0]['start_from_drive_s'] if candidates else None,
        rgb_first_visible_from_drive_s=onset,
        first_candidate_minus_onset_ms=candidates[0]['minus_onset_ms'] if candidates else None,
        candidates_before_guard=None if onset is None else int(alarm[pre].sum()),
        candidates_onset_to_250ms=None if onset is None else int(alarm[post].sum()),
        max_score_s=peak(np.ones(len(times),bool)),
        pre_onset_max_score_s=peak(pre),onset_to_250ms_max_score_s=peak(post))
    rows=[]
    for i,t in enumerate(times):
        aa,bb=r['winner_pair'][i]
        rows.append(dict(relative_time_s=float(r['time_s'][i]),from_drive_s=float(t),
            interval=int(r['interval'][i]),ready=bool(r['ready'][i]),reset=bool(r['reset'][i]),
            background_phase=('pre_drive','drive')[r['background_index'][i]],
            background_rmse=float(r['background_rmse'][i]),score_s=float(r['score'][i]),
            threshold_s=calibration['threshold_s'],alarm=bool(alarm[i]),active=bool(active[i]),
            pair_tile_a=int(r['tile_id'][aa]),pair_tile_b=int(r['tile_id'][bb])))
    return summary,candidates,rows,alarm,active


def figures(output,scenes,results,calibration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(scenes),2,figsize=(12,12),layout='constrained')
    for row,(name,scene) in enumerate(scenes.items()):
        zero=scene['alignment']['drive_start_s']; onset=scene['alignment'].get('rgb_first_visible_from_drive_s')
        for col,sensor in enumerate(('rgb','evs')):
            r=results[name,sensor]; ax=axes[row,col]; t=r['time_s']-zero
            ax.plot(t,r['score'],lw=.9,color='#2463a6')
            ax.axhline(calibration[sensor]['threshold_s'],color='#aa2233',ls='--',lw=1)
            if onset is not None: ax.axvline(onset,color='#19834c',ls='--',lw=1)
            ax.axvline(0,color='gray',lw=.8); ax.set_xlim(-.3,3.2)
            ax.set(title=f'{name} / {sensor}',ylabel='Adjacent residual integral [s]')
            if row==len(scenes)-1: ax.set_xlabel('Time from drive command [s]')
            ax.grid(alpha=.2)
    fig.suptitle('Development only: fixed background maps / thresholds fitted on one negative')
    fig.savefig(output/'score_overview.png',dpi=150);fig.savefig(output/'score_overview.pdf');plt.close(fig)
    # These maps are feature evidence, not RGB vehicle boxes or segmentation GT.
    for name,scene in scenes.items():
        for sensor in ('rgb','evs'):
            r=results[name,sensor]
            onset=scene['alignment'].get('rgb_first_visible_from_drive_s')
            candidates=np.flatnonzero(r['alarm'])
            stamps=[('first_candidate',int(candidates[0]))] if len(candidates) else []
            if onset is not None:
                i=int(np.searchsorted(r['time_s'],scene['alignment']['drive_start_s']+onset))
                if i<len(r['time_s']): stamps.append(('at_or_after_rgb_onset',i))
            for label,i in stamps:
                width,height=scene['meta']['output_size']; size=(height//32,width//32)
                maps=[]
                observed=scene['data'][sensor]['counts'][i]/scene['data'][sensor]['valid_pixels']
                for values in (observed,r['predicted_density'][i],r['residual_z'][i]):
                    arr=np.full(size,np.nan)
                    for tile,v in zip(scene['meta']['tiles'],values): arr[tile['y']//32,tile['x']//32]=v
                    maps.append(arr)
                fg,axs=plt.subplots(1,3,figsize=(12,3.5),layout='constrained')
                vmax=max(float(observed.max()),float(r['predicted_density'][i].max()),1e-6)
                for j,(ax,arr,title) in enumerate(zip(axs,maps,('Observed density','Estimated background','Positive standardized residual'))):
                    im=ax.imshow(arr,origin='upper',vmin=0,vmax=vmax if j<2 else 10,
                                 extent=(0,width,height,0),interpolation='nearest',cmap='magma')
                    ax.set(title=title,xlabel='EVS x [px]',ylabel='EVS y [px]');fg.colorbar(im,ax=ax,shrink=.7)
                time=r['time_s'][i]-scene['alignment']['drive_start_s']
                fg.suptitle(f'{name} / {sensor} / {label}: drive {time:.4f} s')
                fg.savefig(output/name/f'{sensor}_{label}_maps.png',dpi=120);plt.close(fg)


def threshold_sensitivity(scenes,results,negative,settings,margins):
    """Explicit development exploration; does not choose or replace primary settings."""
    summaries=[];candidates=[]
    for margin in margins:
        probe=dict(settings,threshold_margin=margin);validate_settings(probe)
        calibration={sensor:calibrate_threshold(results[negative,sensor],probe) for sensor in ('rgb','evs')}
        for name,scene in scenes.items():
            for sensor in ('rgb','evs'):
                summary,events,_,_,_=report_result(name,sensor,scene,results[name,sensor],calibration[sensor],probe)
                summaries.append(dict(threshold_margin=margin,**summary))
                candidates.extend(dict(threshold_margin=margin,**e) for e in events)
    return summaries,candidates


def review_manifest(scenes,negative,candidates,definition,parameter_hash):
    """Export all four EVS candidates only when each positive has exactly one.

    Never select the closest-to-onset candidate from a run with false alarms.
    Existing video review validates current annotations, sync and calibration.
    """
    selected=[]
    for name,scene in scenes.items():
        if name==negative: continue
        events=[e for e in candidates if e['session']==name and e['method']=='evs']
        if len(events)!=1:
            return None,f'{name}: {len(events)} EVS candidates; review all candidates in CSV instead of selecting one'
        a=scene['alignment'];onset=a.get('rgb_first_visible_from_drive_s')
        hashes=scene.get('source_hashes',{})
        if onset is None or not {'annotation_sha256','time_sync_sha256'} <= hashes.keys():
            return None,f'{name}: review timestamps/source hashes unavailable'
        e=events[0];tiles={t['tile_id']:t for t in scene['meta']['tiles']}
        selected.append(dict(session=name,candidate_recording_s=e['start_relative_time_s'],
            candidate_from_drive_s=e['start_from_drive_s'],drive_start_s=a['drive_start_s'],
            rgb_onset_recording_s=a['drive_start_s']+onset,reference_origin_s=a['reference_origin_s'],
            rgb_timestamp_source=a['timestamp_source'],annotation_sha256=hashes['annotation_sha256'],
            time_sync_sha256=hashes['time_sync_sha256'],
            tiles=[tiles[e['pair_tile_a']],tiles[e['pair_tile_b']]]))
    return dict(schema_version=1,purpose='Grid-background EVS candidates, development spatial review only',
        algorithm=ALGORITHM,detector_parameters_sha256=parameter_hash,split_sha256=digest(DEFAULT_SPLIT),
        detector_window_ms=definition['step_ms']*definition['window_bins'],
        spatial=definition['spatial'],scenes=selected),'eligible; all four EVS candidates included without onset-based selection'


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--bundle',type=Path);source.add_argument('--tile-dir',type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--subset',choices=['development'],default='development')
    p.add_argument('--plots',action='store_true')
    p.add_argument('--probe-margins',nargs='+',type=float,default=[],
                   help='Additional development-only threshold sensitivity; primary setting remains unchanged')
    for k,v in DEFAULTS.items(): p.add_argument('--'+k.replace('_','-'),type=type(v),default=v)
    args=p.parse_args(argv)
    try:
        if args.output.exists(): raise ValueError('output already exists; choose a new directory')
        settings={k:getattr(args,k) for k in DEFAULTS};validate_settings(settings)
        for margin in args.probe_margins: validate_settings(dict(settings,threshold_margin=margin))
        scenes,definition,provenance=load_input(args.bundle,args.tile_dir)
        negative=validate_scenes(scenes);n=scenes[negative]
        if definition['tile_px'] != 32: raise ValueError('this experiment requires 32 px tiles')
        if settings['evs_max_gap_s'] < n['meta']['event_window_s']:
            raise ValueError('EVS gap limit shorter than event observation window')
        if args.plots:
            import matplotlib  # Fail before computing if optional dependency is absent.
        models={};calibration={};results={}
        for sensor in ('rgb','evs'):
            models[sensor]=fit_background(n['data'][sensor],n['meta'],n['alignment']['phases'],sensor,settings)
            r=score_maps(n['data'][sensor],n['meta'],sensor,settings,models[sensor],32)
            results[negative,sensor]=r;calibration[sensor]=calibrate_threshold(r,settings)
        args.output.mkdir(parents=True)
        write_json(args.output/'background_models.json',models)
        contract=dict(algorithm=ALGORITHM,settings=settings,calibration=calibration,
            background_session=negative,model_sha256=digest(args.output/'background_models.json'),
            input_definition=definition,split_sha256=digest(DEFAULT_SPLIT),
            code_sha256={n:digest(Path(__file__).with_name(n)) for n in
                ('rc_popout_grid_background.py','analyze_rc_popout_grid_background.py',
                 'rc_popout_local_detection.py','rc_popout_change_detection.py',
                 'analyze_rc_popout_development_bundle.py')},status='development_candidate_not_evaluated')
        write_json(args.output/'detector_parameters.json',contract)
        write_json(args.output/'run_config.json',dict(**contract,provenance=provenance,
            sessions=list(scenes),subset='development',numpy_version=np.__version__,
            probe_margins=args.probe_margins,
            note='Negative reuse is fitting, not an independent specificity test. Labels enter reports only.'))
        summaries=[];all_candidates=[]
        for name,scene in scenes.items():
            out=args.output/name;out.mkdir()
            for sensor in ('rgb','evs'):
                r=results.get((name,sensor))
                if r is None:
                    r=score_maps(scene['data'][sensor],scene['meta'],sensor,settings,models[sensor],32)
                    results[name,sensor]=r
                summary,candidates,rows,alarm,active=report_result(name,sensor,scene,r,calibration[sensor],settings)
                summaries.append(summary);all_candidates.extend(candidates)
                r.update(alarm=alarm,active=active)
                write_csv(out/f'{sensor}_background_scores.csv',rows)
                np.savez_compressed(out/f'{sensor}_background_maps.npz',**r)
                write_json(out/f'{sensor}_candidates.json',candidates)
                print(f'{name} / {sensor}: candidates={summary["candidates"]}, '
                      f'before_onset_guard={summary["candidates_before_guard"]}, '
                      f'first-onset={summary["first_candidate_minus_onset_ms"]} ms',flush=True)
            write_json(out/'result.json',dict(alignment=scene['alignment'],geometry=scene['meta'],
                algorithm=ALGORITHM,parameters_sha256=digest(args.output/'detector_parameters.json')))
        write_csv(args.output/'summary.csv',summaries);write_json(args.output/'summary.json',summaries)
        write_csv(args.output/'candidates.csv',all_candidates,fields=['session','method','candidate',
            'start_relative_time_s','start_from_drive_s','end_from_drive_s','end_reason','minus_onset_ms',
            'pair_tile_a','pair_tile_b','background_phase','score_s'])
        manifest,reason=review_manifest(scenes,negative,all_candidates,definition,digest(args.output/'detector_parameters.json'))
        write_json(args.output/'candidate_review_status.json',dict(available=manifest is not None,reason=reason))
        if manifest: write_json(args.output/'candidate_review_manifest.json',manifest)
        if args.probe_margins:
            probe_summary,probe_candidates=threshold_sensitivity(scenes,results,negative,settings,args.probe_margins)
            write_csv(args.output/'threshold_sensitivity.csv',probe_summary)
            write_csv(args.output/'threshold_sensitivity_candidates.csv',probe_candidates,fields=['threshold_margin',
                'session','method','candidate','start_relative_time_s','start_from_drive_s','end_from_drive_s',
                'end_reason','minus_onset_ms','pair_tile_a','pair_tile_b','background_phase','score_s'])
        if args.plots: figures(args.output,scenes,results,calibration)
        links=['<p>Development only. One negative fits models and thresholds; zero alarms there is not test performance.</p>',
               '<p>No onset, drive phase or direction enters inference. A candidate is not confirmed vehicle detection.</p>',
               '<a href="summary.csv">Summary CSV</a> | <a href="candidates.csv">All candidate locations/times</a>']
        if args.probe_margins:
            links.append('<p><a href="threshold_sensitivity.csv">Exploratory threshold sensitivity</a>; not independent evaluation.</p>')
        if args.plots:
            links.append('<p>Overview is a drive zoom; CSVs include all annotated times.</p><img width="100%" src="score_overview.png">')
            for image in sorted(args.output.glob('*/*_maps.png')):
                rel=html.escape(image.relative_to(args.output).as_posix(),quote=True)
                links.append(f'<h2>{rel}</h2><img width="100%" src="{rel}">')
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Grid background development</title>'+''.join(links))
        print(f'Report: {args.output}/index.html',flush=True)
        if manifest: print(f'Video review manifest: {args.output}/candidate_review_manifest.json',flush=True)
        return 0
    except (ValueError,KeyError,TypeError,OSError,ImportError,StopIteration) as e:
        p.exit(1,f'error: {e}\n')


if __name__=='__main__':
    raise SystemExit(main())
