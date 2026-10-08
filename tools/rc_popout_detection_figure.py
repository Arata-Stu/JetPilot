"""Text-free method figure from archived measured tiles and native rosbag RGB.

The frozen detector is replayed without refitting. RGB is the last acquired frame
at the EVS candidate time. No generated scene, event points, or inpainting is used.
"""
import argparse
import bisect
import html
import importlib
import json
from pathlib import Path
import sys

import numpy as np

from analyze_rc_popout_grid_background import load_input
from evaluate_rc_popout_grid_background import load_frozen
from rc_popout_candidate_videos import digest, load_manifest, write_json
from rc_popout_grid_background import alarm_episodes, score_maps

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT/'docs/evidence/rc_popout_20260930'
MANIFEST = EVIDENCE/'grid_background_20261006/candidate_review_manifest.json'
FROZEN = EVIDENCE/'grid_background_frozen_20261006'
CALIBRATION = ROOT/'ros2_ws/src/tool/multi_sensor_calibration'


def read_json(path):
    return json.loads(Path(path).read_text())


def load_evidence(bundle, frozen, session):
    parameters, models, freeze = load_frozen(frozen)
    development = read_json(frozen/'development_run_config.json')
    if digest(bundle) != development['provenance']['bundle_sha256']:
        raise ValueError('development bundle differs from the frozen input')
    manifest = load_manifest(MANIFEST)
    if digest(frozen/'detector_parameters.json') != manifest['detector_parameters_sha256']:
        raise ValueError('candidate manifest/frozen parameters mismatch')
    scenes, definition, provenance = load_input(bundle, None)
    if definition != parameters['input_definition']:
        raise ValueError('frozen ROI/calibration/input definition differs from tile archive')
    matches = [s for s in manifest['scenes'] if s['session'] == session]
    if len(matches) != 1:
        raise ValueError('choose test_01, test_05, test_11 or test_14 (development illustration only)')
    candidate, scene = matches[0], scenes[session]
    if (scene['alignment']['reference_origin_s'] != candidate['reference_origin_s']
            or scene['source_hashes']['annotation_sha256'] != candidate['annotation_sha256']
            or scene['source_hashes']['time_sync_sha256'] != candidate['time_sync_sha256']
            or definition['spatial'] != manifest['spatial']):
        raise ValueError('candidate/tile identity mismatch')
    return dict(parameters=parameters, models=models, scene=scene,
                candidate=candidate, manifest=manifest, provenance=provenance,
                frozen_manifest_sha256=digest(frozen/'freeze.json'))


def replay(evidence):
    p, scene = evidence['parameters'], evidence['scene']
    result = score_maps(scene['data']['evs'], scene['meta'], 'evs', p['settings'],
                        evidence['models']['evs'], p['input_definition']['tile_px'])
    threshold = p['calibration']['evs']['threshold_s']
    episodes, alarm, active = alarm_episodes(result, threshold, p['settings']['release_ratio'])
    expected = evidence['candidate']
    if not episodes:
        raise ValueError('frozen replay produced no candidate')
    first = episodes[0]
    if (abs(first['start_time_s']-expected['candidate_recording_s']) > 1e-8
            or first['pair_tile_ids_at_start'] != [t['tile_id'] for t in expected['tiles']]):
        raise ValueError('replayed first candidate differs from the saved review manifest')
    index = first['start_index']
    if not result['ready'][index] or not active[index] or not alarm[index]:
        raise ValueError('candidate sample is not an observable alarm onset')
    return result, index


def validate_sources(record_root, camchain, evidence):
    c = evidence['candidate']
    ann_dir = record_root/'analysis/led_sync'/c['session']
    annotation = ann_dir/'sequence_annotations.json'
    if digest(annotation) != c['annotation_sha256']:
        raise ValueError('annotation changed since the archived detector input')
    ann = read_json(annotation)
    if ann['session'] != c['session']:
        raise ValueError('annotation session mismatch')
    # Match the source pinned in the archive; never silently choose newer sync.
    syncs = [p for p in (ann_dir/'time_sync_led.yaml', ann_dir/'time_sync_led_auto.yaml')
             if p.is_file() and digest(p) == c['time_sync_sha256']]
    if not syncs:
        raise ValueError('no time-sync YAML matches the archived detector input')
    spatial = evidence['parameters']['input_definition']['spatial']
    if digest(camchain) != spatial['camchain_sha256']:
        raise ValueError('camera calibration changed since extraction')
    bag = record_root/c['session']
    if not (bag/'metadata.yaml').is_file() or not list(bag.glob('*.mcap')):
        raise ValueError(f'rosbag metadata/MCAP missing: {bag}')
    return dict(bag=bag, annotation=annotation, time_sync=syncs[0], camchain=camchain)


def held_rgb_index(times, target):
    if (not len(times) or not np.all(np.isfinite(times))
            or np.any(np.diff(times) <= 0) or not np.isfinite(target)):
        raise ValueError('invalid RGB timestamp sequence')
    i = bisect.bisect_right(times, target)-1
    if i < 0 or target-times[i] > .100:
        raise ValueError('no recent past RGB frame at the EVS candidate')
    return i


def validate_support(common, meta):
    roi=meta['roi']
    x,y,w,h=(roi[k] for k in ('x','y','width','height'))
    support=np.zeros_like(common)
    support[y:y+h,x:x+w]=common[y:y+h,x:x+w]
    for tile in meta['tiles']:
        tx,ty,tw,th=(tile[k] for k in ('x','y','width','height'))
        if int(support[ty:ty+th,tx:tx+tw].sum()) != tile['valid_pixels']:
            raise ValueError('current common-view/ROI support differs from archived tile geometry')


def extract_rgb(sources, evidence, result, index):
    import cv2
    sys.path.insert(0, str(CALIBRATION))
    from multi_sensor_calibration import scenario_overlay as renderer
    from multi_sensor_calibration.io import load_yaml

    c, scene = evidence['candidate'], evidence['scene']
    definition = evidence['parameters']['input_definition']
    spatial = definition['spatial']
    origin, _, times = renderer._selected_rgb_times(sources['bag'], definition['rgb_topic'],
        c['rgb_timestamp_source'], start_s=0., duration_s=None, every_n=1, max_frames=None)
    if abs(origin-c['reference_origin_s']) > 1e-6:
        raise ValueError('native RGB origin differs from archived recording')
    # Verify native timing against every saved RGB tile endpoint, not MP4 time.
    relative = np.asarray(times)-origin
    stored = scene['data']['rgb']['time_s']
    ix = np.searchsorted(relative, stored-1e-6)
    if np.any(ix >= len(relative)) or np.any(np.abs(relative[ix]-stored) > 1e-6):
        raise ValueError('native RGB timestamps do not match archived tile inputs')
    target = origin+float(result['time_s'][index])
    selected = held_rgb_index(times, target)
    frames = list(renderer._rgb_frames(sources['bag'], definition['rgb_topic'],
                                      c['rgb_timestamp_source'], [selected]))
    if len(frames) != 1 or abs(frames[0][0]-times[selected]) > 1e-6:
        raise ValueError('decoded RGB frame/timestamp mismatch')
    rgb_time, pixels = frames[0]
    chain = load_yaml(sources['camchain'])
    evs, rgb = renderer._camera(chain, 'cam0', np), renderer._camera(chain, 'cam1', np)
    if (pixels.shape[1], pixels.shape[0]) != rgb['size']:
        raise ValueError('RGB dimensions differ from calibration')
    size = tuple(spatial['output_size'])
    transforms = [np.asarray(spatial[k], float) for k in
                  ('event_to_view_homography', 'rgb_to_view_homography')]
    supports, remaps = [], []
    for camera, transform in zip((evs, rgb), transforms):
        remap = cv2.initUndistortRectifyMap(camera['matrix'], camera['distortion'], None,
                                           camera['matrix'], camera['size'], cv2.CV_32FC1)
        valid = np.ones((camera['size'][1], camera['size'][0]), np.float32)
        valid = cv2.remap(valid, *remap, cv2.INTER_LINEAR)
        valid = cv2.warpPerspective(valid, transform, size, flags=cv2.INTER_LINEAR)
        supports.append(valid >= .999)
        remaps.append(remap)
    common = supports[0] & supports[1]
    validate_support(common, scene['meta'])
    rgb_view = cv2.warpPerspective(cv2.remap(pixels, *remaps[1], cv2.INTER_LINEAR),
                                    transforms[1], size)
    rgb_view[~common] = (35,35,35)
    timing = dict(source_rgb_frame=selected, rgb_time_s=rgb_time,
                  rgb_relative_s=rgb_time-origin, reference_origin_s=origin,
                  evs_time_s=target, evs_relative_s=float(result['time_s'][index]),
                  rgb_age_ms=(target-rgb_time)*1000,
                  event_support_start_s=float(result['support_start_s'][index]))
    return cv2.cvtColor(rgb_view, cv2.COLOR_BGR2RGB), common, timing


def map_values(evidence, result, index):
    data = evidence['scene']['data']['evs']
    threshold = evidence['parameters']['calibration']['evs']['threshold_s']
    return dict(observed_density=data['counts'][index]/data['valid_pixels'],
                estimated_background=result['predicted_density'][index],
                positive_residual=result['residual_z'][index],
                integrated_residual=result['state_s'][index],
                threshold_s=threshold)


def draw_figure(output, rgb, common, evidence, result, index):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyArrowPatch
    from PIL import Image

    meta = evidence['scene']['meta']
    roi = meta['roi']; x,y,w,h = (roi[k] for k in ('x','y','width','height'))
    values = map_values(evidence, result, index)
    scale = max(float(np.max(values[k])) for k in ('observed_density','estimated_background'))
    scale = max(scale, 1e-9)
    limits = dict(observed_density=scale, estimated_background=scale,
                  positive_residual=evidence['parameters']['settings']['z_clip'],
                  integrated_residual=values['threshold_s'])
    colors = dict(observed_density='#2d659b',estimated_background='#2d659b',
                  positive_residual='#ce6918',integrated_residual='#ce6918')

    def panel(ax, name, mark=False):
        if name == 'estimated_background':
            ax.imshow(np.full_like(rgb, 246), interpolation='nearest')
        else:
            ax.imshow(rgb, interpolation='nearest')
        # The photo is contextual. Tile colors are actual archived numerical
        # quantities, not denoised/reconstructed event pixels.
        for tile, value in zip(meta['tiles'], values[name]):
            tx,ty,tw,th = (tile[k] for k in ('x','y','width','height'))
            alpha = .74*min(1., max(0.,float(value)/limits[name]))
            if alpha > 0:
                ax.add_patch(Rectangle((tx-.5,ty-.5),tw,th,facecolor=colors[name],
                                      edgecolor='none',alpha=alpha))
            ax.add_patch(Rectangle((tx-.5,ty-.5),tw,th,fill=False,
                                  edgecolor='white',lw=.28,alpha=.26))
        if mark:
            for tile in evidence['candidate']['tiles']:
                tx,ty,tw,th = (tile[k] for k in ('x','y','width','height'))
                ax.add_patch(Rectangle((tx-.5,ty-.5),tw,th,fill=False,ec='white',lw=3.0))
                ax.add_patch(Rectangle((tx-.5,ty-.5),tw,th,fill=False,ec='#d26000',lw=1.7))
        ax.set(xlim=(x-.5,x+w-.5),ylim=(y+h-.5,y-.5));ax.axis('off')

    fig = plt.figure(figsize=(15,5.5),facecolor='white')
    for pos,name in zip((.02,.358,.696),
                        ('observed_density','positive_residual','integrated_residual')):
        panel(fig.add_axes([pos,.415,.284,.50]),name,name=='integrated_residual')
    for start,end in (((.308,.665),(.349,.665)),((.646,.665),(.687,.665))):
        fig.add_artist(FancyArrowPatch(start,end,transform=fig.transFigure,
            arrowstyle='-|>',mutation_scale=23,lw=2,color='#2d659b'))
    # A tile-only inset prevents the target in the context photo being mistaken
    # for part of the background model. No fabricated car-free photo is used.
    panel(fig.add_axes([.187,.035,.23,.315]),'estimated_background')
    fig.add_artist(FancyArrowPatch((.426,.19),(.495,.415),
        connectionstyle='angle,angleA=0,angleB=90,rad=8',transform=fig.transFigure,
        arrowstyle='-|>',mutation_scale=20,lw=1.8,color='#2d659b'))
    # Actual histories of the two winning tiles, shown without text or numbers.
    columns=[int(np.flatnonzero(result['tile_id']==t['tile_id'])[0]) for t in evidence['candidate']['tiles']]
    lo=max(0,int(np.searchsorted(result['time_s'],result['time_s'][index]-.15)))
    trace=fig.add_axes([.724,.075,.23,.23])
    times=(result['time_s'][lo:index+1]-result['time_s'][index])*1000
    trace.plot(times,result['state_s'][lo:index+1,columns[0]],color='#c45800',lw=2)
    trace.plot(times,result['state_s'][lo:index+1,columns[1]],color='#edaa5d',lw=2)
    trace.axhline(values['threshold_s'],color='#697986',ls='--',lw=1)
    trace.plot([0],[result['state_s'][index,columns].min()],'o',color='#c45800',ms=5)
    trace.set_xlim(times[0],max(1.,times[-1]));trace.set_ylim(bottom=0);trace.axis('off')
    trace.annotate('',xy=(1.,-.03),xytext=(0.,-.03),xycoords='axes fraction',
                   arrowprops=dict(arrowstyle='->',lw=1.3,color='#2d659b'))
    for extension in ('png','svg'):
        fig.savefig(output/f'detection_method_real.{extension}',dpi=240,facecolor='white')
    plt.close(fig)
    for name in limits:
        single=plt.figure(figsize=(8,8*h/w),facecolor='white')
        panel(single.add_axes([0,0,1,1]),name,name=='integrated_residual')
        single.savefig(output/f'{name}.png',dpi=160,facecolor='white')
        plt.close(single)
    Image.fromarray(rgb).save(output/'rgb_full.png')
    Image.fromarray(rgb[y:y+h,x:x+w]).save(output/'rgb_roi.png')
    Image.fromarray(common.astype(np.uint8)*255).save(output/'common_valid_mask.png')
    np.savez_compressed(output/'source_values.npz',**values,
        tile_id=result['tile_id'],time_s=result['time_s'][lo:index+1],
        pair_state_s=result['state_s'][lo:index+1][:,columns])
    return dict(roi=roi,display_max=limits,opacity_rule='0.74 * clip(value/display_max, 0, 1)',
                density_scale_shared=True,integral_color_saturates_at_threshold=True,
                trace_tile_ids=[t['tile_id'] for t in evidence['candidate']['tiles']],
                trace_start_s=float(result['time_s'][lo]))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',default='test_05')
    parser.add_argument('--bundle',type=Path)
    parser.add_argument('--frozen-dir',type=Path,default=FROZEN)
    parser.add_argument('--camchain',type=Path,default=CALIBRATION/'config/calibrations/rc_popout_default/kalibr-camchain.yaml')
    parser.add_argument('--preflight',action='store_true')
    parser.add_argument('--debug',action='store_true')
    args=parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        bundle=args.bundle or args.record_root.parent/'analysis/development_debug_bundle01.zip'
        evidence=load_evidence(bundle,args.frozen_dir,args.session)
        sources=validate_sources(args.record_root,args.camchain,evidence)
        print(f"{args.session}: real RGB + measured 32px EVS tiles / candidate={evidence['candidate']['candidate_recording_s']:.6f}s",flush=True)
        if args.preflight:
            print('Preflight passed: archive/model/sync/calibration checked; no RGB decoded and no output written.')
            return 0
        # Fail before replay/decode if optional plotting dependencies are absent.
        for module in ('matplotlib', 'PIL', 'cv2', 'yaml', 'rosbags'):
            importlib.import_module(module)
        result,index=replay(evidence)
        print('Frozen candidate reproduced; reading native RGB from rosbag...',flush=True)
        rgb,common,timing=extract_rgb(sources,evidence,result,index)
        args.output.mkdir(parents=True)
        appearance=draw_figure(args.output,rgb,common,evidence,result,index)
        report=dict(status='complete',session=args.session,subset='development',
            purpose='Method illustration from measured data; not an additional evaluation result',
            source_mode='native_rosbag_rgb_and_archived_raw_event_tiles',timing=timing,
            candidate=evidence['candidate'],appearance=appearance,
            bundle_sha256=digest(bundle),frozen_manifest_sha256=evidence['frozen_manifest_sha256'],
            source_paths={k:str(v) for k,v in sources.items()},
            annotation_sha256=digest(sources['annotation']),time_sync_sha256=digest(sources['time_sync']),
            camchain_sha256=digest(sources['camchain']),bag_metadata_sha256=digest(sources['bag']/'metadata.yaml'),
            mcap_sizes={p.name:p.stat().st_size for p in sources['bag'].glob('*.mcap')},
            renderer_sha256=digest(Path(__file__)),
            outputs_sha256={p.name:digest(p) for p in args.output.iterdir() if p.is_file()},
            limitations=['RGB is the latest acquired past frame, not interpolated to EVS time.',
                         'Colors encode measured/estimated tile values, not cleaned pixel-level events.',
                         'The background inset is a tile-only model estimate at this time.',
                         'Density, standardized residual, and integral use different color scales.',
                         'The common ROI is shown unchanged. Rotation-only projection retains parallax.',
                         'Full MCAP contents and live RAW were not rehashed or redecoded.'])
        write_json(args.output/'summary.json',report)
        title=html.escape(args.session)
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Measured detection figure</title>'
            '<style>body{font:18px sans-serif;margin:32px;color:#24374b}img{width:100%;max-width:1600px}</style>'
            f'<h1>{title}：実測データによる検知方法の説明</h1><img src="detection_method_real.png">'
            '<p>左から活動密度・背景との差・時間積算と候補タイル。下段左はその時刻の推定背景、右は候補2区画の積算履歴。</p>'
            f'<p>EVS時刻 {timing["evs_relative_s"]:.6f} s。RGBはその {timing["rgb_age_ms"]:.3f} ms前に取得したフレーム。</p>'
            '<p>図内は文字なし。上段は同一のRGBに各段階の数値を重ねた。下段の背景推定は区画ごとの数値のみを表示する。</p>'
            '<p>調整用記録の方法説明であり、新しい検出成績ではない。数値・表示尺度・出典は <a href="summary.json">summary.json</a>。</p>',encoding='utf-8')
        print(f"Saved: {args.output/'detection_method_real.png'} / RGB age={timing['rgb_age_ms']:.3f} ms")
        return 0
    except (OSError,ValueError,KeyError,RuntimeError,ImportError) as exc:
        if args.debug:
            raise
        print(f'error: {exc}',file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
