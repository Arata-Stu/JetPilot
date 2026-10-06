"""Phase-separated, low-rank background maps for cached RGB/EVS grid activity.

Only the negative development recording fits the models and alarm threshold.
Prediction takes activity, geometry and frozen models, never onset/drive labels.
This is statistical background fitting, not a neural network or a vehicle classifier.
"""
import math

import numpy as np

from rc_popout_local_detection import geometry, validate_data


ALGORITHM = 'phase_background_pca_grid_residual_v1'
DEFAULTS = dict(rank=3, fit_step_s=.020, coefficient_limit=3.,
                residual_quantile=.95, decay_tau_s=.050, drift_k=1., z_clip=10.,
                threshold_margin=1.10, threshold_floor_s=.001, release_ratio=.5,
                rgb_max_gap_s=.100, evs_max_gap_s=.003, onset_guard_s=.030)


def validate_settings(settings):
    if set(settings) != set(DEFAULTS):
        raise ValueError('unexpected background settings')
    for k,v in settings.items():
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'invalid positive setting: {k}')
    if (not isinstance(settings['rank'],int) or settings['rank'] > 20
            or settings['residual_quantile'] >= 1 or settings['release_ratio'] >= 1
            or settings['threshold_margin'] <= 1 or settings['drift_k'] >= settings['z_clip']):
        raise ValueError('invalid rank, quantile, threshold margin or integration settings')


def features(data):
    # The whole spatial vector preserves location and co-variation. The square
    # root limits the dominance of very active tiles; no division by near-zero
    # total activity is used. RGB and EVS use exactly the same transform.
    return np.sqrt(data['counts']/data['valid_pixels'][None,:])


def fit_indices(data, phase, settings, sensor):
    """Last fully contained observation in each fixed-duration command-phase bin."""
    if not phase['complete'] or not phase['command_coverage']:
        raise ValueError('background fitting needs complete observed command phases')
    bins = {}
    t, start = data['time_s'], data['support_start_s']
    for segment,(a,b) in enumerate(phase['intervals']):
        valid = np.flatnonzero((start >= a) & (t < b)
                              & (t-start <= settings[f'{sensor}_max_gap_s']+1e-9))
        for i in valid:
            bins[segment, int((t[i]-a)/settings['fit_step_s'])] = int(i)
    indexes = np.array(sorted(bins.values()),dtype=int)
    if len(indexes) < max(10,settings['rank']+2):
        raise ValueError('too few background phase samples')
    return indexes


def reconstruct(x, model):
    center, basis, limit = (np.asarray(model[k],dtype=float) for k in ('center','basis','limit'))
    if not len(basis):
        return np.broadcast_to(center,x.shape).copy()
    coefficient = (x-center) @ basis.T
    coefficient = np.clip(coefficient,-limit,limit)
    return np.maximum(0.,center+coefficient @ basis)


def fit_background(data, meta, phases, sensor, settings):
    validate_settings(settings)
    validate_data(data,meta,sensor)
    x = features(data)
    # A one-event/changed-pixel floor accounts for partial valid tiles too.
    quantum = np.sqrt(1./data['valid_pixels'])
    models = []
    for name in ('pre_drive','drive'):
        matches = [p for p in phases if p['phase']==name]
        if len(matches) != 1:
            raise ValueError(f'expected one {name} fitting phase')
        indexes = fit_indices(data,matches[0],settings,sensor)
        train = x[indexes]
        center = train.mean(axis=0)
        _,s,v = np.linalg.svd(train-center,full_matrices=False)
        rank = min(settings['rank'],int(np.count_nonzero(s > max(1e-12,float(s[0])*1e-10))))
        basis = v[:rank]
        limit = settings['coefficient_limit']*s[:rank]/math.sqrt(len(train)-1)
        model = dict(phase=name,center=center.tolist(),basis=basis.tolist(),limit=limit.tolist())
        fitted = reconstruct(train,model)
        scale = np.maximum(quantum,np.quantile(np.abs(train-fitted),settings['residual_quantile'],axis=0))
        model.update(scale=scale.tolist(),fit_indexes=indexes.tolist(),fit_samples=len(indexes),
            fit_time_s=[float(data['time_s'][indexes[0]]),float(data['time_s'][indexes[-1]])],
            phase_intervals=matches[0]['intervals'],effective_rank=rank,
            training_rmse=float(np.sqrt(np.mean((train-fitted)**2))),
            explained_variance_fraction=float(np.sum(s[:rank]**2)/np.sum(s**2)) if np.any(s) else None)
        models.append(model)
    return dict(sensor=sensor,tile_id=data['tile_id'].tolist(),valid_pixels=data['valid_pixels'].tolist(),
                models=models,fit_scope='negative development pre_drive and drive; equal-time bins per phase')


def validate_model(background,data,sensor):
    if (background['sensor'] != sensor or not np.array_equal(background['tile_id'],data['tile_id'])
            or not np.array_equal(background['valid_pixels'],data['valid_pixels'])):
        raise ValueError('background sensor/geometry mismatch')
    k=len(data['tile_id'])
    if [m['phase'] for m in background['models']] != ['pre_drive','drive']:
        raise ValueError('invalid background phases')
    for m in background['models']:
        center,scale,basis,limit=(np.asarray(m[n],dtype=float) for n in ('center','scale','basis','limit'))
        if (center.shape != (k,) or scale.shape != (k,) or limit.ndim != 1
                or (basis.shape != (len(limit),k) if len(limit) else basis.shape != (0,))
                or any(not np.all(np.isfinite(a)) for a in (center,scale,basis,limit))
                or np.any(center < 0) or np.any(scale <= 0) or np.any(limit < 0)):
            raise ValueError('invalid fitted background arrays')


def score_maps(data,meta,sensor,settings,background,tile_px):
    """Causal fixed-model inference, preserving native sensor sample cadence."""
    validate_settings(settings); validate_data(data,meta,sensor); validate_model(background,data,sensor)
    pairs,_=geometry(meta,tile_px)
    x=features(data)
    # Every sample is independent here. Model selection never reads future
    # samples or the command phase of the scene being scored.
    predictions=np.stack([reconstruct(x,m) for m in background['models']])
    losses=np.mean((predictions-x[None,:,:])**2,axis=2)
    chosen=losses.argmin(axis=0)
    predicted=predictions[chosen,np.arange(len(x))]
    scale=np.array([m['scale'] for m in background['models']])[chosen]
    z=np.clip((x-predicted)/scale,0.,settings['z_clip'])
    t,start,interval=(data[k] for k in ('time_s','support_start_s','interval'))
    state=np.zeros(x.shape[1])
    scores=np.zeros(len(x)); states=np.zeros(x.shape,np.float32)
    winner=np.zeros(len(x),dtype=int); ready=np.ones(len(x),bool); reset=np.zeros(len(x),bool)
    max_gap=settings[f'{sensor}_max_gap_s']
    observed=np.zeros(len(x))
    for i in range(len(x)):
        dt=0. if not i else float(t[i]-t[i-1])
        ready[i]=t[i]-start[i] <= max_gap+1e-9
        reset[i]=(not i or interval[i] != interval[i-1] or dt > max_gap+1e-9
                  or not ready[i] or not ready[i-1])
        if reset[i]:
            state[:]=0.
        else:
            tau=settings['decay_tau_s']
            state=np.maximum(0.,math.exp(-dt/tau)*state-tau*math.expm1(-dt/tau)*(z[i]-settings['drift_k']))
            observed[i]=dt
        values=np.minimum(state[pairs[:,0]],state[pairs[:,1]])
        winner[i]=int(values.argmax()); scores[i]=values[winner[i]]
        states[i]=state
    return dict(time_s=t,support_start_s=start,interval=interval,tile_id=data['tile_id'],
        score=scores,ready=ready,reset=reset,observed_step_s=observed,
        winner_pair=pairs[winner],background_index=chosen,
        background_rmse=np.sqrt(losses[chosen,np.arange(len(x))]),
        predicted_density=(predicted**2).astype(np.float32),residual_z=z.astype(np.float32),
        state_s=states)


def calibrate_threshold(negative,settings):
    """All valid negative intervals, including coast-down and waiting, contribute."""
    values=negative['score'][negative['ready']]
    if not len(values) or not np.all(np.isfinite(values)):
        raise ValueError('no finite negative scores for calibration')
    peak=float(values.max())
    return dict(negative_max=peak,threshold_s=max(settings['threshold_floor_s'],
                                                peak*settings['threshold_margin']),
                rule='max(threshold_floor_s, entire_negative_max * threshold_margin)')


def alarm_episodes(result,threshold,release_ratio):
    if not math.isfinite(threshold) or threshold <= 0 or not 0 <= release_ratio < 1:
        raise ValueError('invalid alarm threshold/release')
    active=np.zeros(len(result['score']),bool); alarm=active.copy()
    episodes=[]; current=None
    def close(i,reason):
        nonlocal current
        if current is not None:
            current.update(end_time_s=float(result['time_s'][i]),end_reason=reason)
            episodes.append(current); current=None
    for i,value in enumerate(result['score']):
        if result['reset'][i] or not result['ready'][i]:
            close(max(0,i-1),'gap_or_invalid')
        if not result['ready'][i]:
            continue
        if current is None and value >= threshold:
            a,b=result['winner_pair'][i]
            current=dict(start_time_s=float(result['time_s'][i]),start_index=i,
                pair_tile_ids_at_start=[int(result['tile_id'][a]),int(result['tile_id'][b])])
            alarm[i]=True
        if value <= threshold*release_ratio:
            close(i,'below_release')
        active[i]=current is not None
    close(len(active)-1,'end_of_recording')
    return episodes,alarm,active
