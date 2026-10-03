"""Retrospective disc-model audit of a fixed small Argoverse 2 trajectory subset."""
import argparse
from collections import Counter
import hashlib
import itertools
import json
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
# Fixed engineering approximations, frozen before validation; not fitted sizes.
RADII={'vehicle':2.0,'bus':3.0,'pedestrian':.4,'motorcyclist':.7,'cyclist':.7,
       'static':.5,'background':.5,'construction':.5,'riderless_bicycle':.7,'unknown':.5}


def independent_pair(a,b,radius,step=10):
    """Closest approach on aligned linear intervals; independent of quadratic roots."""
    relative=a-b
    origin=relative[:-1]
    delta=np.diff(relative,axis=0)
    denominator=np.sum(delta*delta,axis=1)
    alpha=np.zeros(len(delta))
    np.divide(-np.sum(origin*delta,axis=1),denominator,out=alpha,where=denominator>0)
    alpha=np.clip(alpha,0,1)
    distances=np.sum((origin+alpha[:,None]*delta)**2,axis=1)
    hit=distances<=radius*radius
    sample_indices=sorted(set(range(0,len(relative),step))|{len(relative)-1})
    sampled=bool(np.any(np.sum(relative[sample_indices]**2,axis=1)<=radius*radius))
    if not np.any(hit):
        return False,sampled,None
    first=int(np.flatnonzero(hit)[0])
    return True,sampled,{'interval':first,'nearest_fraction':float(alpha[first]),
                         'minimum_distance':float(np.sqrt(distances[first]))}


def slice_tracks(df,begin,end):
    selected=df[(df.timestep>=begin)&(df.timestep<=end)]
    tracks=[]
    excluded=Counter()
    for source_id,group in selected.groupby('track_id',sort=True):
        group=group.sort_values('timestep')
        if group.timestep.tolist()!=list(range(begin,end+1)):
            excluded['missing_or_duplicate_frames']+=1
            continue
        points=group[['position_x','position_y']].to_numpy(dtype=float)
        if not np.isfinite(points).all():
            excluded['non_finite']+=1
            continue
        category=str(group.object_type.iloc[0])
        tracks.append({'id':len(tracks)+1,'source_id':str(source_id),'type':category,
                       'radius':RADII.get(category,.5),'points':points,
                       'speed':float(np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))/.1)})
    if len(tracks)>128:
        raise ValueError('scenario exceeds bounded evaluator limit')
    return tracks,dict(excluded)


def write_csv(path,tracks):
    with path.open('w',encoding='utf8') as file:
        for track in tracks:
            for i,(x,y) in enumerate(track['points']):
                file.write(f"{track['id']},{track['radius']},{i/10:.1f},{x:.17g},{y:.17g}\n")


def source_manifest():
    sources=json.loads((ROOT/'evaluation'/'sources.json').read_text(encoding='utf8'))
    for split in ('train','val'):
        listing=ET.parse(ROOT/'evaluation'/f'{split}-selection-listing.xml').getroot()
        keys=sorted(n.text for n in listing.findall('.//{*}Key') if n.text.endswith('.parquet'))
        if keys[:6]!=[s['key'] for s in sources if s['split']==split]:
            raise RuntimeError('fixed source selection differs from retained inventory')
    for source in sources:
        path=ROOT/'data'/f"{source['split']}-{Path(source['key']).name}"
        if path.stat().st_size!=source['bytes'] or hashlib.sha256(path.read_bytes()).hexdigest()!=source['sha256']:
            raise RuntimeError('trajectory input checksum mismatch')
    return sources


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--binary',default='build/scenario_eval.exe' if sys.platform=='win32' else 'build/scenario_eval')
    parser.add_argument('--output',default='docs/evidence/argoverse')
    args=parser.parse_args()
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    sources=source_manifest()
    rows,replays=[],[]
    stress=[]
    # Synthetic all-overlapping segments deliberately remove spatial pruning.
    with tempfile.TemporaryDirectory() as work:
        csv=Path(work)/'dense.csv'
        csv.write_text(''.join(f'{i},1,0,0,0\n{i},1,1,0,0\n'for i in range(1,65)),encoding='utf8')
        for repeat in range(3):
            child=subprocess.run([str(Path(args.binary).resolve()),str(csv),'0'],capture_output=True,text=True,timeout=30)
            if child.returncode!=0:
                raise RuntimeError(child.stderr)
            native=[json.loads(line)for line in child.stdout.splitlines()]
            assert len(native)==64 and all(len(r['contacts'])==63 for r in native)
            assert sum(r['narrow_checks']for r in native)==sum(r['brute_checks']for r in native)==64*63
            stress.append({'kind':'synthetic_dense_worst_case','repeat':repeat,'tracks':64,
                           'narrow_checks':4032,'brute_checks':4032,'contact_actor_pairs':2016,
                           'audit_ms':sum(r['audit_ms']for r in native),'brute_ms':sum(r['brute_ms']for r in native)})
    for source in sources:
        path=ROOT/'data'/f"{source['split']}-{Path(source['key']).name}"
        df=pd.read_parquet(path)
        scenario=str(df.scenario_id.iloc[0])
        for phase,begin,end in [('observed',0,49),('future',50,109)]:
            tracks,excluded=slice_tracks(df,begin,end)
            if len(tracks)<2:
                raise ValueError('subset phase has too few complete tracks')
            with tempfile.TemporaryDirectory() as work:
                csv=Path(work)/'tracks.csv'
                write_csv(csv,tracks)
                for margin in (0.,.5,1.):
                    expected={t['id']:set() for t in tracks}
                    sampled={10:set(),1:set()}
                    continuous=set()
                    misses=[]
                    for a,b in itertools.combinations(tracks,2):
                        radius=a['radius']+b['radius']+margin
                        contact,sampled_coarse,replay=independent_pair(a['points'],b['points'],radius,10)
                        _,sampled_fine,_=independent_pair(a['points'],b['points'],radius,1)
                        pair=(a['id'],b['id'])
                        if sampled_coarse: sampled[10].add(pair)
                        if sampled_fine: sampled[1].add(pair)
                        if contact:
                            continuous.add(pair)
                            expected[a['id']].add(b['id'])
                            expected[b['id']].add(a['id'])
                            if not sampled_coarse:
                                replays.append({'scenario':scenario,'split':source['split'],'phase':phase,'margin':margin,
                                                'actors':[a['source_id'],b['source_id']],
                                                'radii':[a['radius'],b['radius']],**replay,
                                                'a_interval_points':a['points'][replay['interval']:replay['interval']+2].tolist(),
                                                'b_interval_points':b['points'][replay['interval']:replay['interval']+2].tolist()})
                    for repeat in range(3):
                        child=subprocess.run([str(Path(args.binary).resolve()),str(csv),str(margin)],capture_output=True,text=True,timeout=30)
                        if child.returncode!=0:
                            raise RuntimeError(child.stderr)
                        native=[json.loads(line) for line in child.stdout.splitlines()]
                        assert len(native)==len(tracks)
                        for result in native:
                            assert set(result['contacts'])==expected[result['track']],(scenario,phase,result)
                        rows.append({'scenario':scenario,'split':source['split'],'city':str(df.city.iloc[0]),
                                     'phase':phase,'first_frame':begin,'last_frame':end,'margin':margin,'repeat':repeat,
                                     'tracks':len(tracks),'excluded':excluded,'actor_types':dict(Counter(t['type']for t in tracks)),
                                     'moving_tracks':sum(t['speed']>1. for t in tracks),'max_speed_m_s':max(t['speed']for t in tracks),
                                     'dense_slice':len(tracks)>=20,'continuous_actor_pairs':len(continuous),
                                     'sampled_1s_actor_pairs':len(sampled[10]),'sampled_01s_actor_pairs':len(sampled[1]),
                                     'missed_by_1s':len(continuous-sampled[10]),'missed_by_01s':len(continuous-sampled[1]),
                                     'narrow_checks':sum(r['narrow_checks']for r in native),'brute_checks':sum(r['brute_checks']for r in native),
                                     'build_ms':sum(r['build_ms']for r in native),'audit_ms':sum(r['audit_ms']for r in native),
                                     'brute_ms':sum(r['brute_ms']for r in native),'native':native})
    (output/'trials.jsonl').write_text(''.join(json.dumps(row,sort_keys=True)+'\n'for row in rows),encoding='utf8')
    (output/'missed-contact-replays.jsonl').write_text(''.join(json.dumps(row,sort_keys=True)+'\n'for row in replays),encoding='utf8')
    (output/'dense-stress.jsonl').write_text(''.join(json.dumps(row,sort_keys=True)+'\n'for row in stress),encoding='utf8')
    summary=[]
    for split in ('train','val'):
        for margin in (0.,.5,1.):
            # Counts use one repetition; timings use median of three repeated executions.
            cases=[r for r in rows if r['split']==split and r['margin']==margin and r['repeat']==0]
            timings=[sum(r['audit_ms']for r in rows if r['split']==split and r['margin']==margin and r['repeat']==repeat)for repeat in range(3)]
            summary.append({'split':split,'margin':margin,'independent_scenarios':6,'phase_slices':len(cases),
                            'actor_queries':sum(r['tracks']for r in cases),
                            **{key:sum(r[key]for r in cases)for key in ('continuous_actor_pairs','missed_by_1s','missed_by_01s','narrow_checks','brute_checks')},
                            'median_audit_ms':float(np.median(timings))})
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
    manifest={'sources':sources,'selection':'Fixed first-six parquet list from retained lexicographic public S3 inventory for each official split; no preregistration claim.',
              'license':'Argoverse 2 data: CC BY-NC-SA 4.0; code: MIT',
              'terms_url':'https://www.argoverse.org/about.html#terms-of-use',
              'attribution':'Argo AI; Wilson et al., Argoverse 2, NeurIPS Datasets and Benchmarks 2021.',
              'radii':RADII,'margins_m':[0,.5,1],'sampled_steps_s':[.1,1.],
              'python':sys.version,'platform':platform.platform(),'processor':platform.processor(),'numpy':np.__version__,'pandas':pd.__version__,
              'command':'python evaluation/scenarios.py','trials':len(rows),
              'synthetic_stress_trials':len(stress),
              'binary_sha256':hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
              'source_hash_format':'sha256-lf-normalized-text',
              'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()for p in [Path(__file__),ROOT/'src'/'scenario_eval.cpp',ROOT/'evaluation'/'sources.json',ROOT/'evaluation'/'train-selection-listing.xml',ROOT/'evaluation'/'val-selection-listing.xml']+list((ROOT/'include'/'motion_guard').glob('*.hpp'))},
              'artifact_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in output.glob('*.json*') if p.name!='manifest.json'},
              'limits':['Retrospective recorded paths with linear interpolation; not forecast or control evaluation.',
                        'Disc radii are assumed proxies, not ground-truth vehicle dimensions or collision labels.',
                        'Incomplete tracks excluded separately in each phase; no extrapolation or gap filling.',
                        'Official validation subset held separate; no learned preprocessing or model training.',
                        'Only six validation scenarios, lexicographically selected, not representative of the complete dataset.',
                        'Reported missed contacts are contacts under this model between recorded samples, not missed real collisions.',
                        'No safety certification, closed-loop simulation, uncertainty calibration, or deployment claims.']}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    print(json.dumps({'scenarios':len(sources),'trials':len(rows),'parity':'BVH intervals == brute; actor sets == independent closest-approach oracle','replays':len(replays)}))


if __name__=='__main__':
    main()
