#!/usr/bin/env python3
"""Offline candidate analysis. Never publishes TF or changes source samples."""
import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    if not np.isfinite(result).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < .999:
        raise ValueError('Invalid rigid transform')
    return result


def quaternion_rotation(q):
    q = np.array(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .001:
        raise ValueError('Invalid quaternion')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def difference(a,b):
    angle = np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1)))
    return float(np.linalg.norm(a[:3,3]-b[:3,3])*1000), float(angle)


def solve(samples, method):
    g = [s['g'] for s in samples]; c = [s['c'] for s in samples]
    r,t = cv2.calibrateHandEye([x[:3,:3] for x in g], [x[:3,3] for x in g],
                              [x[:3,:3] for x in c], [x[:3,3] for x in c], method=method)
    return transform(r,t)


def center(poses):
    u,_,v = np.linalg.svd(sum(p[:3,:3] for p in poses))
    r = u@np.diag([1,1,np.linalg.det(u@v)])@v
    return transform(r,np.mean([p[:3,3] for p in poses],axis=0))


def analyze(root):
    samples=[]; rejected=[]; signature=None
    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        try:
            d=json.loads((folder/'sample.json').read_text())
            image=cv2.imread(str(folder/'image.png'))
            if image is None or image.shape[:2] != (d['height'],d['width']):
                raise ValueError('Missing/invalid image')
            sig=[d[k] for k in ('camera_frame','base_frame','tool_frame','columns','rows','square_size_m','camera_matrix','distortion','distortion_model','width','height')]
            if signature is not None and sig != signature:
                raise ValueError('Inconsistent metadata')
            if (d['columns'],d['rows'],d['square_size_m']) != (8,5,.02):
                raise ValueError('Unexpected board')
            g=transform(quaternion_rotation(d['base_T_tool']['quaternion_xyzw']),d['base_T_tool']['translation_m'])
            c=transform(cv2.Rodrigues(np.array(d['camera_T_board']['rvec'],dtype=float))[0],d['camera_T_board']['translation_m'])
            points=np.zeros((40,3),np.float32); points[:,:2]=np.mgrid[:8,:5].T.reshape(-1,2)*.02
            corners=np.array(d['corners_px']); k=np.array(d['camera_matrix']).reshape(3,3)
            if corners.shape != (40,2) or not np.isfinite(corners).all():
                raise ValueError('Invalid corners')
            projected,_=cv2.projectPoints(points,np.array(d['camera_T_board']['rvec']),c[:3,3],k,np.array(d['distortion']))
            error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-corners)**2,axis=1))))
            if not np.isfinite(error) or error>.5 or np.min((points@c[:3,:3].T+c[:3,3])[:,2])<=0:
                raise ValueError('Invalid projection')
            signature=sig
            samples.append(dict(id=folder.name,g=g,c=c,error=error))
        except (OSError,ValueError,KeyError,cv2.error) as exc:
            rejected.append(dict(id=folder.name,reason=str(exc)))
    # Greedy representative selection, best reprojection first. No files removed.
    selected=[]; duplicates=[]
    for s in sorted(samples,key=lambda s:s['error']):
        matches=[p for p in selected if all(v<limit for v,limit in zip(difference(s['g'],p['g']),(5.,2.)))]
        if matches:
            duplicates.append(dict(id=s['id'],representative=matches[0]['id'],difference_mm_deg=difference(s['g'],matches[0]['g'])))
        else:
            selected.append(s)
    selected.sort(key=lambda s:s['id'])
    if len(selected)<6:
        raise ValueError('Fewer than six distinct valid poses')
    vectors=[cv2.Rodrigues(a['g'][:3,:3].T@b['g'][:3,:3])[0].ravel() for a,b in itertools.combinations(selected,2)]
    sv=np.linalg.svd(np.array(vectors),compute_uv=False)
    report=dict(status='NOT_VALIDATED_DO_NOT_APPLY',source=str(root),valid_count=len(samples),
                selected=[s['id'] for s in selected],rejected=rejected,duplicates=duplicates,
                dedup_threshold_mm_deg=[5,2],rotation_excitation_singular_values=sv.tolist(),
                transform_definition='tool0_T_camera_color_optical_frame (NOT camera_link)',methods={})
    for name in ('TSAI','PARK','HORAUD','ANDREFF','DANIILIDIS'):
        try:
            method=getattr(cv2,'CALIB_HAND_EYE_'+name)
            x=solve(selected,method)
            boards=[s['g']@x@s['c'] for s in selected]; mean=center(boards)
            errors=[difference(p,mean) for p in boards]
            loo=[]
            for i,s in enumerate(selected):
                training=selected[:i]+selected[i+1:]
                xi=solve(training,method)
                reference=center([p['g']@xi@p['c'] for p in training])
                loo.append(difference(s['g']@xi@s['c'],reference))
            report['methods'][name]=dict(tool_T_camera=x.tolist(),
                per_sample=[dict(id=s['id'],fit_mm_deg=e,leave_one_out_mm_deg=l) for s,e,l in zip(selected,errors,loo)],
                fit_rms_mm_deg=np.sqrt(np.mean(np.square(errors),axis=0)).tolist(),
                fit_max_mm_deg=np.max(errors,axis=0).tolist(),
                loo_rms_mm_deg=np.sqrt(np.mean(np.square(loo),axis=0)).tolist(),
                loo_max_mm_deg=np.max(loo,axis=0).tolist())
        except (ValueError,cv2.error,np.linalg.LinAlgError) as exc:
            report['methods'][name]=dict(error=str(exc))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    report=analyze(args.session)
    with args.report.open('x') as f:
        json.dump(report,f,indent=2,allow_nan=False)
    print('Valid:',report['valid_count'],'Selected:',len(report['selected']),'Duplicates:',len(report['duplicates']))
    for name,result in report['methods'].items():
        print(name, 'fit RMS [mm,deg]:',result.get('fit_rms_mm_deg'), 'LOO RMS:',result.get('loo_rms_mm_deg'),result.get('error',''))
    print('NOT VALIDATED. No TF/URDF modified. Report:',args.report)
