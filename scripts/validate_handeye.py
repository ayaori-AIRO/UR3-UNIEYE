#!/usr/bin/env python3
"""Evaluate NEW observations using a frozen candidate; never refit/apply TF."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from analyze_handeye import transform, quaternion_rotation, center, difference


FIELDS = ('camera_frame', 'base_frame', 'tool_frame', 'columns', 'rows',
          'square_size_m', 'camera_matrix', 'distortion', 'distortion_model',
          'width', 'height')


def load(folder, expected=None):
    d = json.loads((folder / 'sample.json').read_text())
    signature = [d[k] for k in FIELDS]
    if expected is not None and signature != expected:
        raise ValueError(f'{folder.name}: camera/board metadata changed')
    image = cv2.imread(str(folder / 'image.png'))
    if image is None or image.shape[:2] != (d['height'], d['width']):
        raise ValueError(f'{folder.name}: invalid image')
    if (d['columns'], d['rows'], d['square_size_m']) != (8, 5, .02):
        raise ValueError('Expected 8x5 inner corners and 20mm squares')
    g = transform(quaternion_rotation(d['base_T_tool']['quaternion_xyzw']),
                  d['base_T_tool']['translation_m'])
    rv = np.array(d['camera_T_board']['rvec'], dtype=float)
    c = transform(cv2.Rodrigues(rv)[0], d['camera_T_board']['translation_m'])
    points = np.zeros((40, 3), np.float32)
    points[:, :2] = np.mgrid[:8, :5].T.reshape(-1, 2) * .02
    corners = np.array(d['corners_px'])
    if corners.shape != (40, 2) or not np.isfinite(corners).all():
        raise ValueError('Invalid corners')
    projected, _ = cv2.projectPoints(points, rv, c[:3, 3],
                                     np.array(d['camera_matrix']).reshape(3, 3),
                                     np.array(d['distortion']))
    error = np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-corners)**2, axis=1)))
    if not np.isfinite(error) or error > .5 or np.any((points @ c[:3,:3].T+c[:3,3])[:,2] <= 0):
        raise ValueError('Invalid reprojection/depth')
    return g, c, signature


def validate(report_path, session):
    report = json.loads(report_path.read_text())
    source = Path(report['source'])
    if session.resolve() == source.resolve():
        raise ValueError('Independent validation requires a NEW session')
    raw = np.array(report['methods']['PARK']['tool_T_camera'])
    if raw.shape != (4, 4) or not np.allclose(raw[3], [0,0,0,1]):
        raise ValueError('Invalid candidate matrix')
    x = transform(raw[:3,:3], raw[:3,3])
    training = []
    signature = None
    for name in report['selected']:
        g, c, signature = load(source / name, signature)
        training.append((g,c))
    reference = center([g @ x @ c for g,c in training])
    observations = []; poses = []
    for folder in sorted(session.iterdir()):
        if not folder.is_dir():
            continue
        if (source / folder.name).exists():
            raise ValueError('Training timestamp reused in validation')
        g, c, _ = load(folder, signature)
        near = any(mm < 5 and deg < 2 for mm,deg in
                   (difference(g,p) for p in [a for a,b in training]+poses))
        poses.append(g)
        observations.append(dict(id=folder.name,
                                 similar_to_training_or_previous=near,
                                 residual_mm_deg=difference(g @ x @ c, reference)))
    if not observations:
        raise ValueError('No validation samples')
    errors = np.array([s['residual_mm_deg'] for s in observations])
    return dict(status='REQUIRES_REVIEW_DO_NOT_APPLY', method='PARK_FROZEN_NO_REFIT',
                candidate_report=str(report_path), session=str(session),
                assumption='Board and camera mount unchanged since training',
                sample_count=len(observations), distinct_new_pose_count=sum(not s['similar_to_training_or_previous'] for s in observations),
                rms_mm_deg=np.sqrt(np.mean(errors**2,axis=0)).tolist(),
                max_mm_deg=errors.max(axis=0).tolist(), samples=observations)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('new_session', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    result = validate(args.candidate, args.new_session)
    with args.report.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
