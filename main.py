"""Detección geométrica por fotograma con LiDAR de infraestructura, sin tracking."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

SEQUENCES = ('20241126_0024_crossing1_09', '20241126_0008_crossing1_01', '20241127_0000_crossing1_00')
SENSORS = tuple(f'crossing1_{i}_lidar' for i in (11, 12, 31, 32))


def transform(points, matrix):
    # Convención comprobada en LidarData.point_cloud_in_global_coordinates del devkit.
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Calibración inválida')
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def load_frame(folder, calibration, sync, index):
    clouds, sources = [], {}
    for sensor in SENSORS:
        filename = sync[sensor][index]
        if not filename or Path(filename).name != filename:
            raise ValueError(f'Archivo inválido: {sensor}, frame {index}')
        with np.load(folder / sensor / filename, allow_pickle=False) as data:
            xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        clouds.append(transform(xyz, calibration[sensor]['extrinsics']['gTl']))
        sources[sensor] = filename
    return np.concatenate(clouds), sources


def voxelize(points, size):
    if not len(points):
        return points
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def ground_plane(points):
    """RANSAC de z=ax+by+c sobre mínimos de celdas XY de 2 m."""
    if len(points) < 30:
        raise ValueError('No hay suficientes puntos para estimar el suelo')
    _, groups = np.unique(np.floor(points[:, :2] / 2).astype(int), axis=0, return_inverse=True)
    order = np.lexsort((points[:, 2], groups))
    low = points[order[np.r_[True, np.diff(groups[order]) != 0]]]
    if len(low) < 10:
        raise ValueError('Cobertura insuficiente para estimar el suelo')
    design = np.column_stack((low[:, :2], np.ones(len(low))))
    rng = np.random.default_rng(7)
    best = np.zeros(len(low), dtype=bool)
    for _ in range(150):
        ids = rng.choice(len(low), 3, replace=False)
        if np.linalg.matrix_rank(design[ids]) != 3:
            continue
        model = np.linalg.solve(design[ids], low[ids, 2])
        if np.linalg.norm(model[:2]) > 0.15:
            continue
        inliers = np.abs(design @ model - low[:, 2]) < 0.18
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 10:
        raise ValueError('No se ha podido estimar el suelo')
    return np.linalg.lstsq(design[best], low[best, 2], rcond=None)[0]


def clusters(points, radius, min_points):
    """DBSCAN en XY con vecinos calculados bajo demanda."""
    if not len(points):
        return
    tree = cKDTree(points[:, :2])
    core = tree.query_ball_point(points[:, :2], radius, return_length=True) >= min_points
    labels = np.full(len(points), -1, dtype=int)
    cluster_id = 0
    for seed in np.flatnonzero(core):
        if labels[seed] >= 0:
            continue
        labels[seed] = cluster_id
        stack = [seed]
        while stack:
            current = stack.pop()
            neighbors = np.asarray(tree.query_ball_point(points[current, :2], radius))
            new = neighbors[labels[neighbors] < 0]
            labels[new] = cluster_id
            stack.extend(new[core[new]].tolist())
        yield points[labels == cluster_id]
        cluster_id += 1


def fit_box(points, plane):
    """Caja orientada por PCA, con base en el plano de suelo."""
    center = points[:, :2].mean(axis=0)
    _, axes = np.linalg.eigh(np.cov((points[:, :2] - center).T))
    yaw = float(np.arctan2(axes[1, -1], axes[0, -1]))
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    local = (points[:, :2] - center) @ rotation
    lo, hi = local.min(axis=0), local.max(axis=0)
    xy = center + ((lo + hi) / 2) @ rotation.T
    length, width = hi - lo
    if width > length:
        length, width = width, length
        yaw += np.pi / 2
    bottom = float(np.r_[xy, 1] @ plane)
    height = float(points[:, 2].max() - bottom)
    return {'center': [float(xy[0]), float(xy[1]), bottom + height / 2],
            'dimensions': [float(length), float(width), height],
            'yaw_rad': float((yaw + np.pi / 2) % np.pi - np.pi / 2),
            'num_points': len(points), 'class': 'vehicle_candidate'}


def detect(points, roi, voxel, eps, min_points):
    xmin, xmax, ymin, ymax = roi
    points = points[(points[:, 0] >= xmin) & (points[:, 0] <= xmax)
                    & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)]
    points = voxelize(points, voxel)
    plane = ground_plane(points)
    height = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
    foreground = points[(height >= 0.35) & (height <= 4.5)]
    boxes = []
    for group in clusters(foreground, eps, min_points):
        box = fit_box(group, plane)
        length, width, height = box['dimensions']
        if 1.8 <= length <= 18 and 0.8 <= width <= 3.5 and 0.7 <= height <= 4.5:
            boxes.append(box)
    return points, boxes, plane


def save_preview(points, boxes, roi, destination, title, road=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 10))
    sample = points[::max(1, len(points) // 60000)]
    ax.scatter(sample[:, 0], sample[:, 1], s=0.3, color='0.5', rasterized=True)
    if road is not None:
        from road import draw_road
        draw_road(ax, road)
    for index, box in enumerate(boxes, 1):
        length, width, _ = box['dimensions']
        angle = box['yaw_rad']
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]])
        corners = corners * [length / 2, width / 2] @ rotation.T + box['center'][:2]
        ax.plot(*corners.T, color='tab:red', linewidth=1)
        ax.text(*box['center'][:2], str(index), fontsize=8, color='darkred')
    ax.set(xlim=roi[:2], ylim=roi[2:], xlabel='X global (m)', ylabel='Y global (m)',
           title=f'{title}\nCandidatos a vehículo: {len(boxes)} · sin tracking', aspect='equal')
    fig.tight_layout()
    fig.savefig(destination, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data'))
    parser.add_argument('--output', type=Path, default=Path('results/improved'))
    parser.add_argument('--method', choices=('baseline', 'improved'), default='improved')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('detector_config.json'))
    parser.add_argument('--sequence', choices=SEQUENCES, action='append')
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--limit', type=int, default=None)
    parser.add_argument('--step', type=int, default=1)
    parser.add_argument('--roi', nargs=4, type=float, default=[-45, 45, -45, 45],
                        metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    parser.add_argument('--voxel', type=float, default=None)
    parser.add_argument('--eps', type=float, default=None)
    parser.add_argument('--min-points', type=int, default=None)
    parser.add_argument('--preview-every', type=int, default=20, help='0 desactiva PNG')
    args = parser.parse_args()
    from road import load_road, load_lane_segments
    from detector import DEFAULTS, detect_improved
    config = DEFAULTS.copy()
    if args.method == 'improved':
        config.update(json.loads(args.config.read_text(encoding='utf-8'))['parameters'])
    args.voxel = args.voxel if args.voxel is not None else (0.15 if args.method == 'improved' else 0.2)
    args.eps = args.eps if args.eps is not None else (config['eps'] if args.method == 'improved' else 0.7)
    args.min_points = args.min_points if args.min_points is not None else (config['min_points'] if args.method == 'improved' else 8)
    config.update(eps=args.eps, min_points=args.min_points)
    if args.start < 0 or args.step < 1 or (args.limit is not None and args.limit < 1):
        parser.error('start >= 0, step >= 1 y limit >= 1')
    if not np.isfinite([args.voxel, args.eps]).all() or args.voxel <= 0 or args.eps <= 0 or args.min_points < 3 or args.preview_every < 0:
        parser.error('voxel y eps > 0, min-points >= 3, preview-every >= 0')
    if not np.isfinite(args.roi).all() or args.roi[0] >= args.roi[1] or args.roi[2] >= args.roi[3]:
        parser.error('ROI inválida')
    road = load_road(args.data / 'crossings_lanelet2map.osm', args.roi) if args.method == 'improved' else None
    lanes = load_lane_segments(args.data / 'crossings_lanelet2map.osm') if args.method == 'improved' else None
    for name in args.sequence or SEQUENCES:
        folder = args.data / 'dataset' / name
        calibration = json.loads((folder / 'calibration.json').read_text())
        with (folder / 'timesync_info.csv').open(newline='') as stream:
            sync = {row[0]: row[1:] for row in csv.reader(stream)}
        total = len(sync['timestamp_ms'])
        if args.start >= total:
            parser.error(f'start fuera de rango para {name}: {total} fotogramas')
        indices = list(range(args.start, total, args.step))[:args.limit]
        output = args.output / name
        output.mkdir(parents=True, exist_ok=True)
        metadata = {'sensors': SENSORS, 'coordinate_frame': 'global', 'units': 'm',
                    'method': args.method, 'tracking': False, 'detector_config': config,
                    'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}}
        (output / 'run.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        if road is not None:
            from shapely.geometry import mapping
            (output / 'road_mask.geojson').write_text(json.dumps({'type': 'Feature', 'geometry': mapping(road),
                'properties': {'coordinates': 'global local UTM32, meters; not longitude/latitude', 'margin_m': 0.35}}))
        with (output / 'detections.jsonl').open('w', encoding='utf-8') as detections, (output / 'counts.csv').open('w', newline='', encoding='utf-8') as counts:
            writer = csv.writer(counts)
            writer.writerow(['frame_index', 'timestamp_ms', 'vehicle_candidate_count'])
            for number, index in enumerate(indices):
                points, sources = load_frame(folder, calibration, sync, index)
                if args.method == 'improved':
                    points, boxes, plane = detect_improved(points, args.roi, args.voxel, road, config, lanes)
                else:
                    points, boxes, plane = detect(points, args.roi, args.voxel, args.eps, args.min_points)
                timestamp = int(sync['timestamp_ms'][index])
                record = {'frame_index': index, 'timestamp_ms': timestamp, 'sources': sources,
                          'count': len(boxes), 'ground_plane_abc': plane.tolist(), 'boxes': boxes}
                detections.write(json.dumps(record) + '\n')
                writer.writerow([index, timestamp, len(boxes)])
                if args.preview_every and number % args.preview_every == 0:
                    save_preview(points, boxes, args.roi, output / f'bev_{index:04d}.png', f'{name} | {timestamp}', road)
                print(f'{name} [{number+1}/{len(indices)}] frame={index}: {len(boxes)} candidatos', flush=True)


if __name__ == '__main__':
    main()
