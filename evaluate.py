"""Evaluación exclusivamente por fotograma: cajas BEV orientadas y conteo."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from road import filter_boxes, load_road, box_polygon as polygon

VEHICLE_CLASSES = ('Car', 'Van', 'Bus', 'Truck', 'Trailer', 'OtherVehicle', 'Motorcycle')


def read_labels(path):
    data = json.loads(Path(path).read_text())
    frames = {}
    for track in data['tracks']:
        for i, timestamp in enumerate(track['timestamps']):
            frame = frames.setdefault(round(timestamp * 1000), [])
            if track['object_type'] not in VEHICLE_CLASSES:
                continue
            dimensions = track['dimensions'][0 if len(track['dimensions']) == 1 else i]
            frame.append({'center': track['positions'][i], 'dimensions': dimensions,
                          'yaw_rad': track['orientations'][i], 'class': track['object_type']})
    return frames


def match_boxes(predicted, truth, threshold=0.5):
    """Asignación de máxima cardinalidad sobre IoU válido, luego máximo IoU."""
    ious = np.zeros((len(predicted), len(truth)))
    pp, gg = [polygon(b) for b in predicted], [polygon(b) for b in truth]
    if pp and gg:
        a, b = np.array([p.bounds for p in pp]), np.array([g.bounds for g in gg])
        nearby = ((a[:, None, 0] < b[None, :, 2]) & (a[:, None, 2] > b[None, :, 0])
                  & (a[:, None, 1] < b[None, :, 3]) & (a[:, None, 3] > b[None, :, 1]))
        for i, j in zip(*np.nonzero(nearby)):
            p, g = pp[i], gg[j]
            intersection = p.intersection(g).area
            ious[i, j] = intersection / (p.area + g.area - intersection)
    valid = ious >= threshold
    # La bonificación supera cualquier mejora posible de IoU total.
    score = valid * (min(ious.shape) + 1 + ious)
    rows, cols = linear_sum_assignment(-score)
    pairs = [(int(i), int(j), float(ious[i, j])) for i, j in zip(rows, cols) if valid[i, j]]
    return pairs


def evaluate_frame(predicted, truth, threshold=0.5):
    matches = match_boxes(predicted, truth, threshold)
    tp = len(matches)
    matched = {pair[1] for pair in matches}
    classes = sorted({b.get('class', 'vehicle') for b in truth})
    return {'predicted': len(predicted), 'ground_truth': len(truth), 'tp': tp,
            'fp': len(predicted)-tp, 'fn': len(truth)-tp,
            'count_error': len(predicted)-len(truth), 'matches': matches,
            'class_recall': {name: {'ground_truth': sum(b.get('class', 'vehicle') == name for b in truth),
                'tp': sum(b.get('class', 'vehicle') == name and i in matched for i,b in enumerate(truth))}
                for name in classes}}


def summarize(rows):
    tp, fp, fn = (sum(r[k] for r in rows) for k in ('tp', 'fp', 'fn'))
    errors = np.array([r['count_error'] for r in rows])
    class_totals = {}
    for row in rows:
        for name, values in row.get('class_recall', {}).items():
            totals = class_totals.setdefault(name, {'ground_truth': 0, 'tp': 0})
            for key in totals:
                totals[key] += values[key]
    for totals in class_totals.values():
        totals['recall'] = totals['tp'] / totals['ground_truth']
    return {'frames': len(rows), 'tp': tp, 'fp': fp, 'fn': fn,
            'recall_by_class': class_totals,
            'outside_road_predictions': sum(r.get('outside_road_predictions', 0) for r in rows),
            'precision': tp/(tp+fp) if tp+fp else None,
            'recall': tp/(tp+fn) if tp+fn else None,
            'f1': 2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None,
            'count_mae': float(np.abs(errors).mean()) if len(rows) else None,
            'count_bias': float(errors.mean()) if len(rows) else None,
            'count_rmse': float(np.sqrt(np.mean(errors**2))) if len(rows) else None}


def point_support(points, boxes):
    """Número de retornos reales dentro de cada caja 3D (sin usarlo para detectar)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(points[:, :2])
    counts = []
    for b in boxes:
        center = np.array(b['center'])
        dimensions = np.array(b['dimensions'])
        ids = tree.query_ball_point(center[:2], np.linalg.norm(dimensions[:2])/2 + 0.1)
        local = points[ids] - center
        c, s = np.cos(b['yaw_rad']), np.sin(b['yaw_rad'])
        xy = local[:, :2] @ np.array([[c, -s], [s, c]])
        counts.append(int(np.sum((np.abs(xy)<=dimensions[:2]/2+0.05).all(axis=1)
                                & (local[:, 2] >= -dimensions[2]/2+0.15)
                                & (local[:, 2] <= dimensions[2]/2+0.05))))
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, default=Path('results/improved'))
    parser.add_argument('--data', type=Path, default=Path('data'))
    parser.add_argument('--iou', type=float, default=0.5)
    parser.add_argument('--point-support', action='store_true', help='Diagnosticar recall según retornos LiDAR')
    parser.add_argument('--support-step', type=int, default=1, help='Calcular soporte cada N fotogramas evaluados; las métricas globales usan todos')
    args = parser.parse_args()
    if not 0 < args.iou <= 1:
        parser.error('IoU debe estar entre 0 y 1')
    if args.support_step < 1:
        parser.error('support-step debe ser positivo')
    reports, all_rows = {}, []
    for folder in sorted(args.results.iterdir()):
        detections = folder / 'detections.jsonl'
        if not detections.is_file():
            continue
        metadata = json.loads((folder / 'run.json').read_text())
        roi = metadata['parameters']['roi']
        road = load_road(args.data / 'crossings_lanelet2map.osm', roi)
        labels = read_labels(args.data / 'labels' / f'{folder.name}.json')
        support_path = args.data / 'dataset' / folder.name
        with (support_path / 'timesync_info.csv').open(newline='') as stream:
            sync = {r[0]: r[1:] for r in csv.reader(stream)}
        params = metadata['parameters']
        indices = list(range(params['start'], len(sync['timestamp_ms']), params['step']))[:params['limit']]
        expected = {int(sync['timestamp_ms'][i]) for i in indices}
        if args.point_support:
            from main import load_frame
            calibration = json.loads((support_path / 'calibration.json').read_text())
        rows = []
        seen = set()
        for line in detections.read_text().splitlines():
            record = json.loads(line)
            timestamp = record['timestamp_ms']
            if int(sync['timestamp_ms'][record['frame_index']]) != timestamp:
                raise ValueError('El índice de fotograma no corresponde al timestamp')
            if timestamp in seen or timestamp not in labels:
                raise ValueError(f'Timestamp repetido o sin anotaciones: {timestamp}')
            seen.add(timestamp)
            predicted = filter_boxes(record['boxes'], road)
            truth = filter_boxes(labels[timestamp], road)
            row = evaluate_frame(predicted, truth, args.iou)
            row['outside_road_predictions'] = len(record['boxes']) - len(predicted)
            if args.point_support and len(rows) % args.support_step == 0:
                points, _ = load_frame(support_path, calibration, sync, record['frame_index'])
                support = point_support(points, truth)
                matched_gt = {pair[1] for pair in row['matches']}
                row['ground_truth_point_counts'] = support
                row['support_recall'] = {str(n): {
                    'ground_truth': sum(count >= n for count in support),
                    'tp': sum(count >= n and i in matched_gt for i, count in enumerate(support))}
                    for n in (5, 20, 50)}
            row.update(timestamp_ms=timestamp, frame_index=record['frame_index'])
            rows.append(row)
            if len(rows) % 50 == 0:
                print(f'Evaluación {folder.name}: {len(rows)}/{len(expected)}', flush=True)
        if seen != expected:
            raise ValueError(f'Ejecución incompleta o distinta de los metadatos: {folder.name}; {len(seen)}/{len(expected)} frames')
        report = summarize(rows)
        if args.point_support:
            report['recall_by_min_lidar_points'] = support_summary(rows)
        reports[folder.name] = report
        all_rows.extend(rows)
        (folder / 'evaluation.json').write_text(json.dumps({'summary': report, 'frames': rows}, indent=2))
        with (folder / 'evaluation.csv').open('w', newline='') as stream:
            keys = ['frame_index', 'timestamp_ms', 'predicted', 'ground_truth', 'tp', 'fp', 'fn', 'count_error']
            writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
    if not reports:
        parser.error('No se encontraron detecciones')
    report = {'iou_bev_threshold': args.iou, 'classes': VEHICLE_CLASSES,
              'scope': 'Centros dentro de calzada oficial + margen 0.35 m y ROI; incluye ocluidos',
              'prediction_scope': 'Ambos métodos se filtran a la misma calzada para comparación; se informa también de detecciones externas',
              'sequences': reports, 'total': summarize(all_rows)}
    if args.point_support:
        report['total']['recall_by_min_lidar_points'] = support_summary(all_rows)
        report['point_support_sampling_step'] = args.support_step
        report['point_support_frames'] = sum('support_recall' in row for row in all_rows)
    (args.results / 'evaluation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def support_summary(rows):
    rows = [row for row in rows if 'support_recall' in row]
    result = {}
    for n in ('5', '20', '50'):
        total = sum(row['support_recall'][n]['ground_truth'] for row in rows)
        tp = sum(row['support_recall'][n]['tp'] for row in rows)
        result[n] = {'ground_truth': total, 'tp': tp, 'recall': tp/total if total else None}
    return result


if __name__ == '__main__':
    main()
