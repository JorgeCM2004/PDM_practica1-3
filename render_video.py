"""Vídeos de detecciones por instante; sin asociación temporal ni tracking."""
import argparse
import csv
import json
import hashlib
from pathlib import Path

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from main import SEQUENCES, SENSORS, load_frame
from road import box_polygon, load_road, load_lane_segments
from detector import detect_improved


SIZE = (1800, 1080)
PANEL = 840
TOP = 155
LEFTS = (40, 920)


def font(size):
    for name in ('C:/Windows/Fonts/arial.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def pixels(xy, roi):
    xmin, xmax, ymin, ymax = roi
    return np.column_stack(((xy[:, 0]-xmin)/(xmax-xmin)*(PANEL-1),
                            (ymax-xy[:, 1])/(ymax-ymin)*(PANEL-1))).round().astype(int)


DETECTION_COLOR = (255, 145, 35)


def render(points, record, name, elapsed, roi, road, total, groups):
    xmin, xmax, ymin, ymax = roi
    mask = ((points[:, 0]>=xmin)&(points[:, 0]<=xmax)&
            (points[:, 1]>=ymin)&(points[:, 1]<=ymax))
    points = points[mask]
    if len(groups) != len(record['boxes']):
        raise ValueError('Debe haber un clúster por caja final')
    background = np.full((PANEL, PANEL, 3), (14, 24, 36), dtype=np.uint8)
    plane = np.asarray(record['ground_plane_abc'])
    height = points[:, 2]-points[:, :2]@plane[:2]-plane[2]
    # Fondo original: suelo gris y retornos elevados azul claro, sin máscara vial visual.
    for selected, color in ((height < .4, (66, 79, 90)),
                            ((height >= .4)&(height <= 4.5), (130, 213, 236))):
        xy = pixels(points[selected, :2], roi)
        background[xy[:, 1], xy[:, 0]] = color
    clustered = background.copy()
    for group in groups:
        xy = pixels(group[:, :2], roi)
        # Marcadores de 3 x 3 píxeles para ver los retornos a escala de todo el cruce.
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                shifted = xy+[dx, dy]
                valid = ((shifted>=0)&(shifted<PANEL)).all(axis=1)
                clustered[shifted[valid, 1], shifted[valid, 0]] = DETECTION_COLOR
    canvas = Image.new('RGB', SIZE, (9, 16, 26))
    draw = ImageDraw.Draw(canvas)
    draw.text((40, 20), 'UrbanIng-V2X | Vehículos por fotograma', font=font(32), fill='white')
    draw.text((40, 67), f'{name}   ·   t = {elapsed:.1f} s   ·   Frame {record["frame_index"]} / {total-1}', font=font(23), fill='#b5c8d7')
    draw.text((40, 119), 'NUBE LiDAR · sin colorear clústeres', font=font(23), fill='#b5c8d7')
    draw.text((920, 119), f'DETECCIONES: {record["count"]} candidatos', font=font(25), fill='#ffcd66')
    for left, panel in zip(LEFTS, (background, clustered)):
        canvas.paste(Image.fromarray(panel), (left, TOP))
        polygons = list(road.geoms) if hasattr(road, 'geoms') else [road]
        for polygon in polygons:
            for ring in [polygon.exterior, *polygon.interiors]:
                xy = pixels(np.asarray(ring.coords), roi)+[left, TOP]
                # El mapa ya está recortado a la ROI.
                draw.line([tuple(p) for p in xy], fill='#365970', width=1)
        draw.rectangle((left, TOP, left+PANEL, TOP+PANEL), outline='#526474')
    for box in record['boxes']:
        xy = pixels(np.asarray(box_polygon(box).exterior.coords), roi)+[LEFTS[1], TOP]
        draw.line([tuple(p) for p in xy], fill=DETECTION_COLOR, width=2)
    draw.text((40, 1010), '11 · 12 · 31 · 32   |   Solo LiDAR de infraestructura   |   Sin tracking', font=font(24), fill='white')
    draw.text((40, 1044), 'Naranja: clústeres detectados y cajas · Azul claro: otros puntos elevados · Gris: suelo.', font=font(21), fill='#b5c8d7')
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', action='append', choices=SEQUENCES)
    parser.add_argument('--data', type=Path, default=Path('data'))
    parser.add_argument('--results', type=Path, default=Path('results/improved'))
    parser.add_argument('--output', type=Path, default=Path('results/videos'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in args.sequence or SEQUENCES:
        folder = args.data/'dataset'/name
        calibration = json.loads((folder/'calibration.json').read_text())
        with (folder/'timesync_info.csv').open(newline='') as stream:
            sync = {row[0]: row[1:] for row in csv.reader(stream)}
        source = args.results/name
        run = json.loads((source/'run.json').read_text())
        if run['method'] != 'improved':
            raise ValueError('La visualización de clústeres requiere el detector mejorado')
        if set(run['sensors']) != set(SENSORS) or run['tracking']:
            raise ValueError('Se requieren exactamente los cuatro LiDAR de infraestructura y sin tracking')
        records = [json.loads(line) for line in (source/'detections.jsonl').read_text().splitlines()]
        if [r['frame_index'] for r in records] != list(range(len(sync['timestamp_ms']))):
            raise ValueError('Se requieren todos los fotogramas, ordenados y sin duplicados')
        times = np.array([r['timestamp_ms'] for r in records])
        if not np.array_equal(times, np.asarray(sync['timestamp_ms'], dtype=np.int64)):
            raise ValueError('Las detecciones no coinciden con los tiempos de la secuencia')
        steps = np.diff(times)
        if not len(steps) or np.any(steps <= 0) or not np.allclose(steps, np.median(steps), atol=1):
            raise ValueError('Esta exportación requiere muestreo temporal uniforme')
        fps = 1000/float(np.median(steps))
        roi = run['parameters']['roi']
        if not np.isclose(roi[1]-roi[0], roi[3]-roi[2]):
            raise ValueError('La vista cenital requiere ROI cuadrada para conservar las proporciones')
        road = load_road(args.data/'crossings_lanelet2map.osm', roi)
        lanes = load_lane_segments(args.data/'crossings_lanelet2map.osm')
        cache = args.output/'clusters'/name
        cache.mkdir(parents=True, exist_ok=True)
        # Guardar pertenencia exacta para futuros cambios de estilo sin recalcular detecciones.
        signature = hashlib.sha256(json.dumps([run, calibration], sort_keys=True).encode())
        for path in (Path(__file__).with_name('detector.py'), Path(__file__).with_name('main.py'),
                     Path(__file__).with_name('road.py'), args.data/'crossings_lanelet2map.osm'):
            signature.update(path.read_bytes())
        destination = args.output/f'{name}.mp4'
        temporary = destination.with_suffix('.pending.mp4')
        writer = imageio_ffmpeg.write_frames(str(temporary), SIZE, fps=fps,
            codec='libx264', pix_fmt_out='yuv420p', macro_block_size=2,
            output_params=['-crf', '20', '-movflags', '+faststart'])
        writer.send(None)
        try:
            for index, record in enumerate(records):
                points, sources = load_frame(folder, calibration, sync, record['frame_index'])
                if sources != record['sources'] or record['count'] != len(record['boxes']):
                    raise ValueError('Detecciones inconsistentes con los archivos de origen o el conteo')
                fingerprint = signature.copy()
                fingerprint.update(json.dumps(record, sort_keys=True).encode())
                fingerprint.update(points.tobytes())
                key = fingerprint.hexdigest()
                cached = cache/f'{record["frame_index"]:04d}.npz'
                groups = None
                if cached.exists():
                    with np.load(cached, allow_pickle=False) as saved:
                        if str(saved['key']) == key:
                            lengths = saved['lengths']
                            if lengths.tolist() != [b['num_points'] for b in record['boxes']]:
                                raise ValueError('Caché de clústeres inconsistente')
                            groups = list(np.split(saved['points'], np.cumsum(lengths)[:-1])) if len(lengths) else []
                if groups is None:
                    _, boxes, _ = detect_improved(points, roi, run['parameters']['voxel'], road,
                        run['detector_config'], lanes, include_cluster_points=True)
                    if len(boxes) != len(record['boxes']):
                        raise ValueError('El detector cambió: regenerar las detecciones antes del vídeo')
                    for current, saved in zip(boxes, record['boxes']):
                        if (current['num_points'] != saved['num_points'] or
                            not np.allclose(current['center']+current['dimensions']+[current['yaw_rad']],
                                            saved['center']+saved['dimensions']+[saved['yaw_rad']], atol=1e-6, rtol=0)):
                            raise ValueError('Los clústeres no corresponden a las cajas guardadas')
                    groups = [box['_cluster_points'] for box in boxes]
                    np.savez_compressed(cached, key=key, lengths=np.array([len(g) for g in groups], dtype=int),
                                        points=np.concatenate(groups) if groups else np.empty((0, 3)))
                frame = render(points, record, name, (times[index]-times[0])/1000, roi, road, len(records), groups)
                writer.send(np.asarray(frame))
                if index == 100:
                    frame.save(args.output/f'{name}.jpg', quality=93)
                if index % 50 == 0:
                    print(f'{name}: {index}/{len(records)}', flush=True)
        finally:
            writer.close()
        temporary.replace(destination)
        print(f'{destination}: {len(records)} frames, {fps:g} fps', flush=True)


if __name__ == '__main__':
    main()
