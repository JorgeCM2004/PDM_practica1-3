"""Crear comparativa y visualizaciones a partir de evaluaciones completas."""
import json
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from main import SEQUENCES, load_frame, voxelize
from evaluate import summarize,polygon,read_labels
from road import load_road,filter_boxes,draw_road


def main():
    root=Path('results')
    evaluations={method:json.loads((root/method/'evaluation.json').read_text()) for method in ('baseline','improved')}
    if any(e['iou_bev_threshold'] != 0.5 for e in evaluations.values()):
        raise ValueError('Este informe requiere evaluación a IoU 0.5 para ambos métodos')
    for method in evaluations:
        for name in SEQUENCES:
            if evaluations[method]['sequences'][name]['frames'] != 200:
                raise ValueError('El informe completo requiere los 200 fotogramas de cada secuencia')
    summaries={}
    for method in evaluations:
        rows=[]
        for name in SEQUENCES[1:]:
            rows.extend(json.loads((root/method/name/'evaluation.json').read_text())['frames'])
        summaries[method]=summarize(rows)
    comparison={'development_sequence':SEQUENCES[0], 'validation_sequences':SEQUENCES[1:],
                'validation':summaries, 'all_frames':{m:e['total'] for m,e in evaluations.items()}}
    (root/'comparison.json').write_text(json.dumps(comparison,indent=2))
    fig,axes=plt.subplots(3,1,figsize=(12,10),layout='constrained')
    for ax,name in zip(axes,SEQUENCES):
        for method,color in [('baseline','tab:orange'),('improved','tab:blue')]:
            records=json.loads((root/method/name/'evaluation.json').read_text())['frames']
            ax.plot([r['frame_index']/10 for r in records],[r['predicted'] for r in records],label=method,color=color,alpha=.8)
        ax.plot([r['frame_index']/10 for r in records],[r['ground_truth'] for r in records],label='Anotaciones',color='black')
        ax.set(title=name+(' (ajuste)' if name==SEQUENCES[0] else ' (validación)'),ylabel='Vehículos en calzada',xlabel='Tiempo desde inicio (s)')
        ax.legend(ncol=3);ax.grid(alpha=.2)
    fig.savefig(root/'counts_comparison.png',dpi=150);plt.close(fig)
    road=load_road('data/crossings_lanelet2map.osm')
    for name in SEQUENCES:
        folder=Path('data/dataset')/name
        with (folder/'timesync_info.csv').open(newline='') as stream:
            sync={r[0]:r[1:] for r in csv.reader(stream)}
        points,_=load_frame(folder,json.loads((folder/'calibration.json').read_text()),sync,100)
        points=voxelize(points,.3)
        truth=filter_boxes(read_labels(Path('data/labels')/(name+'.json'))[int(sync['timestamp_ms'][100])],road)
        fig,axes=plt.subplots(1,2,figsize=(16,8),layout='constrained')
        for ax,method in zip(axes,('baseline','improved')):
            records=[json.loads(line) for line in (root/method/name/'detections.jsonl').read_text().splitlines()]
            record=next(r for r in records if r['frame_index']==100)
            boxes=filter_boxes(record['boxes'],road)
            ax.scatter(*points[:,:2].T,s=.2,color='0.7',rasterized=True)
            draw_road(ax,road)
            for i,b in enumerate(truth):
                ax.plot(*polygon(b).exterior.xy,color='green',linewidth=1,linestyle='--',label='Anotaciones' if i==0 else None)
            for i,b in enumerate(boxes):
                ax.plot(*polygon(b).exterior.xy,color='crimson',linewidth=1,label='Detecciones' if i==0 else None)
            ax.set(xlim=(-45,45),ylim=(-45,45),aspect='equal',xlabel='X global (m)',ylabel='Y global (m)',
                   title=f'{method}: {len(boxes)} detecciones / {len(truth)} anotaciones')
            ax.legend(loc='upper left')
        fig.suptitle(f'{name} · fotograma 100 · sin tracking')
        fig.savefig(root/f'comparison_{name}.png',dpi=140);plt.close(fig)
    lines=['# Validación de la práctica 3','',
           'IoU BEV ≥ 0,5; asociación uno a uno dentro de cada fotograma. Sin tracking.',
           'Ambos métodos evaluados en la misma calzada del mapa oficial (margen 0,35 m) y ROI ±45 m.',
           'Las anotaciones de referencia incluyen vehículos ocluidos; no se eliminan para mejorar el resultado.', '',
           '## Secuencias reservadas para validación','',
           '| Método | Precisión | Recall | F1 | Error absoluto medio del conteo |',
           '|---|---:|---:|---:|---:|']
    for method,s in summaries.items():
        lines.append(f"| {method} | {s['precision']:.1%} | {s['recall']:.1%} | {s['f1']:.3f} | {s['count_mae']:.2f} vehículos/frame |")
    lines+=['','## Recall por clase en validación','','| Clase | Original | Mejorado |','|---|---:|---:|']
    for name,s in summaries['improved']['recall_by_class'].items():
        original=summaries['baseline']['recall_by_class'][name]['recall']
        lines.append(f"| {name} | {original:.1%} | {s['recall']:.1%} |")
    lines+=['','La mejora agregada se concentra en turismos. El recall de camiones, autobuses y furgonetas retrocede; el ajuste geométrico no mejora todas las clases.']
    lines+=['','## Resultado por secuencia (mejorado)','','| Secuencia | Uso | Frames | Precisión | Recall | MAE conteo |','|---|---|---:|---:|---:|---:|']
    for name in SEQUENCES:
        s=evaluations['improved']['sequences'][name]
        lines.append(f"| {name} | {'ajuste' if name==SEQUENCES[0] else 'validación'} | {s['frames']} | {s['precision']:.1%} | {s['recall']:.1%} | {s['count_mae']:.2f} |")
    improved=evaluations['improved']
    if 'recall_by_min_lidar_points' in improved['total']:
        lines+=['','## Diagnóstico de retornos LiDAR','',
            f"Muestra de {improved['point_support_frames']} fotogramas (uno de cada {improved['point_support_sampling_step']}); la evaluación principal abarca 600, de los cuales 400 son de validación.",
            'Se excluyen los 15 cm inferiores de las cajas reales para reducir puntos del suelo.',
            '', '| Retornos mínimos en la caja real | Instancias de la muestra | Recall |', '|---|---:|---:|']
        for n,s in improved['total']['recall_by_min_lidar_points'].items():
            lines.append(f"| {n} | {s['ground_truth']} | {s['recall']:.1%} |")
        lines+=['','Este diagnóstico mezcla desarrollo y validación; no sustituye el resultado global ni justifica eliminar vehículos ocluidos del conteo.']
    lines+=['','## Límites de la solución','',
        'La mejora se mide sobre detecciones reales; no es un conteo exacto. Persisten omisiones por oclusión y distancia, fusiones de vehículos próximos y errores de cajas parciales.',
        'El detector no asigna clases específicas. La evaluación incluye Car, Van, Bus, Truck, Trailer, OtherVehicle y Motorcycle; las motos no tienen un modelo especializado y pueden quedar sin detectar.',
        'Los números de las vistas cenitales solo identifican cajas dentro de esa imagen. No se estiman trayectorias, velocidades ni vehículos únicos.',
        '', 'Los parámetros se eligieron con los fotogramas 0, 40, 80, 120 y 160 de la primera secuencia, por F1 a IoU 0,5. Las otras dos no se usaron para ajustarlos.',
        '', '![Conteos](counts_comparison.png)', '']
    (root/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    Path('VALIDATION.md').write_text('\n'.join(lines).replace('(counts_comparison.png)','(results/counts_comparison.png)'),encoding='utf-8')
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
