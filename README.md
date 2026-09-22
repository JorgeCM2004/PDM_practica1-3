# Práctica 3 — UrbanIng-V2X

Detección geométrica de vehículos, cajas 3D orientadas y conteo **por fotograma**.
Solo se utilizan los LiDAR de infraestructura **11, 12, 31 y 32**. No hay tracking,
asociación temporal, estimación de velocidades ni conteo de vehículos únicos.

## Ejecutar

Desde la carpeta del proyecto:

```powershell
uv sync
uv run python main.py --limit 10
uv run python main.py
uv run python evaluate.py --results results/improved
```

Los scripts originales `download_dataset.py` y `extract_dataset.py` descargan y
extraen las tres secuencias. Los datos deben estar en `data/dataset/<secuencia>/`,
las anotaciones en `data/labels/` y el mapa en `data/crossings_lanelet2map.osm`.

La ejecución completa procesa 200 fotogramas por secuencia (600 en total):

- `20241126_0024_crossing1_09`: desarrollo y ajuste.
- `20241126_0008_crossing1_01`: validación sin ajuste de parámetros.
- `20241127_0000_crossing1_00`: validación sin ajuste de parámetros.

Ejemplos de selección:

```powershell
uv run python main.py --sequence 20241126_0008_crossing1_01 --start 50 --limit 10 --output results/example
uv run python main.py --step 20 --output results/sample
uv run python evaluate.py --results results/sample
```

## Archivos del proyecto

- `main.py`: ejecución, lectura de LiDAR y visualización.
- `detector.py`, `road.py` y `detector_config.json`: detección y máscara vial.
- `evaluate.py`: comparación con las anotaciones.
- `report_results.py`: informe y gráficas del método original frente al final.
- `download_dataset.py` y `extract_dataset.py`: preparación de datos.
- `pyproject.toml` y `uv.lock`: dependencias reproducibles.
- `VALIDATION.md`: resultados y limitaciones de la solución.

En `results/` se conservan las salidas finales (`improved`), la referencia necesaria
para las comparativas (`baseline`) y los informes e imágenes de la entrega.

## Qué hace el detector mejorado

1. Selecciona los cuatro archivos de cada instante mediante `timesync_info.csv`.
2. Aplica la calibración `extrinsics.gTl` propia de cada sensor y fusiona las nubes
   en coordenadas globales. Compartir mástil no implica compartir calibración.
3. Construye la calzada a partir de los carriles `subtype=road` del mapa oficial.
   Convierte GPS a UTM32 y resta el origen de crossing1 del devkit. Excluye aceras,
   isletas y carriles exclusivamente ciclistas. El margen fijo es 0,35 m.
4. Recorta la ROI (±45 m por defecto) y reduce la nube con vóxeles de 0,15 m.
5. Estima un plano de suelo por RANSAC y corrige localmente su altura con una malla
   de 3 m, interpolación de huecos y suavizado. No utiliza el modelo de suelo del
   devkit derivado de estados y etiquetas.
6. Agrupa puntos elevados mediante DBSCAN en XY. El área de agrupamiento conserva
   0,5 m de contexto alrededor de la máscara; acepta cajas cuyo centro está en ella.
7. Ajusta rectángulos orientados. Para observaciones pequeñas o estrechas usa el
   eje del carril cercano. Completa las huellas parciales con una dimensión mínima
   de 4 × 1,8 m, expandiendo la cara oculta en sentido opuesto al mástil cercano.
   Es una hipótesis geométrica, no una medida exacta de la carrocería oculta.
8. Filtra tamaños y suprime cajas que se solapan mucho con otra con más puntos.
   La base de cada caja sigue el suelo local; su altura se estima con los puntos.

El mapa es información estática auxiliar, no un sensor adicional. El detector no
lee cámaras, LiDAR de vehículos, estados de vehículos ni anotaciones. Los vehículos
parados se procesan igual que los demás; no se elimina el fondo por persistencia temporal.

`detector_config.json` guarda los parámetros finales. Se eligieron comparando seis
combinaciones en los fotogramas 0, 40, 80, 120 y 160 de la primera secuencia, según
F1 con IoU BEV 0,5. Las otras dos secuencias se reservaron para validación.

## Evaluación y comparación

```powershell
uv run python main.py --method baseline --output results/baseline --preview-every 0
uv run python evaluate.py --results results/baseline
uv run python evaluate.py --results results/improved --point-support --support-step 10
uv run python report_results.py
```

La evaluación convierte las etiquetas del formato del dataset a instantes en ms.
No usa sus identificadores para asociar vehículos entre instantes. Compara cajas
orientadas en planta (IoU BEV ≥ 0,5), con asignación uno a uno de máxima cardinalidad
y después máximo solapamiento. No es una métrica de IoU 3D.

Ambos métodos se evalúan dentro de **la misma calzada y ROI**, según el centro de
las cajas. Se informa por separado de predicciones fuera de calzada: el método
original las puede generar, pero se excluyen de esta comparación espacial común.
Se evalúan Car, Van, Bus, Truck, Trailer, OtherVehicle y Motorcycle. Se mantienen los
vehículos de captura como objetos a detectar, aunque no se utilicen sus sensores.
Bicicletas, patinetes y peatones quedan fuera de este conteo de vehículos motorizados.

Se guardan precisión, recall, F1, falsos positivos, omisiones, MAE/RMSE y sesgo del
conteo, además del recall por clase. **Un conteo correcto no implica cajas correctas**:
los falsos positivos pueden compensar omisiones.

`--point-support` añade recall según los retornos de los cuatro LiDAR dentro de cada
caja real (al menos 5, 20 o 50 puntos). Se excluyen los 15 cm inferiores de la caja
para reducir la contribución del suelo. Es un diagnóstico de observabilidad: **no
reemplaza la evaluación global**, que conserva también los vehículos ocluidos.
Las anotaciones solo se leen en evaluación o en ajuste supervisado de parámetros.
El evaluador rechaza resultados incompletos, timestamps repetidos o desalineados.
`--support-step 10` calcula ese diagnóstico en 60 fotogramas de los 600, manteniendo
las métricas principales sobre todos ellos. El valor 1 analiza el soporte en todos.

Consultar `VALIDATION.md` y `results/REPORT.md` tras generar el informe. Los resultados
sobre la secuencia de desarrollo no deben presentarse como generalización independiente.

## Salidas

En `results/improved/<secuencia>/`:

- `detections.jsonl`: instante, archivos de los cuatro sensores, conteo y cajas.
  `center` es el centro XYZ en metros globales; `dimensions` es longitud, anchura,
  altura; `yaw_rad` es el giro desde X alrededor de Z, módulo π. No estima rumbo.
  `observed_dimensions_xy` permite distinguir la huella observada de la completada.
- `counts.csv`: número de candidatos por fotograma.
- `bev_*.png`: nube cenital, máscara azul y cajas rojas. Los números son índices de
  esa imagen, no identidades persistentes. `--preview-every 0` desactiva las imágenes.
- `road_mask.geojson`: máscara en coordenadas locales métricas, **no lon/lat WGS84**.
- `run.json`: método, configuración y parámetros de ejecución.
- `evaluation.json` y `evaluation.csv`: métricas y resultados por fotograma.

`results/comparison.json`, `counts_comparison.png` y `comparison_<secuencia>.png`
comparan el método original con el mejorado. La evaluación filtra a calzada los
conteos originales, por lo que pueden diferir del `counts.csv` de baseline.

Cada ejecución sobre una misma carpeta sobrescribe CSV, JSONL y metadatos. Usar un
`--output` nuevo para conservar experimentos. Pueden permanecer PNG de ejecuciones
anteriores. Los datos y resultados voluminosos están excluidos de Git.

## Limitaciones

Es un detector geométrico sin modelo semántico entrenado. No garantiza conteo exacto:
puede omitir objetos sin retornos, fusionar vehículos próximos, dividir carrocerías,
y equivocarse al completar cajas. Las motos están incluidas en la evaluación pero
la hipótesis dimensional favorece coches y vehículos mayores. El mapa debe alinearse
con la calibración; el margen no garantiza adaptación a cambios de infraestructura.

No sumar los conteos para obtener vehículos únicos: el mismo vehículo aparece en
muchos fotogramas. No hay tracking.

Referencias: [repositorio oficial](https://github.com/thi-ad/UrbanIng-V2X) y
[artículo](https://arxiv.org/abs/2510.23478). La conversión de coordenadas y etiquetas
se ha contrastado con el código instalado del devkit `urbaning`.
