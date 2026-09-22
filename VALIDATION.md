# Validación de la práctica 3

IoU BEV ≥ 0,5; asociación uno a uno dentro de cada fotograma. Sin tracking.
Ambos métodos evaluados en la misma calzada del mapa oficial (margen 0,35 m) y ROI ±45 m.
Las anotaciones de referencia incluyen vehículos ocluidos; no se eliminan para mejorar el resultado.

## Secuencias reservadas para validación

| Método | Precisión | Recall | F1 | Error absoluto medio del conteo |
|---|---:|---:|---:|---:|
| baseline | 70.1% | 35.0% | 0.467 | 15.49 vehículos/frame |
| improved | 81.5% | 52.4% | 0.638 | 11.06 vehículos/frame |

## Recall por clase en validación

| Clase | Original | Mejorado |
|---|---:|---:|
| Car | 32.3% | 54.9% |
| Truck | 73.5% | 51.0% |
| Van | 52.4% | 51.2% |
| Bus | 29.1% | 25.6% |

La mejora agregada se concentra en turismos. El recall de camiones, autobuses y furgonetas retrocede; el ajuste geométrico no mejora todas las clases.

## Resultado por secuencia (mejorado)

| Secuencia | Uso | Frames | Precisión | Recall | MAE conteo |
|---|---|---:|---:|---:|---:|
| 20241126_0024_crossing1_09 | ajuste | 200 | 80.5% | 50.6% | 18.07 |
| 20241126_0008_crossing1_01 | validación | 200 | 81.3% | 53.4% | 11.46 |
| 20241127_0000_crossing1_00 | validación | 200 | 81.8% | 51.1% | 10.68 |

## Límites de la solución

La mejora se mide sobre detecciones reales; no es un conteo exacto. Persisten omisiones por oclusión y distancia, fusiones de vehículos próximos y errores de cajas parciales.
El detector no asigna clases específicas. La evaluación incluye Car, Van, Bus, Truck, Trailer, OtherVehicle y Motorcycle; las motos no tienen un modelo especializado y pueden quedar sin detectar.
Los números de las vistas cenitales solo identifican cajas dentro de esa imagen. No se estiman trayectorias, velocidades ni vehículos únicos.

Los parámetros se eligieron con los fotogramas 0, 40, 80, 120 y 160 de la primera secuencia, por F1 a IoU 0,5. Las otras dos no se usaron para ajustarlos.

![Conteos](results/counts_comparison.png)
