## Recorrido por los 12 lugares

| Config. | Lugares alcanzados | Fallos | Error al llegar [m] (máx.) | Error AMCL RMSE [m] | AMCL p95 / máx. [m] | Yaw AMCL RMSE [°] | Tiempo por lugar [s] | Recorrido [m] | Mín. a pared [m] | Tiempo < radio [s] | Mín. a persona [m] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fused | 12 / 12 | — | 0.20 (0.47) | 0.32 | 0.71 / 1.34 | 2.6 | 48 | 117 | 0.45 | 0.0 | 0.53 |
| hokuyo | 12 / 12 | — | 0.15 (0.31) | 0.12 | 0.23 / 0.40 | 1.4 | 46 | 117 | 0.42 | 0.0 | 0.54 |

## Llamadas de visitante

| Config. | Desde | Resultado | Dist. final a la persona real [m] | Error de orientación [°] | Tiempo [s] |
|---|---|---|---|---|---|
| fused | mesa | reached | 4.49 | -135 | 37 |
| fused | entrada | reached | 1.24 | -6 | 45 |
| fused | introduccion | reached | 1.17 | -1 | 34 |
| fused | tecnologia | reached | 1.25 | +2 | 35 |
| fused | puertas_sur | nobody | 3.26 | -113 | 27 |
| fused | sala_oeste | reached | 0.88 | +4 | 30 |
| hokuyo | mesa | reached | 4.46 | -141 | 36 |
| hokuyo | entrada | reached | 1.29 | +2 | 45 |
| hokuyo | introduccion | reached | 1.04 | +3 | 37 |
| hokuyo | tecnologia | reached | 1.30 | +5 | 36 |
| hokuyo | puertas_sur | reached | 1.37 | -3 | 42 |
| hokuyo | sala_oeste | reached | 0.89 | -1 | 29 |

## Notas (2026-10-28)

Mundo `museum_test_people`, sin GUI, una corrida por configuración, mismo orden de lugares y de
llamadas (`pepper_visitors/tools/eval_navigation.py`). `fused` = `scan_fusion` (accumulate 4, sin
espacio libre) para AMCL, costmaps y collision_monitor, sobre `museum_fused.yaml` (mapa hecho con
los mismos láseres); `hokuyo` = el láser de referencia de 720 rayos sobre `museum_hokuyo.yaml`.

- Error de AMCL dentro del bloque central (x 1.3–6, y −8…−0.5): fused 0.90 m RMSE, hokuyo 0.31 m;
  fuera: fused 0.24 m (p95 0.30), hokuyo 0.10 m (p95 0.17). El mapa fusionado **no tiene el
  interior del bloque central** (la ruta del SLAM no entró): ahí AMCL se pierde (máx. 1.34 m) y se
  recupera al salir. Buena parte de la diferencia es del mapa, no sólo del sensor.
- Sin choques en ninguna: distancia mínima del centro de Pepper a la pared 0.45 / 0.42 m (radio 0.24).
- Llamada desde la mesa: en las **dos** configuraciones Pepper "llegó" a una persona falsa en
  (−5.65, 1.3): un retrato de los paneles detrás de la mesa táctil (clasificador con prob. 0.35–0.38).
  Fallo de percepción, no del láser: no cuenta para la comparación.
- Puertas sur: con fused YOLO no detectó a nadie en el barrido (con hokuyo sí, a 3.4 m). Una sola
  muestra; no está claro que dependa del láser.
