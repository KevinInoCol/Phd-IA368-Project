# Fase 2 — SLAM en el museo (2026-10-20)

Ruta `museum` de `route_driver` (13 puntos, ~58 m, ~250 s, vuelve al inicio). Referencia: el
museo vacío mapeado con el hokuyo (`vacio/hokuyo_ref`, ATE 0.074 m; 94.6 % de sus celdas
ocupadas a ≤ 15 cm del plano de la malla). Métricas de `evaluate` (ATE frente al ground truth;
precisión/completitud frente a la referencia) y de `pepper_museum/tools/trails.py` (celdas
ocupadas que no están en la referencia). Media de 3 repeticiones salvo `ruido_alto` (2).
Sufijos: `T` = slam_toolbox ajustado (Fase 1), `D` = `scan_fusion` con `dynamic_filter`.

## Localización

| Mundo | Config. | ATE SLAM [m] | Error final SLAM [m] | ATE odometría [m] | Error final odometría [m] | Completitud [%] | Área libre [m²] |
|---|---|---|---|---|---|---|---|
| vacío | acc4nofreeT | 0.40 ± 0.18 | 0.15 | 0.19 | 0.34 | 36.7 | 266.6 |
| vacío | acc4nofree | 0.24 ± 0.07 | 0.26 | 0.15 | 0.34 | 47.4 | 266.5 |
| vacío | v8T | 0.53 ± 0.35 | 0.25 | 0.17 | 0.31 | 22.8 | 136.6 |
| estático (23 personas) | acc4nofreeT | 0.31 ± 0.07 | 0.15 | 0.18 | 0.37 | 37.3 | 265.9 |
| estático | v8T | 0.49 ± 0.18 | 0.57 | 0.18 | 0.41 | 18.1 | 140.4 |
| dinámico (10 + 16 andando) | acc4nofreeT | 0.41 ± 0.05 | **0.09** | 0.16 | 0.37 | 32.4 | 263.8 |
| dinámico | acc4nofreeTD | 0.47 ± 0.08 | 0.31 | 0.15 | 0.30 | 29.5 | 251.1 |
| dinámico | acc4nofree | 0.49 ± 0.04 | 0.24 | 0.15 | 0.33 | 25.9 | 264.2 |
| dinámico | acc4nofreeD | 0.52 ± 0.11 | 0.52 | 0.17 | 0.39 | 28.1 | 250.6 |
| dinámico | v8T | 0.36 ± 0.23 | 0.50 | 0.18 | 0.41 | 36.8 | 138.9 |
| vacío, **odometría con ruido alto** | acc4nofreeT | **0.18 ± 0.14** | 0.15 | 1.22 | 0.73 | 62.3 | 280.1 |
| vacío, odometría con ruido alto | acc4nofree | 0.37 ± 0.19 | 0.29 | 1.37 | 0.78 | 35.9 | 279.8 |

## Estelas (museo dinámico)

Celdas ocupadas del mapa a > 15 cm de la referencia y a < 0.5 m de la trayectoria de algún actor
(sin contar las que están junto a una persona estática). "Base" = la misma medida en el museo
vacío (trayectorias del mundo dinámico, pero sin nadie caminando): lo que cae ahí por error del
SLAM, no por la gente.

| Config. | Junto a actores [m²] | Base (vacío) [m²] | Estela ≈ [m²] | Personas estáticas [m²] |
|---|---|---|---|---|
| v8T | 0.58 ± 0.10 | 0.28 | 0.3 | 0.26 |
| acc4nofreeT | 1.24 ± 0.04 | 0.22 | **1.0** | 0.42 |
| acc4nofree | 1.19 ± 0.03 | 0.11 | 1.1 | 0.39 |
| acc4nofreeTD | 0.30 ± 0.08 | — | **≈ 0.1** | 0.17 |
| acc4nofreeD | 0.33 ± 0.01 | — | ≈ 0.2 | 0.19 |

Figuras: `route_ref.png` (ruta y referencia), `dinamico/trails.png` (v8T, acc4nofreeT,
acc4nofreeTD), `*/trajectories.png`, `*/position_error.png`, `*/maps.png`. Tablas por corrida en
`*/summary.md`.

## Conclusiones

1. **El museo es más difícil que la oficina por su tamaño, no por la gente.** Con la odometría
   simulada (poco ruidosa) el SLAM con los láseres de Pepper (7 m de alcance, 45 rayos) se
   desvía a mitad de ruta en los tramos abiertos —las trayectorias se "encogen" ~0.4 m, como en
   un pasillo sin referencias a lo largo— y su ATE (0.3–0.5 m) es peor que el de la odometría
   sola (~0.17 m). El cierre de lazo lo recupera: el error final de acc4nofreeT (0.09–0.15 m)
   es menor que el de la odometría (0.30–0.41 m).
2. **Con una odometría mala el SLAM sí es imprescindible**, igual que en la oficina: con ruido
   alto la odometría acumula 1.2–1.4 m de ATE y acc4nofreeT 0.18 m; el ajuste de Karto vuelve a
   ayudar (0.18 frente a 0.37 m sin ajustar).
3. **La gente caminando deja estelas y la acumulación las triplica**: ~1 m² de obstáculos falsos
   a lo largo de las trayectorias con acc4, ~0.3 m² sin acumular. Las 23 personas estáticas no
   empeoran la localización (0.31 m frente a 0.40 m en vacío, dentro de la variación).
4. **El filtro dinámico de `scan_fusion` elimina las estelas** (1.24 → 0.30 m², casi la base)
   pero no mejora la localización (ATE 0.47 frente a 0.41 m, error final mayor) y quita algo de
   mapa real (251 frente a 264 m² de área libre) y parte de las personas quietas (los láseres,
   bajos, ven entre sus piernas). Sirve para el mapa (p. ej. para Nav2), no para localizar.
