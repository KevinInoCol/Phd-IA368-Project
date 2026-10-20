# Fase 3 — buscar, acercarse y mirar (2026-10-20)

Evaluación final (`eval_moving_v4.jsonl`, visitor_approach con todas las correcciones): museo con
10 personas quietas y 16 actores caminando (`museum_with_people_moving`), una llamada desde cada
una de 10 poses de arranque (teletransporte + pose inicial de AMCL). Ground truth:
`pepper_visitors/tools/eval_approach.py`.

| Inicio (mundo) | Resultado | Cerca y visibles | Encontradas | Falsos estáticos | Andando | Sobre el mapa | Elegida | Clase | ¿La más cercana? | Dist. final [m] | Tiempo [s] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| (1.0, 4.3) | reached | 3 | 3 | 0 | 4 | 0 | female02_Inf_Der | female_02 | no | 0.99 | 39.6 |
| (7.0, -3.0) | reached | 3 | 3 | 0 | 7 | 0 | male03_Inf_Med_2 | male_03 | sí | 1.17 | 28.3 |
| (5.0, -9.0) | reached | 1 | 1 | 0 | 4 | 0 | male03_Inf_Izq | male_03 | sí | 1.06 | 29.6 |
| (-1.7, -6.0) | reached | 3 | 3 | 0 | 3 | 1 | female02_Sup_Med | female_02 | sí | 1.03 | 36.5 |
| (-5.5, 7.0) | reached | 1 | 1 | 0 | 1 | 0 | female03_Sup_Der | male_03 | sí | 1.02 | 32.8 |
| (3.0, -2.0) | reached | 1 | 1 | 0 | 4 | 0 | female03_Med_Med | male_03 | sí | 1.0 | 35.6 |
| (9.3, -8.0) | reached | 2 | 2 | 0 | 2 | 0 | male03_Inf_Med_2 | male_03 | sí | 0.98 | 36.2 |
| (-5.0, -1.5) | reached | 2 | 2 | 0 | 1 | 1 | female02_Sup_Med | female_02 | sí | 1.06 | 34.0 |
| (-1.5, 1.0) | reached | 4 | 4 | 0 | 5 | 0 | male03_Sup_Der_Junto | male_03 | sí | 1.05 | 37.6 |
| (2.0, -9.0) | reached | 2 | 2 | 0 | 4 | 0 | male03_Inf_Izq | male_03 | sí | 0.99 | 40.6 |

Llamadas: 10; llegó: 10; eligió la más cercana: 9; error de posición de las estáticas: media 0.16 m, máx 0.33 m (n = 27)
Distancia final: 1.04 m de media; tiempo 35 s de media

La única elección "no" es un empate: male03_Sup_Der_Junto a 4.01 m y female02_Inf_Der a 4.04 m.
El clasificador tomó a female_03 (chaqueta marrón) por male_03 en sus dos llamadas: sólo cambia
el saludo, no a quién se acerca.

## Versiones anteriores (mismas poses), qué falló y qué se cambió

| Archivo | Llegó | La más cercana | Falsos estáticos | Cambio después |
|---|---|---|---|---|
| `eval_moving.jsonl` (8 poses) | 8/8 | 5/8 | 1 | dos personas a la espalda (el barrido de cabeza deja ~64° sin ver) → búsqueda de 360° siempre; una persona tapada por un pedestal (la profundidad medía el pedestal) → descartar posiciones sobre el mapa del museo vacío |
| `eval_moving_v2.jsonl` (8 poses) | 7/8 | 6/8 | 0 | un actor que pasaba junto a una persona quieta se coló en su pista y la hizo parecer en movimiento → asociación uno a uno por frame y prueba de estática por mayoría (70 % de los puntos) |
| `eval_moving_v3.jsonl` (10 poses) | 10/10 | 8/10 | 3 | se eligió a un actor quieto unos segundos, que el clasificador llamaba "otro" → votación con 5 recortes y descartar "otro" |
| **`eval_moving_v4.jsonl`** (10 poses) | **10/10** | **9/10** (+1 empate) | **0** | — |

`calls.jsonl`: primeras pruebas en `museum_test_people` (se acerca a female02_Sup_Med, estimada a
2 cm de su posición real a 2.8 m) y "No veo a nadie cerca" desde el pasillo inferior (la persona
más cercana, a 6.9 m > `max_distance`), más la primera llamada en el museo dinámico.
