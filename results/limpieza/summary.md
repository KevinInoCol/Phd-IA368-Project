# Limpieza del museo y base de carga — resultados (2026-11-16)

| Versión | Limpio | Sala oeste | Esquina base | Pasillo sur | Sala norte | Entrada | Tiempo (sim.) | Recorrido | Recargas en la misión |
|---|---|---|---|---|---|---|---|---|---|
| v1: sólo zigzag, mapa del láser | **80.9 %** | 98 % | 88 % | 87 % | 72 % | 69 % | 13.3 min | 151 m | 1 |
| v2: + repaso, mapa del láser | **92.9 %** | 98 % | 96 % | 99 % | 90 % | 83 % | 22.1 min | 223 m | 2 |
| final, corrida 1 | **85.7 %** | 100 % | 98 % | 88 % | 81 % | 75 % | 21.7 min | 214 m | 2 |
| final, corrida 2 | **84.8 %** | 100 % | 91 % | 86 % | 82 % | 75 % | 22.9 min | 218 m | 2 |
| final, corrida 3 | **81.5 %** | 77 % | 98 % | 88 % | 79 % | 73 % | 23.0 min | 217 m | 2 |
| **final, media de 3** | **84.0 %** | 92 % | 95 % | 87 % | 80 % | 74 % | 22.5 min | 216 m | 6 |
| final, museo con gente caminando | **87.1 %** | 100 % | 97 % | 92 % | 86 % | 69 % | 22.3 min | 215 m | 2 |

Mundo `museum_test_people` (4 personas quietas) salvo la última fila (`museum_with_people_moving`: 10
quietas y 16 caminando). Batería inicial 45 % para forzar recargas. v1 y v2 usaban el mapa del láser
(41 m² de suciedad); las finales, el mapa de navegación con los salientes (39 m², 38 m² con la gente
caminando). Detalle por corrida: `results/limpieza/*/summary.json` y `cleaning.png`.

| Zona | sala_oeste | esquina_base | pasillo_sur | sala_norte | entrada |
|---|---|---|---|---|---|
| RMSE de AMCL (media de las 3) | 0.13 m | 0.15 m | 0.14 m | 0.26 m | 0.23 m |

- **El repaso suma ~12 puntos** (v1 81 % → v2 93 %): bordes y tramos que Nav2 abandonó.
- **8 de 8 recargas en mitad de la misión** salieron bien: pausa, Nav2 hasta la base, haz, carga,
  salida y vuelta al mismo tramo. Con la reserva según la distancia, desde la sala norte Pepper
  decidió volver con 22 % (con el umbral fijo de 20 % llegaba a la base con 8 %).
- **Lo que queda sucio es sobre todo localización**: en la sala norte y la entrada AMCL se
  equivoca ~0.24 m (0.13–0.15 m en el resto). Pepper recorre los carriles donde cree estar, la
  pasada queda corrida y una franja del borde no se limpia; el repaso no la ve, porque usa la misma
  pose. Por eso la entrada se queda en ~70–75 %.
- **Con gente caminando no empeora** (87 %): los tramos se cortan con el costmap y Nav2 esquiva.
- **Hallazgo: obstáculos que el láser no ve.** En dos corridas Pepper chocó al volver a la base
  con la punta de un panel colgante en (−1.9, −5.7): a la altura de los láseres el panel termina 1 m
  antes que a la de los hombros. Quedó doblado por la cadera (cabeza a 0.45 m) y no pudo acoplarse
  (`results/limpieza/choque_panel/`). En el museo hay 13 m² así (paneles, salientes, vitrinas):
  `pepper_museum/tools/museum_nav_map.py` los suma al mapa del hokuyo (`maps/museum_nav`), que es
  ahora el mapa de Nav2/AMCL en `cleaning_demo`. La base también se separó 0.5 m de la pared
  oeste, que está inclinada. Con eso no volvió a haber choques con paneles.
- **Roce con una persona** (corrida 3): el carril pasó a 0.34 m de una persona quieta y el brazo
  de Pepper la tocó; la cabeza bajó a 0.66 m y se recuperó en 3 s. El láser ve las piernas, no
  los brazos.

**Siguiente versión (implementada, sin evaluar todavía)**: carriles pegados al borde de cada zona
(`edge_inset` 0.12 m, contra el corrimiento de AMCL) y 0.45 m de margen a todo obstáculo del costmap
que no está en el mapa (`people_margin`).
