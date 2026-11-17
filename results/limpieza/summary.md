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

**Siguiente versión (v4)**: carriles pegados al borde de cada zona (`edge_inset` 0.12 m, contra
el corrimiento de AMCL) y 0.45 m de margen a todo obstáculo del costmap que no está en el mapa
(`people_margin`). Las tres corridas con visitantes (abajo) ya la usan: 91.5 %, 88.2 % y 85.0 %
limpio, aun atendiendo 5–6 llamadas cada una. Falta medirla sin llamadas, para compararla con la
tabla.

## Si le hablan mientras limpia (2026-11-17)

La limpieza no tapa lo de la fase 3: si alguien llama a Pepper mientras limpia (voz por
`voice_listener`, pestaña Visitante de la app o `/pepper/visitor_call`), Pepper deja la limpieza,
busca con la cámara a quien le habló, se acerca, conversa y vuelve a limpiar donde se quedó.

1. `visitor_approach` publica `searching` en `/pepper/visitor_status`. El `cleaner` cancela su
   tramo, apaga el cepillo y pasa a **PAUSED_VISITOR**.
2. Pepper busca como en la fase 3: YOLO sobre la cámara frontal, posición con la cámara de
   profundidad, barrido de cabeza y giro de 180°. Elige a la persona quieta más cercana que el
   clasificador no toma por alguien que camina, va con Nav2 a 1 m y se pone de frente. Si ya estaba
   a menos de 0.75 m (lo llamaron limpiando junto a ella), primero retrocede a 1 m.
3. La charla la lleva Gemini en el bridge de la app. Termina cuando el visitante se despide (el
   bridge publica `/pepper/visitor_done`) o tras 60 s sin que nadie hable (`talk_timeout`).
   Entonces `visitor_approach` publica `done` y deja de mirar a la persona.
4. Con `done`, `nobody`, `nav_failed`, `charging` o `error`, el `cleaner` vuelve al tramo donde
   estaba.

**Con la base de carga**:
- Si lo llaman mientras carga con al menos 40 % (`min_battery`), sale de la base y atiende.
- Con menos de 40 %, o si va camino de la base, contesta "Ahora necesito cargarme… ven a hablar
  conmigo a la base de carga" y no se mueve.
- Mientras atiende, `autodock` aplaza el docking automático salvo bajo 10 % de batería, para no
  irse a mitad de la charla.

Prueba: `CALLS=chat,silent,charging,... GUI=true pepper_cleaning/scripts/run_cleaning.sh <dir>`
(`tools/visitor_calls.py` llama a Pepper durante la limpieza; la charla la simula con tres frases y
una despedida). Tres corridas completas, 16 llamadas (`results/limpieza/visitantes_r*`):

| | Resultado |
|---|---|
| Pausa la limpieza al llamarlo | 13 de 13 llamadas en mitad de la limpieza, 0.2–1.0 s |
| Se acerca a la persona elegida | **11 de 11**, a 0.80–1.29 m y de frente (±3°); una llamada a 0.45 m, ver abajo |
| Vuelve a limpiar | **13 de 13**, a los 61–90 s de la llamada |
| Cepillo durante la pausa | apagado: el % limpio no sube en ninguna pausa |
| En la base | con 14 % no va; con 55–56 % sale de la base, y en el museo con gente atiende |
| Limpieza total, atendiendo 5–6 llamadas | 91.5 %, 88.2 % y 85.0 % (con gente caminando) |

- **Las 5 llamadas sin nadie** fueron cerca de la base o en un pasillo, donde en
  `museum_test_people` no hay gente quieta a menos de 6 m. Pepper buscó, dijo "No veo a nadie
  cerca" y volvió a limpiar.
- **La llamada a 0.45 m** fue en plena limpieza junto a la persona: `visitor_approach` tomaba
  cualquier distancia de 1.3 m o menos como "ya estoy cerca". Ahora, si queda a menos de 0.75 m,
  se aparta a 1 m. En una prueba aparte pasó de 0.50 m a 1.11 m de la persona.
- **Una vez la batería cruzó el umbral durante la charla.** `autodock` esperó a que terminara, y
  luego Pepper fue a cargarse y siguió limpiando.
- **Pendiente, no lo causa esta función.** En la corrida con gente caminando, Pepper iba de camino
  a la sala norte por una ruta de Nav2 que pasó a 0.47 m de una persona quieta. Quedó doblado
  (cabeza a 0.45 m) durante 75 s y se enderezó solo. El radio de Pepper en Nav2 (0.24 m, del repo
  base) no cuenta los brazos. Falta darle más margen sin cerrar los pasillos estrechos.
