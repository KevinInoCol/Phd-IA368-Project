# Fase 3 — voz sin micrófono (2026-10-21)

`pepper_visitors/tools/voice_test.py`: 10 frases (6 llamadas, 4 no) sintetizadas con Piper en dos
voces (es_ES-davefx-medium, es_MX-claude-high) suenan en un sumidero virtual de PipeWire
(`pepper_test_audio`); `voice_listener` lo escucha con `capture_sink:=true`. Con `--snr`, ruido
rosa mezclado. Latencia: desde el final del audio hasta `/pepper/visitor_call` (incluye los 0.8 s
de silencio que cierran la frase).

| Condición | Llamadas detectadas | Falsas llamadas | «Pepper» bien escrito | Latencia |
|---|---|---|---|---|
| sin hotwords, limpio | 12/12 | 0/8 | 2/6 | 1.07 s |
| sin hotwords, SNR 10 dB | 11/12 | 0/8 | 1/6 | 1.22 s |
| sin hotwords, SNR 5 dB | 12/12 | 0/8 | 1/6 | 1.19 s |
| hotwords «Pepper», limpio | 12/12 | 0/8 | 5/6 | 1.15 s |
| hotwords «Pepper», SNR 10 dB | 12/12 | 0/8 | 3/6 | 1.25 s |
| hotwords «Pepper», SNR 5 dB | 11/12 | 0/8 | 2/6 | 1.41 s |

Se deja `hotwords: "Pepper"` por defecto: la detección es igual (la lista de palabras de llamada
ya incluye "pepe", "peper", "hola", "oye"…), pero el nombre sale bien escrito el doble de veces
y no aparece ninguna falsa llamada. Los fallos: a 10 dB sin hotwords "Pepper" → "Espera"; a 5 dB
con hotwords, "Pepper," inicial desapareció (la frase no tenía otra palabra de llamada).

## De punta a punta

`visitors_demo.launch.py world:=museum_with_people_moving x:=7.0 y:=-3.0 voice:=true
device:=pepper_test_audio capture_sink:=true` y la frase «Hola Pepper, ¿me puedes contar algo
de esta pieza?» (voz es_MX) en el sumidero: llamada detectada 0.6 s después del final de la
frase; búsqueda con giro de 180°; elige a male03_Inf_Med_2 (la estática más cercana, a su
espalda, 2.0 m); Nav2 la deja a 1 m y saluda. 39 s desde el inicio de la frase
(`e2e_calls.jsonl`).

Pendiente con el micrófono real: el nivel de la voz y el ruido de la sala (`energy_factor`,
`min_rms`) y el acento del usuario.
