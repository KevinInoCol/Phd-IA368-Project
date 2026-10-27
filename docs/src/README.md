# Fuentes de las páginas de `docs/`

- `arquitectura.html` = `arquitectura.tpl.html` + imágenes de `img/` incrustadas:
  `python3 build.py . ../arquitectura.html` (desde esta carpeta).
- `nodos-topicos.html` = `nodos.tpl.html` + grafos de Graphviz (`dot`) generados por `gen_graph.py`:
  `python3 gen_graph.py nodos.tpl.html ../nodos-topicos.html`. Los nodos, tópicos, Hz y conexiones
  están en las listas del principio de `gen_graph.py` (medidos el 2026-10-27 con `ros2 node info`).
- `shot.py`: pone una cámara en Gazebo (mundo `default`), guarda un cuadro y la quita; así salieron
  las vistas del museo (`shot.py iso -11 -16 15 0 0.62 0.86 1.05 iso.png 1600 900`).
