"""rviz/cleaning.rviz = pepper_nav.rviz (de la base) + las capas de la limpieza.

    python3 tools/make_rviz.py

Añade: piso sucio (/pepper/dirt/grid), base de carga y haz IR (/pepper/dock/markers), lugares
(/pepper/places/markers) y el plan de limpieza (/pepper/clean_plan). No modifica la base.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
base = os.path.join(get_package_share_directory("pepper_nav"), "rviz", "pepper_nav.rviz")
cfg = yaml.safe_load(open(base))
displays = cfg["Visualization Manager"]["Displays"]
qos = {"Depth": 5, "Durability Policy": "Transient Local", "History Policy": "Keep Last",
       "Reliability Policy": "Reliable"}
displays += [
    {"Class": "rviz_default_plugins/Map", "Name": "Piso sucio", "Enabled": True, "Alpha": 0.6,
     "Color Scheme": "costmap", "Draw Behind": False, "Topic": {**qos, "Value": "/pepper/dirt/grid"},
     "Update Topic": {**qos, "Durability Policy": "Volatile", "Value": "/pepper/dirt/grid_updates"},
     "Use Timestamp": False, "Value": True},
    {"Class": "rviz_default_plugins/MarkerArray", "Name": "Base de carga", "Enabled": True,
     "Namespaces": {}, "Topic": {**qos, "Value": "/pepper/dock/markers"}, "Value": True},
    {"Class": "rviz_default_plugins/MarkerArray", "Name": "Lugares", "Enabled": True,
     "Namespaces": {}, "Topic": {**qos, "Value": "/pepper/places/markers"}, "Value": True},
    {"Class": "rviz_default_plugins/Path", "Name": "Plan de limpieza", "Enabled": True,
     "Color": "25; 255; 140", "Alpha": 0.8, "Line Style": "Lines", "Line Width": 0.03,
     "Buffer Length": 1, "Head Diameter": 0.3, "Head Length": 0.2, "Length": 0.3, "Radius": 0.03,
     "Shaft Diameter": 0.1, "Shaft Length": 0.1, "Pose Style": "None", "Offset": {"X": 0, "Y": 0, "Z": 0.02},
     "Pose Color": "255; 85; 255", "Topic": {**qos, "Value": "/pepper/clean_plan"}, "Value": True},
]
out = os.path.join(HERE, "..", "rviz", "cleaning.rviz")
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w") as f:
    f.write("# Generado por tools/make_rviz.py: pepper_nav.rviz + capas de pepper_cleaning\n")
    yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
print(out, len(displays), "capas")
