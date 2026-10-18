#!/usr/bin/env python3
"""Convierte los mundos del museo de la tesis (Gazebo Classic, ROS 1) a Gazebo Harmonic.

    python3 tools/convert_worlds.py "<pepper_sim_ws>/src/pepper_virtual/pepper_gazebo_plugin/worlds"

Lee museum.world, museum_with_people.world y museum_with_people_moving.world y escribe
worlds/*.sdf en este paquete. Se mantiene el contenido (museo, personas, luces, actores y sus
trayectorias) y cambia lo que en Harmonic es distinto:

  - Física, sensores y escena: los sistemas de gz-sim, igual que los mundos de la base
    (pepper_gazebo_plugin/worlds/simple_office_with_people.world).
  - El museo (model iscas_museum con su malla) pasa a <include> de models/iscas_museum.
  - Actores: la animación es model://actor_walk/meshes/walk.dae (en Classic venía de la
    instalación de Gazebo) y se quita ActorCollisionsPlugin. Ese plugin daba colisión a los
    actores para que los láseres de rayos de Gazebo Classic los vieran; en Harmonic los
    láseres de Pepper son gpu_lidar (por render) y ven la malla del actor sin él.
    Los actores no chocan físicamente con el robot.
  - En Harmonic el origen de walk.dae está en la cadera: con z = 0 (como en la tesis) el actor
    queda hundido en el suelo hasta el pecho. Se suma ACTOR_Z a la z de cada waypoint, como en
    los ejemplos de actores de gz-sim.
  - loop y auto_start explícitos (en Classic eran true por defecto).
  - Se descartan los bloques <state> (instantánea de la simulación guardada por Gazebo).
"""

import copy
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "worlds")
WALK = "model://actor_walk/meshes/walk.dae"
MUSEUM_POSE = "1.86134 -2.16704 0 0 0 0"
ACTOR_Z = 1.0

HEADER = """<?xml version="1.0" ?>
<!-- {title}
     Convertido de la tesis (pepper_sim_ws, Gazebo Classic: worlds/{src}) con
     pepper_museum/tools/convert_worlds.py. No editar a mano: regenerar. -->
<sdf version="1.6">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://iscas_museum</uri>
      <name>iscas_museum</name>
      <pose>{museum_pose}</pose>
    </include>
"""

FOOTER = """
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>25.5204 -24.0365 14.7272 0 0.275643 2.3562</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
"""

WORLDS = [
    ("museum.world", "museum.sdf", "Museo ISCAS vacío."),
    ("museum_with_people.world", "museum_with_people.sdf", "Museo ISCAS con 23 personas estáticas."),
    ("museum_with_people_moving.world", "museum_with_people_moving.sdf",
     "Museo ISCAS con 10 personas estáticas y 16 actores caminando (trayectorias de la tesis)."),
]


def text(e, indent):
    ET.indent(e, space="  ", level=indent // 2)
    return " " * indent + ET.tostring(e, encoding="unicode").strip() + "\n"


def convert_actor(old):
    new = ET.Element("actor", name=old.get("name"))
    if old.find("pose") is not None:
        new.append(copy.deepcopy(old.find("pose")))
    skin = ET.SubElement(new, "skin")
    ET.SubElement(skin, "filename").text = WALK
    traj_type = old.find("script/trajectory").get("type")
    anim = ET.SubElement(new, "animation", name=traj_type)
    ET.SubElement(anim, "filename").text = WALK
    ET.SubElement(anim, "interpolate_x").text = "true"
    script = ET.SubElement(new, "script")
    ET.SubElement(script, "loop").text = old.findtext("script/loop") or "true"
    ET.SubElement(script, "auto_start").text = old.findtext("script/auto_start") or "true"
    tr = copy.deepcopy(old.find("script/trajectory"))
    for wp in tr.findall("waypoint"):
        p = [float(v) for v in wp.findtext("pose").split()]
        p[2] += ACTOR_Z
        wp.find("pose").text = " ".join(f"{v:g}" for v in p)
    script.append(tr)
    return new


def convert(src_dir, src, dst, title):
    world = ET.parse(os.path.join(src_dir, src)).getroot().find("world")
    body, counts = [], {"include": 0, "light": 0, "actor": 0}
    for c in world:
        if c.tag == "include":
            body.append(text(copy.deepcopy(c), 4))
        elif c.tag == "light" and c.get("name") != "sun":
            body.append(text(copy.deepcopy(c), 4))
        elif c.tag == "actor":
            body.append(text(convert_actor(c), 4))
        else:
            continue
        counts[c.tag] += 1
    with open(os.path.join(OUT, dst), "w") as f:
        f.write(HEADER.format(title=title, src=src, museum_pose=MUSEUM_POSE))
        f.write("\n" + "\n".join(body))
        f.write(FOOTER)
    print(f"{dst}: {counts['include']} personas, {counts['light']} luces, {counts['actor']} actores")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    os.makedirs(OUT, exist_ok=True)
    for src, dst, title in WORLDS:
        convert(sys.argv[1], src, dst, title)


if __name__ == "__main__":
    main()
