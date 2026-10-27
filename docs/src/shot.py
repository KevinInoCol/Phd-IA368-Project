"""Spawn a static camera in Gazebo at x y z roll pitch yaw, grab one frame, save PNG, remove it.
usage: shot.py name x y z roll pitch yaw hfov out.png [w h]"""
import sys, time, subprocess, threading
import numpy as np, cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
name, x, y, z, R, Pp, Y, fov, out = sys.argv[1:10]
w, h = (sys.argv[10], sys.argv[11]) if len(sys.argv) > 11 else ("1600", "900")
topic = f"/shot/{name}"
sdf = f"""<sdf version='1.9'><model name='{name}'><static>true</static><pose>{x} {y} {z} {R} {Pp} {Y}</pose>
<link name='l'><sensor name='c' type='camera'><topic>{topic}</topic><update_rate>5</update_rate><always_on>1</always_on>
<camera><horizontal_fov>{fov}</horizontal_fov><image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
<clip><near>0.1</near><far>100</far></clip></camera></sensor></link></model></sdf>"""
req = 'sdf: "' + sdf.replace('"', '\\"').replace("\n", " ") + '"'
subprocess.run(["gz", "service", "-s", "/world/default/create", "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req], check=True)
got, n = threading.Event(), [0]
def cb(msg):
    n[0] += 1
    if n[0] < 4 or got.is_set():  # skip the first frames (textures still loading)
        return
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    cv2.imwrite(out, cv2.cvtColor(a, cv2.COLOR_RGB2BGR)); got.set()
node = Node(); node.subscribe(Image, topic, cb)
ok = got.wait(60)
subprocess.run(["gz", "service", "-s", "/world/default/remove", "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", f'name: "{name}" type: MODEL'])
print("saved" if ok else "timeout", out)
