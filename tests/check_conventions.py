"""One-off: verify MuJoCo free-joint qvel angular frame convention."""
import numpy as np
import mujoco

XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.2 0.1 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)

# rotate body 90 deg about world z, then set qvel[3:6] = (1,0,0)
d.qpos[3:7] = [np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]  # yaw +90
d.qvel[3:6] = [1.0, 0.0, 0.0]
mujoco.mj_forward(m, d)

res = np.zeros(6)
mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, 1, res, 0)  # world
print("qvel[3:6]      :", d.qvel[3:6])
print("world ang vel  :", res[:3])
res_l = np.zeros(6)
mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, 1, res_l, 1)  # local
print("local ang vel  :", res_l[:3])
# If world == (1,0,0): qvel is WORLD frame. If local == (1,0,0): BODY frame.
