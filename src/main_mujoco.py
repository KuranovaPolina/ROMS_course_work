import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
from lxml import etree # lxml is needed to change .xml models in script
import math

model = mujoco.MjModel.from_xml_path("/Users/polinakuranova/uni/robot_simulation/ROMS/ROMS_course_work/ROMS_course_work/model/model.xml")
data = mujoco.MjData(model)

start_time = 0

def control_func_pd(model, data):
  pass

with mujoco.viewer.launch_passive(model, data) as viewer:  
  start_time = time.time()
  step_start = start_time

  mujoco.set_mjcb_control(control_func_pd)

  while viewer.is_running():
    step_start = time.time()
    x = data.body("car").xpos

    mujoco.mj_step(model, data)
    viewer.sync()
 