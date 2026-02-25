import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path(
    "/home/sooraj/Documents/git_personal/Biomechanics_math_modelling/mujoco_xml/waist_mujoco/waist_mechanism_pitch.xml"
)
data = mujoco.MjData(model)

dt = model.opt.timestep

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # camera initial view
    viewer.cam.azimuth = 225
    viewer.cam.elevation = -20
    viewer.cam.distance = 2.0
    viewer.cam.lookat[:] = [0, 0, 0.1]
    
    start_clock = time.time()
    next_time  = start_clock

    time_log = []
    torque_log = []

    while viewer.is_running():

        # Control
        t = data.time
        
        roll = 0.0*np.pi/180 * np.cos(2*np.pi/4*t)
        pitch = 10.0*np.pi/180 * np.sin(2*np.pi/4*t)
        yaw = 40.0*np.pi/180 * np.sin(2*np.pi/4*t)
        
        data.ctrl[0] = roll
        data.ctrl[1] = pitch
        data.ctrl[2] = yaw

        # Step simulation
        mujoco.mj_step(model, data)
        
        # Real-time sync
        next_time += dt
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        # Sim time and real time
        sim_time  = data.time
        elapsed_time = time.time() - start_clock
        # print(f"Simulation time: {sim_time:.3f} | Real time: {elapsed_time:.3f}")
        
        # # Joint Torque extraction
        # torque1 = data.qfrc_actuator[0]*1000
        # torque2 = data.qfrc_actuator[1]*1000
        
        # # print(f"T_pitch: {torque1:.5f} | T_roll: {torque2:.5f}")
        
        # time_log.append(data.time)
        # torque_log.append(torque2)

        viewer.sync()

# plt.figure()
# plt.plot(time_log, torque_log)
# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.xlim([0, 20.0])
# plt.ylim([-5.0, 5.0])
# plt.title("Pitch Joint Torque vs Time")
# plt.grid(True)
# plt.show()