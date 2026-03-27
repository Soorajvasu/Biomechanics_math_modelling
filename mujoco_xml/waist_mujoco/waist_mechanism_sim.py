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
        
        joint18 = 0.0*np.pi/180 * np.cos(2*np.pi/4*t)
        joint19 = 15.0*np.pi/180 * np.sin(2*np.pi/4*t)
        yaw = 0.0*np.pi/180 * np.sin(2*np.pi/4*t)
        
        data.ctrl[0] = joint18
        data.ctrl[1] = joint19
        data.ctrl[2] = yaw

        # Step simulation
        mujoco.mj_step(model, data)

        # 🔹 Compute inverse dynamics (key addition)
        mujoco.mj_inverse(model, data)

        # 🔹 Extract clean joint torque
        torque2 = data.qfrc_inverse[1]   # NO scaling

        # Time sync
        next_time += dt
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Log data
        time_log.append(data.time)
        torque_log.append(torque2)

        viewer.sync()

# Plot
plt.figure()
plt.plot(time_log, torque_log)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.xlim([0, 20.0])
plt.ylim([-5.0, 5.0])
plt.title("Pitch Joint Torque vs Time (Inverse Dynamics)")
plt.grid(True)
plt.show()