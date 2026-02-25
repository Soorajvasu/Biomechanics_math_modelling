import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path(
    "/home/soorajvasu/Documents/git_personal/Biomechanics_math_modelling/mujoco_xml/waist_mujoco/waist_mechanism_pitch.xml"
)
data = mujoco.MjData(model)

dt = model.opt.timestep

with mujoco.viewer.launch_passive(model, data) as viewer:

    start_wall = time.time()
    next_time  = start_wall

    while viewer.is_running():

        viewer.cam.azimuth = 225
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0.1]

        # Control
        t = data.time
        
        roll = 0.0*np.pi/180 * np.cos(2*np.pi/4*t)
        pitch = 015.0*np.pi/180 * np.sin(2*np.pi/4*t)
        yaw = 0.0
        
        data.ctrl[0] = roll
        data.ctrl[1] = pitch
        data.ctrl[2] = yaw

        # Step simulation
        mujoco.mj_step(model, data)

        # --- Real-time sync ---
        next_time += dt
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Print time
        sim_time  = data.time
        real_time = time.time() - start_wall
        print(f"Sim time: {sim_time:.3f} | Real time: {real_time:.3f}")
        
        torque1 = data.qfrc_actuator[0]*10000 
        torque2 = data.qfrc_actuator[1]*10000  
        
        time_log = []
        torque_log = []
        
        # time_log.append(data.time)
        # torque_log.append(data.qfrc_actuator[0])

        viewer.sync()

# plt.figure()
# plt.plot(time_log, torque_log)
# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.title("Pitch Joint Torque vs Time")
# plt.grid(True)
# plt.show()



# dt = model.opt.timestep  # 0.001

# with mujoco.viewer.launch_passive(model, data) as viewer:

#     start_wall = time.time()

#     while viewer.is_running():

#         step_start = time.time()

#         t = data.time

#         viewer.cam.azimuth = 225
#         viewer.cam.elevation = -20
#         viewer.cam.distance = 2.0
#         viewer.cam.lookat[:] = [0, 0, 0.1]

#         # --- Desired trajectory (1 Hz) ---
#         roll_des  = 0.0
#         pitch_des = np.deg2rad(15.0) * np.sin(2*np.pi*0.25*t)
#         yaw_des   = 0.0

#         data.ctrl[0] = roll_des
#         data.ctrl[1] = pitch_des
#         data.ctrl[2] = yaw_des

#         mujoco.mj_step(model, data)
#         viewer.sync()

#         # # --- Real-time pacing ---
#         # elapsed = time.time() - step_start
#         # if elapsed < dt:
#         #     time.sleep(dt - elapsed)

#         # --- Print once per second only ---
#         if int(data.time) != int(data.time - dt):
#             print(f"Sim time: {data.time:.2f}s | "
#                   f"Real elapsed: {time.time() - start_wall:.2f}s")