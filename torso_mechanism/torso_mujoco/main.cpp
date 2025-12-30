#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// --------------------------------------------------------------
// --- Global variables
// --------------------------------------------------------------
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;

// Control targets (in degrees)
double target_pitch_deg = 0.0;
double target_roll_deg  = 0.0;

// Control speed (degrees per second)
const double control_speed = 90.0;  // 90° per second

// --------------------------------------------------------------
// --- MuJoCo controller callback
// --------------------------------------------------------------
void controller(const mjModel* m, mjData* d)
{
    // Convert to radians
    double desired_pitch = target_pitch_deg * mjPI / 180.0;
    double desired_roll  = target_roll_deg  * mjPI / 180.0;

    // Clamp to actuator control ranges
    double pitch_min = m->actuator_ctrlrange[0];
    double pitch_max = m->actuator_ctrlrange[1];
    double roll_min  = m->actuator_ctrlrange[2];
    double roll_max  = m->actuator_ctrlrange[3];

    if (desired_pitch < pitch_min) desired_pitch = pitch_min;
    if (desired_pitch > pitch_max) desired_pitch = pitch_max;
    if (desired_roll  < roll_min)  desired_roll  = roll_min;
    if (desired_roll  > roll_max)  desired_roll  = roll_max;

    // Apply control signals
    d->ctrl[0] = desired_pitch;  // pitch actuator
    d->ctrl[1] = desired_roll;   // roll actuator
}

// --------------------------------------------------------------
// --- Keyboard callback
// --------------------------------------------------------------
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS || act == GLFW_REPEAT)
    {
        double step = control_speed * 0.01; // small step per key press
        switch (key)
        {
            case GLFW_KEY_UP:    target_pitch_deg += step; break;
            case GLFW_KEY_DOWN:  target_pitch_deg -= step; break;
            case GLFW_KEY_RIGHT: target_roll_deg  += step; break;
            case GLFW_KEY_LEFT:  target_roll_deg  -= step; break;

            case GLFW_KEY_BACKSPACE:
                mj_resetData(m, d);
                mj_forward(m, d);
                target_pitch_deg = 0.0;
                target_roll_deg  = 0.0;
                printf("Simulation reset.\n");
                return;
        }
    }
}

// --------------------------------------------------------------
// --- Main
// --------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " model.xml" << std::endl;
        return 1;
    }

    // Load MuJoCo model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(argv[1], nullptr, error, 1000);
    if (!m)
    {
        std::cerr << "Load error: " << error << std::endl;
        return 1;
    }

    // Make data
    d = mj_makeData(m);

    // Init GLFW
    if (!glfwInit()) mju_error("Could not initialize GLFW");

    // Create window
    window = glfwCreateWindow(1200, 900, "MuJoCo Pitch/Roll Control", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Camera settings: free isometric view
    cam.type      = mjCAMERA_FREE;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.05;
    cam.distance   = 1.0;
    cam.azimuth    = 135.0;
    cam.elevation  = -30.0;

    // Set keyboard callback
    glfwSetKeyCallback(window, keyboard);

    // Assign custom controller
    mjcb_control = controller;

    // Main simulation loop
    while (!glfwWindowShouldClose(window))
    {
        // Step simulation
        mj_step(m, d);

        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // Update scene
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        // Render
        mjrRect viewport = {0, 0, width, height};
        mjr_render(viewport, &scn, &con);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();

    return 0;
}
