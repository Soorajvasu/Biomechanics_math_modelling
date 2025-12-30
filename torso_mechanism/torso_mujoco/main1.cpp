#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>

// constants
const double PI = 3.141592653589793;

// simulation parameters
const double freq_pitch = 1.0;   // Hz
const double freq_roll  = 0.5;   // Hz
const double amp_pitch  = 0.2;   // radians
const double amp_roll   = 0;   // radians

// MuJoCo objects
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// GLFW window pointer
GLFWwindow* window = nullptr;

// Control callback: compute control inputs at each step
void controller(const mjModel* m, mjData* d)
{
    double t = d->time;

    // find joint IDs
    int id_pitch = mj_name2id(m, mjOBJ_JOINT, "pitch");
    int id_roll  = mj_name2id(m, mjOBJ_JOINT, "roll");

    // sinusoidal targets
    double pitch_target = amp_pitch * sin(2 * PI * freq_pitch * t);
    double roll_target  = amp_roll  * sin(2 * PI * freq_roll  * t);

    // assign to actuator controls (position actuators)
    d->ctrl[0] = pitch_target;
    d->ctrl[1] = roll_target;
}

int main(int argc, const char** argv)
{
    // Load model
    const char* model_path = "torso_mechanism.xml";
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(model_path, nullptr, error, 1000);
    if (!m)
    {
        std::cerr << "Error loading model: " << error << std::endl;
        return 1;
    }

    // Make data
    d = mj_makeData(m);

    // Init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    window = glfwCreateWindow(1200, 900, "Torso Mechanism Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize visualization data structures
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

    // Main simulation loop
    while (!glfwWindowShouldClose(window))
    {
        // Step simulation with control
        controller(m, d);
        mj_step(m, d);

        // Get size of window and update scene
        mjv_updateScene(m, d, &opt, &cam, mjCAT_ALL, &scn);

        // Render
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &scn, &con);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}