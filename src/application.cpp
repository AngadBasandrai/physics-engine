#include "GLFW/glfw3.h"
#include "core/physicsWorld.h"
#include "core/joints/distanceJoint.h"
#include "constants.h"
#include "util/textRenderer.h"
#include <cstdio>

int main(void)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Physics Engine", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    // change coords to set top left = 0,0 bottom right = 64,48
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
 
    glOrtho(0.0, SCREEN_WIDTH, SCREEN_HEIGHT, 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    PhysicsWorld world;
    
    RigidBody rect1(31.0f, 23.0f, 0.0f, 0.0f, 2.0f, 2.0f, 5.0f, ShapeType::ELLIPSE, 32.0f, .0f, true);
    int r1Id = world.AddBody(rect1);
    
    RigidBody rect2(35.0f, 28.0f, 0.0f, 0.0f, 2.0f, 2.0f, 5.0f, ShapeType::ELLIPSE, 32.0f, 1.0f, false);
    int r2Id = world.AddBody(rect2);

    double lastTime = glfwGetTime(); // get time

    double clock = 0;

    bool jointed = false;

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        double currentTime = glfwGetTime();
        float dt = (float)(currentTime - lastTime) * 3;
        lastTime = currentTime;
        clock += (dt/3);

        if (!jointed && clock > 1.0f){
            DistanceJoint joint(5.0f, 1.0f, r1Id, r2Id, true, &world);
            world.AddDistanceJoint(joint);
        }

        world.Update(dt, GRAVITY, VISCOSITY, BOUND_COR);

        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        world.Render();

        float totalEnergy = world.CalculateTotalEnergy(GRAVITY);

        glColor3f(1.0f, 1.0f, 1.0f);
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Energy: %.2f J", totalEnergy);
        RenderText(buffer, 1.0f, 1.0f, 0.15f);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}