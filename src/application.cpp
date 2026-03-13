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
    window = glfwCreateWindow(1280, 720, "Physics Engine", NULL, NULL);
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
    
    RectangularBody rect1(64.0f, 35.0f, 0.0f, 0.0f, 2.0f, 2.0f, 5.0f, 32.0f, .0f, false);
    rect1.bodyType = 1;
    int r1Id = world.AddBody(rect1);
    
    RectangularBody rect2(78.0f, 40.0f, 0.0f, 0.0f, 2.0f, 2.0f, 5.0f, 32.0f, .0f, false);
    rect2.bodyType = 1;
    int r2Id = world.AddBody(rect2);

    double lastTime = glfwGetTime(); // get time

    double clock = 0;
    int frames = 0;

    DistanceJoint joint(5.0f, 9.8f, r1Id, r2Id, false, &world);
    world.AddDistanceJoint(joint);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        frames+=1;
        double currentTime = glfwGetTime();
        float dt = (float)(currentTime - lastTime);
        lastTime = currentTime;
        clock += dt;

        world.Update(dt, GRAVITY, DRAG_COEFF, BOUND_COR);

        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        world.Render();

        float totalEnergy = world.CalculateTotalEnergy(GRAVITY);

        glColor3f(1.0f, 1.0f, 1.0f);
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Energy: %.2f J", totalEnergy);
        RenderText(buffer, 51.0f, 1.0f, 0.15f);

        glColor3f(1.0f, 1.0f, 1.0f);
        char fps[64];
        snprintf(fps, sizeof(fps), "%6.2f FPS", frames/clock);
        RenderText(fps, 1.0f, 1.0f, 0.15f);

        glColor3f(1.0f, 1.0f, 1.0f);
        char clockbuffer[64];
        snprintf(clockbuffer, sizeof(clockbuffer), "Engine clock: %.2f", clock);
        RenderText(clockbuffer, 101.0f, 1.0f, 0.15f);


        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}