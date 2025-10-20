#include "GLFW/glfw3.h"
#include "core/physicsWorld.h"
#include "constants.h"

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

    // change coords to set top left = 0,0 bottom right = 640,480
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 64.0, 48.0, 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    PhysicsWorld world;
    
    RigidBody rect1(10.0f, 2.0f, 5.0f, 0.0f, 3.0f, 3.0f, 1.0f, ShapeType::RECTANGLE);
    rect1.angularVelocity = 1.0f; 
    world.AddBody(rect1);
    
    RigidBody rect2(54.0f, 2.0f, -5.0f, 0.0f, 4.0f, 2.5f, 1.0f, ShapeType::RECTANGLE);
    rect2.angularVelocity = -1.0f;
    world.AddBody(rect2);
    
    RigidBody ellipse1(32.0f, 5.0f, 0.0f, 0.0f, 3.0f, 1.5f, 1.0f, ShapeType::ELLIPSE);
    ellipse1.angularVelocity = .4f; 
    world.AddBody(ellipse1);
    
    RigidBody ellipse2(20.0f, 10.0f, 3.0f, 2.0f, 2.0f, 2.0f, 0.8f, ShapeType::ELLIPSE);
    world.AddBody(ellipse2);

    double lastTime = glfwGetTime(); // get time

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        double currentTime = glfwGetTime();
        float dt = (float)(currentTime - lastTime) * 3;
        lastTime = currentTime;

        world.Update(dt, GRAVITY, VISCOSITY, BOUNDCOR);

        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        world.Render();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}