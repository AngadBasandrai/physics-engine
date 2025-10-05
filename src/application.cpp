#include "GLFW/glfw3.h"
#include "core/physicsWorld.h"

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

    // set up sim
    PhysicsWorld world;
    world.AddBody({ 2.0f, 2.0f, 5.0f, 0.0f, 2.0f, 2.0f, 1.0f, ShapeType::RECTANGLE});
    world.AddBody({ 62.0f, 2.0f, -5.0f, 0.0f, 2.0f, 2.0f, 1.0f, ShapeType::ELLIPSE32 });

    double lastTime = glfwGetTime(); // get time

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        double currentTime = glfwGetTime();
        float dt = (float)(currentTime - lastTime) * 3;
        lastTime = currentTime;

        world.Update(dt);

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