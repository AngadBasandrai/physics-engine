ifeq ($(OS),Windows_NT)
    RM = cmd /C del /Q
    EXE = app.exe
    RUN = .\$(EXE)
    LIBS = -lglfw3 -lopengl32 -lgdi32
else
    RM = rm -f
    EXE = app
    RUN = ./$(EXE)
    LIBS = -lglfw -lGL -lX11 -lpthread -lXrandr -lXi -ldl
endif

CXX = g++
CXXFLAGS = -IDependencies/GLFW/include
LDFLAGS = -LDependencies/GLFW/lib $(LIBS)
SOURCES = src/application.cpp src/core/physicsWorld.cpp src/core/joints/distanceJoint.cpp src/collision/collision.cpp src/core/rigidbody.cpp

all: build run

clean:
	$(RM) $(EXE)

build:
	$(CXX) $(SOURCES) $(CXXFLAGS) $(LDFLAGS) -o $(EXE)

run:
	$(RUN)

.PHONY: all build run clean
