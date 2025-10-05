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

all: build run

clean:
	$(RM) $(EXE)

build:
	$(CXX) src/application.cpp $(CXXFLAGS) $(LDFLAGS) -o $(EXE)

run:
	$(RUN)

.PHONY: all build run clean
