# ------------------------------------------------------------
# Paths
# ------------------------------------------------------------
MUJOCO_INC = $(HOME)/Downloads/mujoco_latest/include
MUJOCO_LIB = $(HOME)/Downloads/mujoco_latest/lib

# ------------------------------------------------------------
# Compiler and flags
# ------------------------------------------------------------
COMPILER = g++
CFLAGS = -I$(MUJOCO_INC)
LDFLAGS = -L$(MUJOCO_LIB) -lmujoco -lglfw -lGL -lGLU -lX11 -lpthread -ldl

# ------------------------------------------------------------
# Build target
# ------------------------------------------------------------
all:
	$(COMPILER) main.cpp $(CFLAGS) $(LDFLAGS) -o main
