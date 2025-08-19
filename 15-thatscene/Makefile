# Compiler and flags
CXX := C:/msys64/ucrt64/bin/g++.exe
CXXFLAGS := -g -std=c++20 -fopenmp -IC:/tools/SDL3-3.2.14/x86_64-w64-mingw32/include
LDFLAGS := -LC:/tools/SDL3-3.2.14/x86_64-w64-mingw32/lib -lmingw32 -lSDL3 -mwindows

# All source directories
SRC_DIRS := src
# All .cpp files (non-recursive per dir)
SRC := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.cc))
# Generate corresponding .o files under build/
OBJ := $(patsubst %.cpp,build/%.o,$(SRC))
# Output binary
BIN := build/demo.exe

# Default target
all: $(BIN)

# Link all object files
$(BIN): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to compile .cpp into .o (preserving subdirs)
build/%.o: %.cpp
	@if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target
clean:
	rm -f $(OBJ) $(BIN)

# Debug info
info:
	@echo SRC = $(SRC)
	@echo OBJ = $(OBJ)
