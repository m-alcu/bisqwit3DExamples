I have gathered some interesting examples from https://bisqwit.iki.fi/jutut/kuvat/programming_examples/polytut/

All examples are retrieved and minimal modified

SDL2 -> SDL3, yoy should install SDL3 in your folder, look at makefile to guess where to install your c++ compiler and sdl3:

CXX := C:/msys64/ucrt64/bin/g++.exe
CXXFLAGS := -g -std=c++20 -fopenmp -IC:/tools/SDL3-3.2.14/x86_64-w64-mingw32/include
LDFLAGS := -LC:/tools/SDL3-3.2.14/x86_64-w64-mingw32/lib -lmingw32 -lSDL3 -mwindows

You can launch from vscode.
