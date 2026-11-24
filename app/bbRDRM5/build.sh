#!/bin/bash
# Builds only the bbRDRM5 app (other apps and the shared Makefile are not touched)
cd "$(dirname "$0")" && mkdir -p ../../applicationsBin/bbRDRM5 && make clean >/dev/null 2>&1
make GLOBAL_INCLUDES="-I/usr/local/include -I/opt/local/include -I/usr/X11/include" \
     GLOBAL_CCFLAGS="-O2 -g -Wall -std=c++17 -DTINYXML_USE_STL -DTIXML_USE_STL -fno-stack-protector" \
     GLOBAL_LIBS="-L./ -L/usr/local/lib -L/usr/X11/lib -lsimBlinkyBlocks -lmuparser -lglut -lGL -lGLEW -lGLU -lpthread -ldl -lstdc++ -lm" \
     2>&1 | grep -E "error|warning: "
ls -la ../../applicationsBin/bbRDRM5/bbRDRM5
