Notes - rtaudio project:

Runs only with a device that has input and output channels (duplex channels >= 1),
so a wired headset (USB or TRRS).

Windows terminal commands:

cmake -B build -S .

cd rtaudio-6.0.1
cd..\..\..
cmake --build build --config Release

cd build\tests\Release
audioprobe

cd build\tests\Release
duplex 1 44100 1 0

cd..\..\..\..

Code changes (except: duplex.cpp, somefunc_2025.cpp, plot_dump.py):

CMakeLists.txt -> option(BUILD_SHARED_LIBS "Build as shared library" OFF) # changed by me to OFF

tests\CMakeLists.txt -> add_executable(duplex duplex.cpp somefunc_2025.cpp) # somefunc_2025.cpp added by me