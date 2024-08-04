@echo off
setlocal enabledelayedexpansion

g++ --version

for %%f in ("%~dp0..") do set root=%%~ff
echo Got root of repository: %root%

set params=-Wall -I src/include -std=c++20
if "%1"=="debug" (
    set params=%params% -g -D DEBUG
    echo Building in debug mode
) else (
    set params=%params% -O3
)

if not exist %root%\build mkdir %root%\build
if not exist %root%\result mkdir %root%\result
echo Compiling %root%/src/main.cpp to %root%/build/main.exe
echo Running "g++ %params% %root%/src/main.cpp -o %root%/build/main.exe"
g++ %params% %root%/src/main.cpp -o %root%/build/main.exe
