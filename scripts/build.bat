@echo off
setlocal enabledelayedexpansion

g++ --version

for %%f in ("%~dp0..") do set root=%%~ff
echo Got root of repository: %root%
if not exist %root%\build mkdir %root%\build

if "%1"=="alglib" (
    echo Building ALGLIB
    for %%f in (%root%\extern\alglib-cpp\src\*.cpp) do (
        set command=g++ -c -O3 -D AE_CPU=AE_INTEL -m avx2 -m fma -std=c++20 %%f -o %root%/build/%%~nf.o
        echo Running "!command!"
        !command!
    )

) else (
    echo Building main.exe
    set params=-Wall -I src/include -I extern/alglib-cpp/src -std=c++20
    if "%1"=="debug" (
        set params=!params! -g -D DEBUG
        echo Building in debug mode

    ) else (
        set params=!params! -O3
        if "%1"=="profile-generate" (
            set params=!params! -f profile-generate
            echo Building in profile mode

        ) else if "%1"=="profile-use" (
            set params=!params! -f profile-use
            echo Building using generated profile data

        ) else echo Building normally
    )

    set command=g++ !params! %root%/src/main.cpp %root%/build/*.o -o %root%/build/main.exe
    echo Running "!command!"
    !command!
)
