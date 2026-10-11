@echo off
rem Configure + build OdrForge with MSVC (x64) and Ninja. Usage: build.bat [Release|Debug]
setlocal
set CFG=%1
if "%CFG%"=="" set CFG=Release
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cmake -S "%~dp0." -B "%~dp0build\%CFG%" -G Ninja -DCMAKE_BUILD_TYPE=%CFG% || exit /b 1
rem a just-run executable can stay locked for a while on Windows: move it aside
if exist "%~dp0build\%CFG%\osm2xodr.exe" move /y "%~dp0build\%CFG%\osm2xodr.exe" "%~dp0build\%CFG%\osm2xodr.old.exe" >nul 2>&1
cmake --build "%~dp0build\%CFG%" || exit /b 1
ctest --test-dir "%~dp0build\%CFG%" --output-on-failure
