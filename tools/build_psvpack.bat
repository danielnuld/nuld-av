@echo off
rem psvpack.exe for Windows, with Visual Studio's Build Tools: run tools\build_psvpack.bat
rem (tools/livesrv.py and tools/mkpsv.py use it when it is there, and WSL's psvpack otherwise)
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /O2 /W3 /Fe:"%~dp0psvpack.exe" /Fo:"%TEMP%\psvpack.obj" "%~dp0psvpack.c"
