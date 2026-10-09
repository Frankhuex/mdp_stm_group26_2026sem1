@echo off
setlocal
if defined VSCMD_VER goto compiler_ready
set "testVsRoot="
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
    for /f "usebackq tokens=*" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "testVsRoot=%%I"
)
if not defined testVsRoot set "testVsRoot=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community"
call "%testVsRoot%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 exit /b 1
:compiler_ready
pushd "%~dp0\..\.."
if not exist "tests\host\build" mkdir "tests\host\build"
lib /nologo /machine:x64 /def:tests\host\no_sdk\kernel32.def /out:tests\host\build\host_kernel32.lib
if errorlevel 1 exit /b 1
lib /nologo /machine:x64 /def:tests\host\no_sdk\msvcrt.def /out:tests\host\build\host_msvcrt.lib
if errorlevel 1 exit /b 1
cl /nologo /TC /W3 /DMOTION_SEQUENCE_CASE=1 /DMOTION_SEQUENCE_HEADING_ON=1 /Zl /GS- /Gy /Itests\host\no_sdk /Itests\host /ICore\Inc /Fotests\host\build\ /Fetests\host\build\test_straight_heading.exe tests\host\test_straight_heading.c Core\Src\control\straight_control.c Core\Src\control\motion_heading.c tests\host\no_sdk\runner.c tests\host\unused_turn_hardware.c /link /NODEFAULTLIB /OPT:REF /ENTRY:TestEntry /SUBSYSTEM:CONSOLE tests\host\build\host_kernel32.lib tests\host\build\host_msvcrt.lib
if errorlevel 1 exit /b 1
if /I "%~1"=="--build-only" (
    popd
    exit /b 0
)
tests\host\build\test_straight_heading.exe
set "test_result=%ERRORLEVEL%"
popd
exit /b %test_result%
