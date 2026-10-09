@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 exit /b 1
pushd "%~dp0\..\.."
if not exist "tests\host\build" mkdir "tests\host\build"
cl /nologo /TC /W3 /D_CRT_SECURE_NO_WARNINGS /Itests\host\round_trip_stubs /ICore\Inc /Fotests\host\build\test_straight_round_trip.obj /Fetests\host\build\test_straight_round_trip.exe tests\host\test_straight_round_trip.c
if errorlevel 1 exit /b 1
tests\host\build\test_straight_round_trip.exe
set "test_result=%ERRORLEVEL%"
popd
exit /b %test_result%
