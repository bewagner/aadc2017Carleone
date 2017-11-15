@echo off
set batdir=%~dp0
echo Dir is %batdir%
pushd %batdir%

set output_dir=gen-ExtIf
rem delete old folder
rem rmdir /S /Q %output_dir%
rem gen new folder
rem mkdir %output_dir%

thrift.exe --gen py --out %output_dir% ExtIf.thrift

echo.
echo Finished! your thrift files have been generated in "%batdir%%output_dir%"
popd
pause
