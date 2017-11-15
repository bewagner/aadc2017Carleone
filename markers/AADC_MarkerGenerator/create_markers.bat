

for /l %%x in (0, 1, 19) do (
   aadc_markerGenerator.exe -d=5 --id=%%x --ms=500 aruco_marker_%%x.png

)