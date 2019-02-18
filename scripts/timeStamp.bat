@echo off
for /f "delims=" %%a in ('wmic OS Get localdatetime  ^| find "."') do set dt=%%a
set YYYY=%dt:~0,4%
set MM=%dt:~4,2%
set DD=%dt:~6,2%
set HH=%dt:~8,2%
set Min=%dt:~10,2%
set Sec=%dt:~12,2%

set stamp=%YYYY%-%MM%-%DD%_%HH%-%Min%-%Sec%

mk "C:\Users\ryoma\Desktop\FEHRobot\Archive\%stamp%"

xcopy "C:\Users\ryoma\Desktop\FEHRobot\FEHRobot" "C:\Users\ryoma\Desktop\FEHRobot\Archive\%stamp%"