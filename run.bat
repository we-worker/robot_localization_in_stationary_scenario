rem 1注释：显示回显
@echo on
rem 2注释：激活我们的虚拟环境
call conda activate main
@REM 3注释：切换路径到要执行的脚本文件目录位置,如果在别的盘，需要在cd后面加上" /d ",代表d盘
cd /d D:\Documents\CodeProjects\Robot_2024\robot_localization_in_stationary_scenario
@REM 4注释：执行python文件
start python Main.py
@REM 延时1秒
timeout /t 1
@REM 5注释：切换路径到要另一个要执行的脚本文件目录位置
@REM cd /d D:\Documents\CodeProjects\Robot_2024\robot_localization_in_stationary_scenario
@REM 6注释：执行另一个python文件
start python Location_simulation.py
@REM 7注释：执行文件后，暂停,防止窗口一闪而过，方面调试，后续可以删除这行
pause