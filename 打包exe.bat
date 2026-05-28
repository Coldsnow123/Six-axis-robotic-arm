@echo off
chcp 65001 >nul
echo ========================================
echo   打包 HandIK_Controller.exe
echo ========================================
echo.

echo [1/3] 安装 PyInstaller...
pip install pyinstaller -i https://pypi.tuna.tsinghua.edu.cn/simple

echo.
echo [2/3] 开始打包 (约 3~10 分钟, 取决于电脑性能)...
pyinstaller --clean --noconfirm ^
    --add-data "models\hand_landmarker.task;models" ^
    --add-data "models\pose_landmarker_full.task;models" ^
    --hidden-import mediapipe.tasks.python.vision ^
    --hidden-import serial.tools.list_ports ^
    --name HandIK_Controller ^
    arm_tracker.py

echo.
echo [3/3] 复制到当前目录...
if exist "dist\HandIK_Controller\HandIK_Controller.exe" (
    copy /Y "dist\HandIK_Controller\HandIK_Controller.exe" "HandIK_Controller.exe"
    echo.
    echo ========================================
    echo   打包成功!
    echo   双击 HandIK_Controller.exe 即可启动
    echo ========================================
) else (
    echo 打包可能失败, 请检查上方错误信息
)

pause
