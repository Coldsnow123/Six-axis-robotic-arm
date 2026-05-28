@echo off
echo ========================================
echo   Hand IK Controller - 安装依赖
echo ========================================
echo.
echo 正在安装 Python 依赖...
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
echo.
echo 安装完成！运行 python arm_tracker.py 启动
pause
