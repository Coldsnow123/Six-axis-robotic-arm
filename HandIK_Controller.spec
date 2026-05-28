# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['arm_tracker.py'],
    pathex=[],
    binaries=[],
    datas=[('models\\hand_landmarker.task', 'models'), ('models\\pose_landmarker_full.task', 'models')],
    hiddenimports=['mediapipe.tasks.python.vision', 'serial.tools.list_ports'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='HandIK_Controller',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='HandIK_Controller',
)
