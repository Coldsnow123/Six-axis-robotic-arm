# -*- mode: python ; coding: utf-8 -*-

a = Analysis(
    ['arm_tracker.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('models/hand_landmarker.task', 'models'),
        ('models/pose_landmarker_full.task', 'models'),
    ],
    hiddenimports=['mediapipe.tasks.python.vision', 'serial.tools.list_ports'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='HandIK_Controller',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,
)
