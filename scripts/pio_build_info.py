"""extra_script для PlatformIO: обновляет main/build_info.h перед сборкой.

PlatformIO читает конфигурацию CMake, но компилирует своим SCons, поэтому
add_custom_target из main/CMakeLists.txt здесь не выполняется. Без этого
крючка версия и дата застывали бы ровно так же, как в штатном дескрипторе
приложения ESP-IDF.

Путь берётся из PROJECT_DIR, а не из __file__: SCons исполняет скрипт
через exec(), и __file__ в его пространстве имён не определён.
"""
import os
import subprocess
import sys

Import("env")  # noqa: F821  — предоставляется SCons внутри PlatformIO

_project = env.subst("$PROJECT_DIR")  # noqa: F821
subprocess.run(
    [sys.executable, os.path.join(_project, "scripts", "gen_build_info.py")],
    check=True,
)
