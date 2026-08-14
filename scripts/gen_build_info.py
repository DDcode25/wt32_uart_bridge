#!/usr/bin/env python3
"""Генерирует main/build_info.h — версию из git и отметку времени сборки.

Зачем отдельный генератор, а не штатный дескриптор приложения ESP-IDF:
версия и дата в нём берутся из __DATE__/__TIME__ собственной трансляционной
единицы, которая при инкрементальной сборке не перекомпилируется. На плате
из-за этого оставались версия чужой ветки и дата первой сборки, хотя бинарь
пересобирался — отличить залитое от старого было нельзя.

Скрипт вызывается из двух мест, потому что сборка поддерживается обеими
системами: platformio.ini дёргает его перед каждой сборкой (PlatformIO
читает CMake, но компилирует своим SCons, и цели CMake там не выполняются),
а CMakeLists.txt — на этапе настройки, для сборки через idf.py.

Файл переписывается только при изменении содержимого, иначе каждая сборка
трогала бы mtime и тянула лишнюю перекомпиляцию.
"""
import os
import subprocess
import sys
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "main", "build_info.h")


def git_version():
    try:
        out = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=ROOT, capture_output=True, text=True, timeout=10,
        )
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


def main():
    now = datetime.now(timezone.utc)
    content = (
        "/* Генерируется при сборке (scripts/gen_build_info.py). Не редактировать. */\n"
        "#pragma once\n"
        '#define FIRMWARE_GIT_VERSION "%s"\n'
        '#define FIRMWARE_BUILD_DATE  "%s"\n'
        '#define FIRMWARE_BUILD_TIME  "%s UTC"\n'
    ) % (git_version(), now.strftime("%Y-%m-%d"), now.strftime("%H:%M:%S"))

    old = None
    if os.path.exists(OUT):
        with open(OUT, "r", encoding="utf-8") as f:
            old = f.read()
    if old != content:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w", encoding="utf-8") as f:
            f.write(content)
    return 0


if __name__ == "__main__":
    sys.exit(main())
