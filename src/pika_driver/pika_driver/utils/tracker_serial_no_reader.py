#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import ctypes
import ctypes.util
from pathlib import Path

import pysurvive


def find_libsurvive() -> str:
    env_path = os.environ.get("LIBSURVIVE_PATH")
    if env_path and os.path.exists(env_path):
        return env_path

    found = ctypes.util.find_library("survive")
    if found:
        return found

    current_dir = Path(__file__).resolve().parent
    local_libsurvive_dir = current_dir.parent / "libsurvive"

    candidates = []
    if local_libsurvive_dir.exists():
        patterns = [
            "**/libsurvive.so",
            "**/libsurvive.so.0",
            "**/libsurvive.dylib",
        ]
        for pattern in patterns:
            candidates.extend(local_libsurvive_dir.glob(pattern))

    candidates = [str(p) for p in candidates if p.exists()]
    if candidates:
        candidates.sort(key=len)
        return candidates[0]

    raise RuntimeError("libsurvive 공유 라이브러리를 찾지 못했습니다.")


def load_libsurvive():
    lib_path = find_libsurvive()
    print(f"[INFO] libsurvive 로드 경로: {lib_path}")

    lib = ctypes.CDLL(lib_path)
    lib.survive_simple_serial_number.argtypes = [ctypes.c_void_p]
    lib.survive_simple_serial_number.restype = ctypes.c_char_p
    return lib


def safe_name(dev) -> str:
    try:
        return str(dev.Name(), "utf-8")
    except Exception:
        return "<unknown>"


def ptr_to_void_p(ptr_obj) -> ctypes.c_void_p:
    if isinstance(ptr_obj, int):
        return ctypes.c_void_p(ptr_obj)

    try:
        return ctypes.cast(ptr_obj, ctypes.c_void_p)
    except Exception:
        pass

    try:
        return ctypes.c_void_p(int(ptr_obj))
    except Exception as e:
        raise TypeError(
            f"dev.ptr를 c_void_p로 변환할 수 없습니다: {ptr_obj} ({type(ptr_obj)})"
        ) from e


def read_serial(lib, dev) -> str:
    try:
        ptr = ptr_to_void_p(dev.ptr)
        raw = lib.survive_simple_serial_number(ptr)
        if raw is None:
            return "<null>"
        return raw.decode("utf-8")
    except Exception as e:
        return f"<read-failed: {type(e).__name__}: {e}>"


def update_seen_from_objects(ctx, lib, seen):
    try:
        devices = list(ctx.Objects())
    except Exception as e:
        print(f"[WARN] Objects() 실패: {e}")
        return

    for dev in devices:
        name = safe_name(dev)
        serial = read_serial(lib, dev)
        seen[name] = serial


def update_seen_from_updates(ctx, lib, seen, timeout_sec=0.1):
    end_t = time.time() + timeout_sec
    while time.time() < end_t and ctx.Running():
        updated = ctx.NextUpdated()
        if not updated:
            continue

        name = safe_name(updated)
        serial = read_serial(lib, updated)
        seen[name] = serial


def main():
    print(f"[INFO] Python: {sys.executable}")
    lib = load_libsurvive()

    ctx = pysurvive.SimpleContext([])
    if not ctx:
        raise RuntimeError("pysurvive.SimpleContext 생성 실패")

    print("[INFO] 장치 검색 중...")

    seen = {}
    deadline = time.time() + 15.0

    while time.time() < deadline and ctx.Running():
        update_seen_from_objects(ctx, lib, seen)
        update_seen_from_updates(ctx, lib, seen, timeout_sec=0.2)
        time.sleep(0.2)

    print("\n=== DEVICE SERIAL LIST ===")
    for name in sorted(seen.keys()):
        print(f"name={name:>6s} | serial={seen[name]}")
    print("==========================\n")


if __name__ == "__main__":
    main()