#!/usr/bin/env python3
"""
pio_lsp.py

Usage examples:
  # generate compile_commands.json for default env and put at project root
  python pio_lsp.py

  # specify PlatformIO env, include toolchain include paths
  python pio_lsp.py --env d1_mini --include-toolchain

  # put final compile_commands at custom path
  python pio_lsp.py --out /path/to/compile_commands.json
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from glob import glob
from pathlib import Path

def find_project_root(start: Path) -> Path:
    for p in [start] + list(start.parents):
        if (p / "platformio.ini").is_file():
            return p
    raise FileNotFoundError("platformio.ini not found in this dir or any parent")

def find_pio_executable():
    for name in ("pio", "platformio"):
        p = shutil.which(name)
        if p:
            return p
    return None

def run_compiledb(pio_cmd, project_root: Path, env: str | None, include_toolchain: bool, extra_env: dict):
    cmd = [pio_cmd, "run", "-t", "compiledb"]
    if env:
        cmd += ["-e", env]
    env_copy = os.environ.copy()
    if include_toolchain:
        env_copy["COMPILATIONDB_INCLUDE_TOOLCHAIN"] = "1"
    # allow overriding where the compile db is written (optional)
    if extra_env.get("COMPILATIONDB_PATH"):
        env_copy["COMPILATIONDB_PATH"] = extra_env["COMPILATIONDB_PATH"]
    # run in project root
    print("Running:", " ".join(cmd), "in", str(project_root))
    r = subprocess.run(cmd, cwd=str(project_root), env=env_copy, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(r.stdout)
    if r.returncode != 0:
        raise RuntimeError(f"PlatformIO compiledb command failed (exit {r.returncode})")

def locate_compdbs(project_root: Path):
    """Return list of existing compile_commands.json paths to consider."""
    paths = []
    # common: project root
    p_root = project_root / "compile_commands.json"
    if p_root.exists():
        paths.append(p_root)
    # common alternative: .pio/build/<env>/compile_commands.json
    pattern = project_root / ".pio" / "build" / "*" / "compile_commands.json"
    for p in glob(str(pattern)):
        paths.append(Path(p))
    # sometimes nested in .pio or other build dirs
    for p in project_root.rglob("compile_commands.json"):
        # avoid duplicates
        if p not in paths:
            paths.append(p)
    return paths

def merge_compile_commands(paths, out_path: Path):
    seen = set()
    merged = []
    for p in paths:
        try:
            arr = json.loads(p.read_text())
            if not isinstance(arr, list):
                continue
            for entry in arr:
                # de-dup by (file, command, directory) triple
                key = (entry.get("file"), entry.get("command"), entry.get("directory"))
                if key in seen:
                    continue
                seen.add(key)
                merged.append(entry)
        except Exception as e:
            print(f"Warning: failed to read {p}: {e}", file=sys.stderr)
    out_path.write_text(json.dumps(merged, indent=2))
    print(f"Wrote {len(merged)} entries to {out_path}")

def main():
    ap = argparse.ArgumentParser(description="Generate/merge compile_commands.json for PlatformIO projects")
    ap.add_argument("--env", "-e", help="PlatformIO environment name (passed to `pio run -e`) ")
    ap.add_argument("--include-toolchain", action="store_true", help="Set COMPILATIONDB_INCLUDE_TOOLCHAIN=1 when running compiledb")
    ap.add_argument("--out", "-o", default="compile_commands.json", help="Output path for merged compile_commands.json (default: project-root/compile_commands.json)")
    ap.add_argument("--cwd", default=".", help="Run from this directory (default: current dir)")
    args = ap.parse_args()

    start = Path(args.cwd).resolve()
    try:
        root = find_project_root(start)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        sys.exit(1)

    pio = find_pio_executable()
    if pio is None:
        print("Could not find 'pio' or 'platformio' executable in PATH.", file=sys.stderr)
        sys.exit(1)

    # run compiledb
    try:
        run_compiledb(pio, root, args.env, args.include_toolchain, {})
    except Exception as e:
        print("Error running compiledb:", e, file=sys.stderr)
        sys.exit(2)

    # locate generated databases
    candidates = locate_compdbs(root)
    if not candidates:
        print("No compile_commands.json found after compiledb. Looked in project root and .pio/build/*.", file=sys.stderr)
        sys.exit(3)

    out_path = Path(args.out)
    if not out_path.is_absolute():
        out_path = (root / out_path).resolve()

    # if multiple, merge; if single, copy
    if len(candidates) == 1:
        shutil.copy2(candidates[0], out_path)
        print(f"Copied {candidates[0]} -> {out_path}")
    else:
        print("Multiple compile_commands.json found; merging them.")
        merge_compile_commands(candidates, out_path)

    print("\nDone. In Neovim: restart your LSP (e.g. :LspRestart) or reload the buffer to pick up the new compile_commands.json.")

if __name__ == "__main__":
    main()
