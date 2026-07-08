"""Format C++ source files with clang-format."""

from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

CPP_FORMAT_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}


def cpp_files(root: pathlib.Path) -> list[pathlib.Path]:
    """Return C++ source files below the repository src directory."""
    src_dir = root / "src"
    if not src_dir.exists():
        return []

    return sorted(path for path in src_dir.rglob("*") if path.is_file() and path.suffix in CPP_FORMAT_EXTENSIONS)


def main() -> int:
    """Run clang-format on C++ files."""
    root = pathlib.Path.cwd()
    files = cpp_files(root)
    if not files:
        sys.stdout.write("No C++ files found.\n")
        return 0

    executable = shutil.which("clang-format")
    if executable is None:
        sys.stderr.write("clang-format not found. Install it with `pixi add clang-format`.\n")
        return 127

    result = subprocess.run([executable, "-i", *map(str, files)], check=False)  # noqa: S603
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
