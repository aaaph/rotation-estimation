"""Lint C++ source files with clang-tidy."""

from __future__ import annotations

import functools
import os
import pathlib
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import TextIO

CPP_LINT_EXTENSIONS = {".cc", ".cpp", ".cxx"}
DEFAULT_CLANG_TIDY_JOBS = 4
MIN_PACKAGE_RELATIVE_PARTS = 2


@dataclass(frozen=True)
class LintJob:
    """A clang-tidy invocation target."""

    path: pathlib.Path
    build_dir: pathlib.Path


@dataclass(frozen=True)
class LintResult:
    """Captured clang-tidy output for one source file."""

    path: pathlib.Path
    returncode: int
    stdout: str
    stderr: str


def cpp_files(root: pathlib.Path) -> list[pathlib.Path]:
    """Return C++ translation units below the repository src directory."""
    src_dir = root / "src"
    if not src_dir.exists():
        return []

    return sorted(path for path in src_dir.rglob("*") if path.is_file() and path.suffix in CPP_LINT_EXTENSIONS)


def package_name_for(path: pathlib.Path, root: pathlib.Path) -> str | None:
    """Return the ROS package name for a source path under src."""
    try:
        relative = path.relative_to(root / "src")
    except ValueError:
        return None

    if len(relative.parts) < MIN_PACKAGE_RELATIVE_PARTS:
        return None
    return relative.parts[0]


def compile_database_dir(path: pathlib.Path, root: pathlib.Path) -> pathlib.Path | None:
    """Return the compile database directory for a C++ source file."""
    package_name = package_name_for(path, root)
    if package_name is None:
        return None

    build_dir = root / "build" / package_name
    if not (build_dir / "compile_commands.json").exists():
        return None
    return build_dir


def write_clang_tidy_output(output: str, stream: TextIO) -> None:
    """Write clang-tidy output without compiler warning summary noise."""
    for line in output.splitlines(keepends=True):
        if line.strip().endswith("warnings generated."):
            continue
        stream.write(line)


def clang_tidy_jobs(file_count: int) -> int:
    """Return the number of parallel clang-tidy jobs to run."""
    if file_count <= 0:
        return 1

    configured_jobs = os.environ.get("CLANG_TIDY_JOBS")
    if configured_jobs is not None:
        try:
            jobs = int(configured_jobs)
        except ValueError as error:
            msg = "CLANG_TIDY_JOBS must be a positive integer."
            raise ValueError(msg) from error

        if jobs <= 0:
            msg = "CLANG_TIDY_JOBS must be a positive integer."
            raise ValueError(msg)
        return min(jobs, file_count)

    return min(DEFAULT_CLANG_TIDY_JOBS, os.cpu_count() or 1, file_count)


def run_lint_job(executable: str, job: LintJob) -> LintResult:
    """Run clang-tidy for one source file."""
    result = subprocess.run(  # noqa: S603
        [executable, "--quiet", "-p", str(job.build_dir), str(job.path)],
        check=False,
        capture_output=True,
        text=True,
    )
    return LintResult(
        path=job.path,
        returncode=result.returncode,
        stdout=result.stdout,
        stderr=result.stderr,
    )


def main() -> int:
    """Run clang-tidy on C++ translation units."""
    root = pathlib.Path.cwd()
    files = cpp_files(root)
    if not files:
        sys.stdout.write("No C++ files found.\n")
        return 0

    executable = shutil.which("clang-tidy")
    if executable is None:
        sys.stderr.write("clang-tidy not found. Install it with `pixi add clang-tools`.\n")
        return 127

    lint_jobs: list[LintJob] = []
    for path in files:
        build_dir = compile_database_dir(path, root)
        if build_dir is None:
            sys.stderr.write(
                f"No compile_commands.json found for {path}. Run `pixi run build` before `pixi run lint-cpp`.\n"
            )
            return 2
        lint_jobs.append(LintJob(path=path, build_dir=build_dir))

    try:
        jobs = clang_tidy_jobs(len(lint_jobs))
    except ValueError as error:
        sys.stderr.write(f"{error}\n")
        return 2

    sys.stdout.write(f"Running clang-tidy with {jobs} parallel job(s).\n")
    runner = functools.partial(run_lint_job, executable)
    exit_code = 0
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        for lint_result in executor.map(runner, lint_jobs):
            sys.stdout.write(f"clang-tidy {lint_result.path}\n")
            write_clang_tidy_output(lint_result.stdout, sys.stdout)
            write_clang_tidy_output(lint_result.stderr, sys.stderr)
            exit_code = max(exit_code, lint_result.returncode)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
