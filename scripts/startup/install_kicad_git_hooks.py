"""Cross-platform installer for KiCad git hooks and Git LFS.

Replaces install_kicad_git_hooks.sh with a Python script that works
on Linux, macOS, and Windows.
"""

import platform
import shutil
import stat
import subprocess
import sys
from pathlib import Path


def get_repo_root():
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True, text=True, check=True,
    )
    return Path(result.stdout.strip())


def install_git_lfs():
    """Install git-lfs if not already present."""
    if shutil.which("git-lfs"):
        print("Git LFS already installed.")
        return

    system = platform.system()

    if system == "Linux":
        if shutil.which("apt-get"):
            subprocess.run(["sudo", "apt-get", "update"], check=True)
            subprocess.run(["sudo", "apt-get", "install", "-y", "git-lfs"], check=True)
        elif shutil.which("dnf"):
            subprocess.run(["sudo", "dnf", "install", "-y", "git-lfs"], check=True)
        elif shutil.which("pacman"):
            subprocess.run(["sudo", "pacman", "-S", "--noconfirm", "git-lfs"], check=True)
        else:
            print("Error: No supported package manager found. Install git-lfs manually.")
            sys.exit(1)

    elif system == "Darwin":
        if shutil.which("brew"):
            subprocess.run(["brew", "install", "git-lfs"], check=True)
        else:
            print("Error: Homebrew not found. Install git-lfs manually.")
            sys.exit(1)

    elif system == "Windows":
        if shutil.which("winget"):
            subprocess.run(["winget", "install", "--id", "GitHub.GitLFS", "-e"], check=True)
        elif shutil.which("choco"):
            subprocess.run(["choco", "install", "git-lfs", "-y"], check=True)
        else:
            print("Error: No supported package manager found (winget or choco).")
            print("Install git-lfs manually: https://git-lfs.com")
            sys.exit(1)

    else:
        print(f"Error: Unsupported OS: {system}")
        sys.exit(1)

    subprocess.run(["git", "lfs", "install"], check=True)
    print("Git LFS installed.")


def write_hook(hooks_dir, hook_name, content):
    """Write a git hook file and make it executable."""
    hook_path = hooks_dir / hook_name
    hook_path.write_text(content, encoding="utf-8")

    # Make executable on Unix (no-op semantically on Windows, but Git for
    # Windows' bash respects the file content regardless)
    if platform.system() != "Windows":
        hook_path.chmod(hook_path.stat().st_mode | stat.S_IEXEC)


def main():
    repo_root = get_repo_root()
    hooks_dir = repo_root / ".git" / "hooks"
    hooks_dir.mkdir(parents=True, exist_ok=True)

    print("Installing Git Hooks (KiCad + Git LFS)...")

    install_git_lfs()

    # Use forward slashes in hook scripts — Git for Windows' bash handles them
    root = repo_root.as_posix()

    # post-checkout: rebuild KiCad symbol library + LFS fetch
    write_hook(hooks_dir, "post-checkout", f"""#!/bin/bash
# Implode symbols
bazel run //tools/symbols:convert -- symbols2library --source "{root}/parts/schematic/oem" --out "{root}/parts/schematic/oem.kicad_sym"

# Fetch large files
if command -v git-lfs >/dev/null 2>&1; then
    git lfs post-checkout "$@"
fi
""")

    # post-merge: same as post-checkout
    write_hook(hooks_dir, "post-merge", f"""#!/bin/bash
# Implode symbols
bazel run //tools/symbols:convert -- symbols2library --source "{root}/parts/schematic/oem" --out "{root}/parts/schematic/oem.kicad_sym"

# Fetch large files
if command -v git-lfs >/dev/null 2>&1; then
    git lfs post-merge "$@"
fi
""")

    # pre-commit: explode library into atomic symbol files
    write_hook(hooks_dir, "pre-commit", f"""#!/bin/bash
# Explode symbols
bazel run //tools/symbols:convert -- library2symbols --source "{root}/parts/schematic/oem.kicad_sym" --out "{root}/parts/schematic/oem"

# Add the exploded files
git add "{root}/parts/schematic/oem"
""")

    # post-commit: rebuild the single library file for local use
    write_hook(hooks_dir, "post-commit", f"""#!/bin/bash
# Implode symbols
bazel run //tools/symbols:convert -- symbols2library --source "{root}/parts/schematic/oem" --out "{root}/parts/schematic/oem.kicad_sym"
""")

    # pre-push: LFS upload
    write_hook(hooks_dir, "pre-push", """#!/bin/bash
if command -v git-lfs >/dev/null 2>&1; then
    git lfs pre-push "$@"
fi
""")

    print("Hooks installed.")


if __name__ == "__main__":
    main()
