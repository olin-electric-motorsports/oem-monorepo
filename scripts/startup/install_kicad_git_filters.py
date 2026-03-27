"""Cross-platform installer for KiCad git filters.

Replaces install_kicad_git_filters.sh. Sets up a git clean filter
that normalizes KiCad power/flag symbol numbering in .kicad_sch files.
"""

import platform
import shutil
import subprocess
import sys
from pathlib import Path


def main():
    # The filter uses sed to normalize power/flag symbol numbers.
    # On Windows, Git for Windows bundles sed, but we fall back to a
    # Python-based filter if sed isn't available.
    if shutil.which("sed"):
        filter_cmd = "sed -E 's/#(PWR|FLG)[0-9]+/#\\1?/'"
    else:
        # Fallback: use Python as the filter command
        this_script = Path(__file__).resolve().as_posix()
        filter_cmd = f'python "{this_script}" --run-filter'

    subprocess.run(
        ["git", "config", "--local", "filter.kicad_sch.clean", filter_cmd],
        check=True,
    )
    subprocess.run(
        ["git", "config", "--local", "filter.kicad_sch.smudge", "cat"],
        check=True,
    )

    # Append to .git/info/attributes if not already present
    repo_root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()

    attributes_path = Path(repo_root) / ".git" / "info" / "attributes"
    attributes_path.parent.mkdir(parents=True, exist_ok=True)

    filter_line = "*.kicad_sch filter=kicad_sch"
    existing = attributes_path.read_text(encoding="utf-8") if attributes_path.exists() else ""
    if filter_line not in existing:
        with open(attributes_path, "a", encoding="utf-8") as f:
            f.write(filter_line + "\n")

    print("KiCad git filters installed.")


def run_filter():
    """Act as a git clean filter: read stdin, normalize, write stdout."""
    import re
    for line in sys.stdin:
        sys.stdout.write(re.sub(r"#(PWR|FLG)[0-9]+", r"#\1?", line))


if __name__ == "__main__":
    if "--run-filter" in sys.argv:
        run_filter()
    else:
        main()
