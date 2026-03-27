"""Cross-platform removal of KiCad git filters.

Replaces remove_kicad_git_filters.sh.
"""

import subprocess


def main():
    subprocess.run(
        ["git", "config", "--local", "filter.kicad_sch.clean", ""],
        check=True,
    )
    subprocess.run(
        ["git", "config", "--local", "filter.kicad_sch.smudge", ""],
        check=True,
    )
    print("KiCad git filters removed.")


if __name__ == "__main__":
    main()
