"""Cross-platform removal of KiCad git hooks.

Replaces remove_kicad_git_hooks.sh.
"""

import subprocess
from pathlib import Path


def main():
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True, text=True, check=True,
    )
    repo_root = Path(result.stdout.strip())
    hooks_dir = repo_root / ".git" / "hooks"

    for hook_name in ["pre-commit", "post-checkout", "post-merge", "post-commit", "pre-push"]:
        hook_path = hooks_dir / hook_name
        if hook_path.exists():
            backup = hook_path.with_suffix(".old")
            hook_path.rename(backup)
            print(f"Moved {hook_name} -> {hook_name}.old")
        else:
            print(f"Skipped {hook_name} (not found)")

    print("Hooks removed.")


if __name__ == "__main__":
    main()
