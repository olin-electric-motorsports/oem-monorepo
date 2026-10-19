"""Run portable logic and GPIO tests. Example: --compiler path/to/zig cc."""
import argparse
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[5]
APP = pathlib.Path(__file__).resolve().parents[1]
LOGIC_TESTS = ["air_test", "air_paths_test", "air_known_issues_test", "air_invariant_test"]
SANITIZERS = ["-g", "-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--compiler", nargs="+", default=["cc"])
    parser.add_argument("--sanitize", action="store_true",
                        help="add AddressSanitizer and UBSan (gcc or clang on Linux/macOS)")
    args = parser.parse_args()
    extra = SANITIZERS if args.sanitize else []
    with tempfile.TemporaryDirectory(prefix="air-tests-") as output:
        def run(name, sources, flags):
            executable = pathlib.Path(output) / (name + ".exe")
            subprocess.run(args.compiler + ["-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(ROOT)] + extra + flags + [str(APP / s) for s in sources] +
                ["-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
        for negative in (0, 1):
            for test in LOGIC_TESTS:
                run(f"{test}_{negative}", ["air.c", "air_protocol.c", f"tests/{test}.c"],
                    ["-DAIR_CONTROLLED_NEGATIVE=" + str(negative)])
        cases = [
            ("default", [], ["air_board_config.c"]),
            ("unassigned", ["-DAIR_BOARD_CONFIGURED=1"], ["air_board_config.c"]),
            ("disabled", ["-DTEST_ASSIGNED_BOARD"], []),
            ("assigned", ["-DTEST_ASSIGNED_BOARD", "-DAIR_BOARD_CONFIGURED=1"], []),
            ("duplicate", ["-DTEST_ASSIGNED_BOARD", "-DAIR_BOARD_CONFIGURED=1", "-DTEST_DUPLICATE_PIN"], []),
        ]
        for name, flags, config in cases:
            run(name, ["air_board.c", "tests/board_test.c"] + config,
                ["-I" + str(APP / "tests")] + flags)
    print("PASS: four logic tests for both contactor topologies and five GPIO configuration cases")


if __name__ == "__main__":
    main()
