"""Run portable logic and GPIO tests. Example: --compiler path/to/zig cc."""
import argparse
import pathlib
import subprocess
import tempfile

# The repo root and this firmware folder
ROOT = pathlib.Path(__file__).resolve().parents[6]
APP = pathlib.Path(__file__).resolve().parents[1]
# The four logic tests, and the extra flags --sanitize adds
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
        # Compile the sources with these flags and run the result; any failure stops everything
        def run(name, sources, flags):
            executable = pathlib.Path(output) / (name + ".exe")
            subprocess.run(args.compiler + ["-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I" + str(ROOT), "-I" + str(APP)] + extra + flags + [str(APP / s) for s in sources] +
                ["-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
        # Every logic test, for both AIR wirings
        for negative in (0, 1):
            for test in LOGIC_TESTS:
                run(f"{test}_{negative}", ["../air.c", "../air_protocol.c", f"tests/{test}.c"],
                    ["-DAIR_CONTROLLED_NEGATIVE=" + str(negative)])
        # The GPIO test with five pin tables: the real empty one, switched off and
        # on; a test table, switched off and on; and one with a pin used twice
        cases = [
            ("default", [], ["../air_board_config.c"]),
            ("unassigned", ["-DAIR_BOARD_CONFIGURED=1"], ["../air_board_config.c"]),
            ("disabled", ["-DTEST_ASSIGNED_BOARD"], []),
            ("assigned", ["-DTEST_ASSIGNED_BOARD", "-DAIR_BOARD_CONFIGURED=1"], []),
            ("duplicate", ["-DTEST_ASSIGNED_BOARD", "-DAIR_BOARD_CONFIGURED=1", "-DTEST_DUPLICATE_PIN"], []),
        ]
        for name, flags, config in cases:
            run(name, ["../air_board.c", "tests/board_test.c"] + config,
                ["-I" + str(APP / "tests")] + flags)
    print("PASS: four logic tests for both contactor topologies and five GPIO configuration cases")


if __name__ == "__main__":
    main()
