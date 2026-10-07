"""Compare the real C codec with cantools and the repository-generated DBC.

Requires the repository's Python requirements. --compiler accepts e.g. zig cc.
Also regresses DBC generation with the IVT DBC, which has no BU_ node section.
"""
import argparse
import pathlib
import random
import subprocess
import sys
import tempfile

import cantools

ROOT = pathlib.Path(__file__).resolve().parents[5]
APP = pathlib.Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--compiler", nargs="+", default=["cc"])
    args = parser.parse_args()
    rng = random.Random(441)
    with tempfile.TemporaryDirectory(prefix="air-protocol-") as temp:
        temp = pathlib.Path(temp)
        dbc = temp / "air.dbc"
        subprocess.run([sys.executable, str(ROOT / "common/can_api/dbc_generator.py"),
                        "-o", str(dbc), str(APP / "air.yml"), str(APP / "legacy_bms.yml"),
                        str(ROOT / "vehicle/common/gmeter/gmeter.dbc")], check=True)
        db = cantools.database.load_file(dbc)
        code = ['#include "vehicle/mkviii/software/air_control/air_protocol.h"',
                '#include <stdio.h>', '#include <string.h>',
                '#define CHECK(e) do { if (!(e)) { printf("line %d\\n", __LINE__); return 1; } } while (0)',
                'int main(void) { air_measurements_t d = {0};']
        for _ in range(512):
            data = bytes(rng.randrange(256) for _ in range(7))
            decoded = db.decode_message("bms_core", data, decode_choices=False, scaling=False)
            code += ['{ const uint8_t p[7] = {' + ','.join(map(str, data)) + '};',
                     'CHECK(air_decode(&d, AIR_CAN_BMS_ID, p, 7, 42));',
                     f'CHECK(d.bms_state == {decoded["bms_state"]});',
                     f'CHECK(d.bms_fault == {decoded["bms_fault_code"]});',
                     f'CHECK(d.pack_mv == {(decoded["pack_voltage"] * 256 + 5) // 10}); }}']
            data = bytes([1] + [rng.randrange(256) for _ in range(5)])
            decoded = db.decode_message("IVT_Msg_Result_U1", data, decode_choices=False, scaling=False)
            error = any(decoded[key] for key in decoded if key.endswith(("Error", "OCS")))
            code += ['{ const uint8_t p[6] = {' + ','.join(map(str, data)) + '};',
                     'CHECK(air_decode(&d, AIR_CAN_IVT_ID, p, 6, 42));',
                     f'CHECK(d.tractive_mv == {decoded["IVT_Result_U1"]});',
                     f'CHECK(d.ivt_error == {int(error)}); }}']
            fault, state = rng.randrange(17), rng.randrange(7)
            signals = dict(air_fault=fault, air_state=state)
            flags = ["air_p_status", "air_n_status", "ss_tsms", "ss_imd", "ss_mpc",
                     "ss_tsmp", "ss_hvd", "ss_bms", "ss_emeter", "imd_status"]
            signals.update({s: rng.randrange(2) for s in flags})
            expected = db.encode_message("air_control_critical", signals, scaling=False)
            shutdown = sum(signals[s] << i for i, s in enumerate(flags[2:9]))
            code += [f'{{ air_t a = {{.fault = {fault}, .state = {state}}};',
                     f'air_inputs_t in = {{.shutdown_closed = {shutdown}, .imd_ok = {signals["imd_status"]},'
                     f'.air_p_closed = {signals["air_p_status"]}, .air_n_closed = {signals["air_n_status"]}}};',
                     'uint8_t p[4]; const uint8_t expected[4] = {' + ','.join(map(str, expected)) + '};',
                     'air_encode_status(p, &a, &in); CHECK(memcmp(p, expected, 4) == 0); }']
        code += ['puts("PASS: 1536 C codec comparisons with cantools/DBC"); return 0; }']
        fixture = temp / "vectors.c"
        fixture.write_text('\n'.join(code))
        exe = temp / "vectors.exe"
        subprocess.run(args.compiler + ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(ROOT),
                        str(APP / "air_protocol.c"), str(fixture), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
