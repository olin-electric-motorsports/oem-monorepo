#!/usr/bin/env python3
"""Build real firmware C with a host HAL substitute; never access hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
CASES = [
    'adc_init_error', 'adc_calibration_error', 'adc_retry_calibration',
    'adc_read_config_error', 'adc_read_start_error', 'adc_read_timeout',
    'adc_read_stop_error', 'temporary_calibration_unchanged',
    'apps_latch_unchanged', 'startup_input_still_required',
    'apps_elapsed_time', 'apps_no_elapsed_time', 'apps_clock_wrap',
    'apps_interrupted_fault', 'heartbeat_delayed_task', 'heartbeat_clock_wrap',
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='*', choices=CASES)
    args = parser.parse_args()
    generated = ROOT / 'bazel-bin/vehicle/mkviii/software/lv/VCU'
    if not (generated / 'can_api.c').exists():
        parser.error('First run: bazel build --config=m4 //vehicle/mkviii/software/lv/VCU:vcu_can_api')
    with tempfile.TemporaryDirectory(prefix='oem-firmware-tests-') as tmp:
        binary = Path(tmp) / 'test_vcu'
        command = ['gcc', '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=undefined', '-fno-omit-frame-pointer']
        for path in [HERE / 'include', HERE, ROOT, ROOT / 'vehicle/mkviii/software/lv/VCU/Inc',
                     ROOT / 'vehicle/mkviii/software/lv/VCU', generated]:
            command.append('-I' + str(path))
        command += [str(p) for p in [HERE / 'test_vcu.c', HERE / 'mock_hal.c',
                    ROOT / 'vehicle/mkviii/software/lv/VCU/Src/adc.c',
                    ROOT / 'vehicle/mkviii/software/lv/VCU/Src/gpio.c',
                    ROOT / 'common/adc/adc.c', generated / 'can_api.c', generated / 'can_tools.c']]
        subprocess.run(command + ['-o', str(binary)], check=True)
        failed = 0
        for case in args.cases or CASES:
            result = subprocess.run([str(binary), case], capture_output=True, text=True)
            print(result.stdout, end='')
            if result.returncode:
                print('FAIL', case, result.stderr.strip())
                failed += 1
        return bool(failed)

if __name__ == '__main__':
    raise SystemExit(main())
