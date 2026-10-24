#!/usr/bin/env python3
"""Run VCU C regression checks on the host with a VCU-local HAL substitute."""
import argparse
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
VCU = HERE.parent
ROOT = next(parent for parent in VCU.parents if (parent / 'MODULE.bazel').is_file())
CASES = [
    'adc_init_error', 'adc_calibration_error', 'adc_retry_calibration',
    'adc_read_config_error', 'adc_read_start_error', 'adc_read_timeout',
    'adc_read_stop_error', 'temporary_calibration_unchanged',
    'apps_latch_unchanged', 'startup_input_still_required',
    'apps_elapsed_time', 'apps_no_elapsed_time', 'apps_clock_wrap',
    'apps_interrupted_fault', 'heartbeat_delayed_task', 'heartbeat_clock_wrap',
    'can_first_disable', 'can_three_slot_burst', 'can_deferred_status',
    'can_backpressure', 'can_submit_error', 'can_recovery_disabled',
    'can_pending_uses_safe_state', 'can_command_deadline', 'can_bus_off', 'can_ack_error',
    'can_busy_before_deadline', 'healthy_drive_and_brake',
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='*', choices=CASES)
    args = parser.parse_args()
    generated = ROOT / 'bazel-bin' / VCU.relative_to(ROOT)
    if not (generated / 'can_api.c').exists():
        parser.error('First run: bazel build --config=m4 //vehicle/mkviii/software/lv/VCU:vcu_can_api')
    with tempfile.TemporaryDirectory(prefix='oem-vcu-tests-') as tmp:
        binary = Path(tmp) / 'test_vcu'
        command = ['gcc', '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=undefined', '-fno-omit-frame-pointer']
        for path in [HERE / 'include', HERE, ROOT, VCU / 'Inc',
                     VCU, generated]:
            command.append('-I' + str(path))
        command += [str(p) for p in [HERE / 'test_vcu.c', HERE / 'mock_hal.c',
                    VCU / 'Src/adc.c',
                    VCU / 'Src/gpio.c',
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
