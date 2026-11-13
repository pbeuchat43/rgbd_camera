"""Replay complete private numeric traces; emit a path-free summary."""
# Copyright (c) 2026 Paul N. Beuchat. MIT License.
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--trace', action='append', required=True, help='LABEL=PRIVATE_TRACE_DIRECTORY')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()

    def run(*arguments):
        return json.loads(subprocess.check_output([str(args.executable), *map(str, arguments)], text=True))

    report = dict(schema=1, prototype=True, production_guard_changed=False, observation_source='provider_dequeue',
                  stream_order=['accel', 'gyro', 'rgb_event', 'depth_event', 'left_event', 'right_event'],
                  parameters=dict(startup_ns=1_000_000_000, receipt_margin_ns=5_000_000,
                                  bin_ns=250_000_000, bins=8, history_segments=64, max_slew_ppm=5000),
                  checks=run('--self-test'), traces={})
    for item in args.trace:
        label, directory = item.split('=', 1)
        trace = Path(directory)
        complete = json.loads((trace/'complete.json').read_text())
        assert complete == dict(schema='oak-clock-raw-v1', byte_order='little', imu_struct='7q7Q', image_struct='5q')
        identities = {}
        for name, size in [('arrival', 112), ('dequeue', 112), ('images', 40)]:
            metadata = json.loads((trace/(name+'-window.json')).read_text())
            binary = trace/(name+'.bin')
            assert metadata['overflow'] == 0 and metadata['row_bytes'] == size
            assert binary.stat().st_size == metadata['rows'] * size
            with binary.open('rb') as stream:
                digest = hashlib.file_digest(stream, 'sha256').hexdigest()
            identities[name] = dict(rows=metadata['rows'], sha256=digest)
        modes = [run(trace, clock, policy) for clock, policy in [('sdk', 'stop'), ('sdk', 'drop'), ('local', 'stop')]]
        report['traces'][label] = dict(inputs=identities, modes=modes)
    report['benchmark'] = run('--benchmark')
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    for label, trace in report['traces'].items():
        for mode in trace['modes']:
            print(json.dumps(dict(trace=label, clock=mode['clock'], policy=mode['fault_policy'],
                                  stopped=mode['stopped'], faults=mode['clock_faults'], combined=mode['combined'],
                                  combined_rejected=mode['combined_rejected'], accel=mode['streams'][0])))
    print(json.dumps(report['benchmark']))


if __name__ == '__main__':
    main()
