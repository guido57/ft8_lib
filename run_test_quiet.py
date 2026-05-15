#!/usr/bin/env python3
import sys, os, subprocess, re

def parse(line):
    fields = line.strip().split()
    if len(fields) < 8:
        return None
    freq = fields[3]
    dest = fields[5] if len(fields) > 5 else ''
    source = fields[6] if len(fields) > 6 else ''
    report = fields[7] if len(fields) > 7 else ''
    if dest and dest[0] == '<' and dest[-1] == '>':
        dest = '<...>'
    if source and source[0] == '<' and source[-1] == '>':
        source = '<...>'
    return ' '.join([dest, source, report])

wav_dir = sys.argv[1] if len(sys.argv) > 1 else 'tests/20m_busy'
wav_files = [os.path.join(wav_dir, f) for f in os.listdir(wav_dir)]
wav_files = [f for f in wav_files if os.path.isfile(f) and os.path.splitext(f)[1] == '.wav']
txt_files = [os.path.splitext(f)[0] + '.txt' for f in wav_files]

is_ft4 = False
if len(sys.argv) > 2 and sys.argv[2] == '-ft4':
    is_ft4 = True

passed = 0
failed = 0

for wav_file, txt_file in zip(sorted(wav_files), sorted(txt_files)):
    if not os.path.isfile(txt_file): continue
    print(wav_file, end=' ')
    cmd_args = ['./decode_ft8', wav_file]
    if is_ft4:
        cmd_args.append('-ft4')
    env = os.environ.copy()
    env['DLOG_LEVEL'] = 'LOG_ERROR'
    result = subprocess.run(cmd_args, stdout=subprocess.PIPE, env=env)
    result = result.stdout.decode('utf-8').split('\n')
    result = [parse(x) for x in result if x and parse(x) is not None]
    result = set(result)
    
    expected = open(txt_file).read().split('\n')
    expected = [parse(x) for x in expected if x and parse(x) is not None]
    expected = set(expected)
    
    extra_decodes = result - expected
    missed_decodes = expected - result
    
    if len(extra_decodes) == 0 and len(missed_decodes) == 0:
        print(f"✓ ({len(result)} messages)")
        passed += 1
    else:
        print(f"✗ ({len(result)}/{len(expected)} messages)")
        if len(extra_decodes) > 0:
            print(f'  Extra: {len(extra_decodes)}')
        if len(missed_decodes) > 0:
            print(f'  Missed: {len(missed_decodes)}')
        failed += 1

print(f"\nResults: {passed} passed, {failed} failed")
