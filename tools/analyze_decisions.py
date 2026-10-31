#!/usr/bin/env python3
"""Inspect one captured MPPI decision without rerunning a closed-loop scenario."""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--time',type=float)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with args.csv.open() as f:
        rows=[r for r in csv.DictReader(f) if None not in r and all(v is not None for v in r.values())]
    if not rows:parser.error('no complete decision rows')
    stamp=max(float(r['t']) for r in rows) if args.time is None else min(
        {float(r['t']) for r in rows},key=lambda t:abs(t-args.time))
    frame=[r for r in rows if float(r['t'])==stamp]
    keys=('candidate','selected','valid','rejection','raw_peak','conditioned_peak',
          'predicted_progress','xy_motion','cost','path','heading','velocity','alignment',
          'effort','progress_cost','goal')
    table=[]
    for r in frame:
        table.append({k:(None if not math.isfinite(float(r[k])) else float(r[k])) for k in keys})
    context={k:float(frame[0][k]) for k in ('t','reference_speed','reference_limit','noise_scale',
                'reorientation','alignment_error','terminal_hold','seed_peak','seed_conditioned_peak')}
    data=dict(context=context,candidates=table)
    text=json.dumps(data,indent=2,allow_nan=False)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')


if __name__=='__main__':main()
