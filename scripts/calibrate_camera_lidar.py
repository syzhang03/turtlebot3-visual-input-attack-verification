#!/usr/bin/env python3
"""Fit a conservative ambiguity band using clean fixed-scene pairs only."""
import argparse
import csv
import json
import math

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('pairs_csv')
parser.add_argument('--margin', type=float, default=.15)
args = parser.parse_args()
if not math.isfinite(args.margin) or args.margin <= 0:
    parser.error('margin must be positive')
with open(args.pairs_csv) as stream:
    rows = list(csv.DictReader(stream))
visible = [float(r['clearance']) for r in rows
           if r['visual_present']=='True' and math.isfinite(float(r['clearance']))]
clear = [float(r['clearance']) for r in rows
         if r['visual_present']=='False' and math.isfinite(float(r['clearance']))]
if not visible or not clear:
    raise ValueError('Need finite clean observations on both sides of the visual transition')
transition = [max(visible), min(clear)]
print(json.dumps(dict(source=args.pairs_csv, pairs=len(rows), margin=args.margin,
    transition_distances_m=transition,
    verify_near=math.floor((min(transition)-args.margin)*100)/100,
    verify_far=math.ceil((max(transition)+args.margin)*100)/100,
    limits='Only the fixed red obstacle, ROI, geometry and verify threshold used in this clean run.'), indent=2))
