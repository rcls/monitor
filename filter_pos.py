#!/usr/bin/python3

import csv
import sys

_, IN, BOM, OUT = sys.argv

refs = []

for L in csv.reader(open(BOM, newline='')):
    if L[-1].startswith('C'):
        refs.extend(L[1].split(','))

refset = set(refs)

lines = ['Designator,Mid X,Mid Y,Layer,Rotation']

prefix_rotate = [
    ('MSOP-', -90),
    ('QFN-', -90),
    ('SOIC-', -90),
    ('SOT-23', 180),
    ('WSON', -90),
    ('Texas_S-PWSON', -90)]

for L in csv.reader(open(IN, newline='')):
    #print(L)
    ref, val, package, posx, posy, rot, side = L
    if not ref in refset:
        continue
    rot = float(rot)
    for prefix, rotate in prefix_rotate:
        if package.startswith(prefix):
            rot += rotate
        if rot > 180:
            rot -= 360
        if rot <= -180:
            rot += 360
    lines.append(f'{ref},{posx}mm,{posy}mm,{side},{rot}')

outf = open(OUT, 'w')
for L in lines:
    print(L, file=outf)
