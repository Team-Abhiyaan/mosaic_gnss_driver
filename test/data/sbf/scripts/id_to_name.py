#! /usr/bin/python3
import os
import pickle
from sys import stdin

if __name__ == '__main__':
    names = None
    with open(os.path.join(os.path.dirname(__file__), 'sbf_name_id_desc.pkl'), 'rb') as handle:
        names = pickle.load(handle)

    for line in stdin.readlines():
        sp = list(filter(None, line.strip().split(' ')))
        if len(sp) != 2:
            continue
        nm = None
        try:
            nm = names[sp[1]]
        except KeyError:
            continue

        print(*sp, nm[0], nm[1], sep='\t')
