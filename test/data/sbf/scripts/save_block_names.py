#! /usr/bin/python3
import os
import re
import pickle

if __name__ == '__main__':
    names = {}
    p = re.compile("\d{4}")
    with open(os.path.join(os.path.dirname(__file__), 'sbf_name_id_clean'), 'r', encoding="utf-8") as table:
        for line in table:
            line = line.strip()
            matches = list(p.finditer(line))
            if len(matches) != 1:
                continue
            m = matches[0]

            num = line[m.start():m.end()]
            name = line[:m.start()].strip().replace(' ', '')

            desc = line[m.end():].strip()
            if desc.find('*') != -1:
                desc = desc[:desc.find('*') - 1]
            if desc.find('•') != -1:
                desc = desc[:desc.find('•') - 1]
            if desc[-2] == ' ' and desc[-1] in ('R', 'S', 'E'):
                desc = desc[:-2]

            names[num] = (name, desc)

    with open(os.path.join(os.path.dirname(__file__), 'sbf_name_id_desc.pkl'), 'wb') as f:
        pickle.dump(names, f)