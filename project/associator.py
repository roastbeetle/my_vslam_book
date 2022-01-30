#!/usr/bin/env python
f_rgb = open("./rgb2.txt",'r')
f_dep = open("./depth2.txt",'r')
f_ass = open("./associate.txt",'w')

while True:
    rgb_line = f_rgb.readline()
    if not rgb_line:
        break
    dep_line = f_dep.readline()
    f_ass.write(rgb_line[0:-1] + " " + dep_line)
