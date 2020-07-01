import numpy as np
import cv2
import sys

def get_size(dumpfile):
    w, h = 0, 0
    tag = 'Image Format : '
    with open(dumpfile, "rt") as f:
        for line in f:
            if tag in line:
                s = line.split(tag)[1].split(' (')[0]
                s1, s2 = s.split('x')
                w, h = int(s1), int(s2)
                break
    return (w, h)

def parse_dump(dumpfile, w, h):
    w, h = int(w/16), int(h/16)
    skipflag, zeromv = [], []
    with open(dumpfile, "rt") as f:
        for line in f:
            if 'skipflag: ' in line:
                _, str1 = line.split('skipflag: ')
                skip_str = str1.split(', ')
                skip_int = np.array(skip_str[0:w*h]).astype(np.int8)
                skipflag.append(skip_int.reshape(h, w))
            elif 'mvinfo: ' in line:
                _, str2 = line.split('mvinfo: ')
                mvinfo_str = str2.split(', ')
                mvinfo_int = np.array(mvinfo_str[0:w*h*16]).astype(np.int8)
                zeromv.append(mvinfo_int.reshape(h*4, w*4))
    return skipflag, zeromv

def dump_to_file(data, tag):
    for i, frame in enumerate(data):
        filename = 'out_' + tag + '_' + str(i+1)
        txtfile = filename + '.txt'
        np.savetxt(txtfile, frame, fmt='%d')
        if False:
            pic = frame * 256
            bmpfile = filename + '.bmp'
            cv2.imwrite(bmpfile, pic)

def calc_skip_zeromv(skipflag, zeromv):
    skip_zeromv = []
    for s, z in zip(skipflag, zeromv):
        sz = np.zeros(z.shape)
        h, w = z.shape
        for y in range(h):
            for x in range(w):
                if s[int(y/4)][int(x/4)] == 1 and z[y][x] == 1:
                    sz[y][x] = 1
                else:
                    sz[y][x] = 0
        skip_zeromv.append(sz)
    return skip_zeromv

def execute(dumpfile):
    w, h = get_size(dumpfile)
    skipflag, zeromv = parse_dump(dumpfile, w, h)
    dump_to_file(skipflag, 'skip')
    dump_to_file(zeromv, 'mvinfo')
    skip_zeromv = calc_skip_zeromv(skipflag, zeromv)
    dump_to_file(skip_zeromv, 'skip_zeromv')
    return

if __name__ == "__main__":
    if len(sys.argv) == 2:
        dumpfile = sys.argv[1]
    else:
        print("ERROR: invalid command line. example: python convert.py dump_file.log")
        dumpfile = "D:\\Code\\mintaka33\\extract-h265-encode-mode\\jm19.0\\JM\\bin\\dump.log"
        #exit()

    execute(dumpfile)

    print('done')