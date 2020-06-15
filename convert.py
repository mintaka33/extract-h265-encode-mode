import numpy as np
import cv2
import sys

def gen_reorder_map():
    a = np.array([[0, 1], [2, 3]])
    a1 = np.column_stack((a, a+4))
    b = np.row_stack((a1, a1+8))
    b1 = np.column_stack((b, b+16))
    c = np.row_stack((b1, b1+32))
    c1 = np.column_stack((c, c+64))
    c2 = np.row_stack((c1, c1+128))
    h, w = c2.shape
    return c2.reshape((h*w))

def reorder(indata, reorder_map):
    sz = indata.shape[0]
    out = np.zeros((sz))
    for i in range(sz):
        out[i] = indata[reorder_map[i]]
    return out

def parse_dump(dumpfile):
    reorder_map = gen_reorder_map()
    with open(dumpfile, "rt") as f:
        row_skipflg, row_zeromv, frame_skipflag, frame_zeromv = [], [], [], []
        cur_row = 0
        for line in f:
            if line.find('mvinfo:POC') != -1:
                if len(row_skipflg) > 0: # last ctu row
                    frame_skipflag.append(row_skipflg)
                    row_skipflg, cur_row = [], 0
                if len(row_zeromv) > 0: # last ctu row
                    frame_zeromv.append(row_zeromv)
                    row_zeromv, cur_row = [], 0
                if len(frame_skipflag) > 0:
                    skipflag_data.append(frame_skipflag)
                    cur_row, row_skipflg, frame_skipflag = 0, [], []
                if len(frame_zeromv) > 0:
                    zeromv_data.append(frame_zeromv)
                    cur_row, row_zeromv, frame_zeromv = 0, [], []
            elif line.find('mvinfo:ctu') != -1:
                _, strx, stry = line.split(':')
                x, y = int(strx.split('=')[1]), int(stry.split('=')[1])
                if cur_row != y:
                    frame_skipflag.append(row_skipflg)
                    frame_zeromv.append(row_zeromv)
                    row_skipflg, row_zeromv = [], []
                    cur_row = y
            elif line.find('mvinfo:skipflag') != -1:
                tag1, tag2, w_str, h_str, data_str = line.split(':')
                w, h = int(w_str), int(h_str)
                data_str = data_str.split(',')
                data_int = np.array(data_str[0:w*h]).astype(np.int8)
                data_reorder = reorder(data_int, reorder_map)
                data = data_reorder.reshape((w, h))
                row_skipflg.append(data)
            elif line.find('mvinfo:zeromvflag') != -1:
                tag1, tag2, w_str, h_str, data_str = line.split(':')
                w, h = int(w_str), int(h_str)
                data_str = data_str.split(',')
                data_int = np.array(data_str[0:w*h]).astype(np.int8)
                data_reorder = reorder(data_int, reorder_map)
                data = data_reorder.reshape((w, h))
                row_zeromv.append(data)
        # last frame
        frame_skipflag.append(row_skipflg)
        skipflag_data.append(frame_skipflag)
        frame_zeromv.append(row_zeromv)
        zeromv_data.append(frame_zeromv)

def generate_maps(data):
    out_maps = []
    for i, frame in enumerate(data):
        frame_out = np.array([])
        for row in frame:
            row_out = np.array([])
            for ctu in row:
                if row_out.size == 0:
                    row_out = ctu
                else:
                    row_out = np.column_stack((row_out, ctu))
            if frame_out.size == 0:
                frame_out = row_out
            else:
                frame_out = np.row_stack((frame_out, row_out))
        out_maps.append(frame_out)
    return np.array(out_maps)

def dump_maps(data, tag):
    for i, frame in enumerate(data):
        filename = 'out_' + tag + '_' + str(i)
        txtfile = filename + '.txt'
        np.savetxt(txtfile, frame, fmt='%d')
        pic = frame * 256
        bmpfile = filename + '.bmp'
        cv2.imwrite(bmpfile, pic)

def execute(dumpfile):
    skipflag_data = []
    zeromv_data = []
    parse_dump(dumpfile)

    skip_maps = generate_maps(skipflag_data)
    zeromv_maps = generate_maps(zeromv_data)
    zeromv_skip_maps = []
    for i in range(len(skip_maps)):
        zm = skip_maps[i] * zeromv_maps[i]
        zeromv_skip_maps.append(zm)

    dump_maps(skip_maps, 'skip')
    dump_maps(zeromv_maps, 'zeromv')
    dump_maps(zeromv_skip_maps, 'zeromv_skip')

if __name__ == "__main__":
    if len(sys.argv) == 2:
        dumpfile = sys.argv[1]
    else:
        print("ERROR: invalid command line. example: python convert.py dump_file.log")
        exit()

    execute(dumpfile)

    print('done')
