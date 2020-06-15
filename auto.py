import os
import sys
import convert

if __name__ == "__main__":
    if len(sys.argv) == 2:
        videofile = sys.argv[1]
    else:
        print("ERROR: invalid command line. example: python auto.py inputvideo.264")
        exit()

    dump_log = 'dump.log'

    dec_cmd = 'ffmpeg -i ' + videofile + ' out_%04d.bmp -y'
    os.system(dec_cmd)

    dump_cmd = 'TAppDecoder.exe -b ' + videofile + ' >' + dump_log
    os.system(dump_cmd)

    convert.execute(dump_log)

    print('done')