# extract-h265-encode-mode
Extract video encoding info (like motion vector, skip mode, CU/PU/TU partitions) from H.265/HEVC stream

## Build HM

```bash
cd build
cmake ..\HM-master
```

## generate encode mode dump log
```bash
TAppDecoder.exe -b test.265 >dump.log
```

## convert dump to map
```bash
python convert.py dump.log
```

## result

### skip mode map (each pixel/flag represents one 4x4 block)
![skip](https://github.com/mintaka33/extract-h265-encode-mode/blob/master/result/skip_1.jpg)

### zero mv map (each pixel/flag represents one 4x4 block)
![zeromv](https://github.com/mintaka33/extract-h265-encode-mode/blob/master/result/zeromv_1.jpg)

### skip + zeromv map (each pixel/flag represents one 4x4 block)
![skip+zeromv](https://github.com/mintaka33/extract-h265-encode-mode/blob/master/result/zeromv_skip_1.jpg)


