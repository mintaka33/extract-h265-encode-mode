# extract-h265-encode-mode
Extract video encoding information like motion vector, skip mode, CU/PU/TU partitions from H.265/HEVC stream

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