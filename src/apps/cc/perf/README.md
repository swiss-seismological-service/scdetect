# scdetect-cc benchmarking

This manual describes how to run the `scdetect-cc` application benchmarks.

## Generating required data

A `Makefile` is prepared in order to generate the benchmark data. Invoke the
machinery with e.g.

```
$ make config BUILD_DIR=~/work/projects/seiscomp/build \
  SEISCOMP_ROOT=~/seiscomp
```

where the `BUILD_DIR` is
the [CMake project build directory](https://cmake.org/cmake/help/latest/manual/cmake.1.html)
. Setting this variable is required since `scdetect-cc` application benchmark
suite related utilities are build, only (i.e. not installed).

## Running the benchmarks

In order to run benchmarks simply make use of the benchmarking driver script
`perf.py`:

```
$ ./perf.py -h
usage: perf.py [-h] [--debug] [--plot] [--trials NUM] [--data-size N]
               [--estimate-overload-capacity]
               PATH_BINARY PATH_DATA

Benchmark scdetect-cc

positional arguments:
  PATH_BINARY           path to binary executable
  PATH_DATA             base path to data

optional arguments:
  -h, --help            show this help message and exit
  --debug               debug mode (default: False)
  --plot                display a plot (default: False)
  --trials NUM          number of trials (default: 3)
  --data-size N         run benchmark based on a waveform data size of N
                        minutes per stream (default: 10)
  --estimate-overload-capacity
                        estimate scdetect-cc's real-time overload capacity
                        (default: False)
```

`scdetect-cc` application benchmarks are focused towards so called
*Three-Stream-Detectors* (AKA station detectors) under the consideration of
different sampling frequencies (i.e. 50 Hz, 100 Hz and 200 Hz) including
different template waveform lengths (i.e. from 2 s to 20 s). Particularly, it is
both the cross-correlation performance and the detection performance which is
measured.

## Example

TODO

## Limitations

At the time being, `scdetect-cc` application benchmarks do not cover:

- benchmarking the linker performance characteristics in case of a lower
  `"triggerOnThreshold"`s.
- benchmarking the linker performance characteristics in case of more (i.e. > 3)
  streams to be associated.
- benchmarking the overall application performance characteristics in case of
  higher sampling frequencies (e.g. in the kHz and MHz range)
- benchmarking the application performance characteristics in case of amplitude
  / magnitude calculation enabled.
