# SCDetect - Computationally efficient earthquake detection

[![continuous-integration](https://github.com/swiss-seismological-service/scdetect/actions/workflows/continuous-integration.yml/badge.svg)](https://github.com/swiss-seismological-service/scdetect/actions/workflows/continuous-integration.yml) [![Documentation Status](https://readthedocs.org/projects/scdetect/badge/?version=latest)](https://scdetect.readthedocs.io/) [![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

## About

SCDetect is a [SeisComP](https://github.com/SeisComP) package. With the
extension module `scdetect-cc` it implements both real-time and classical
offline earthquake detection based on waveform cross-correlation, also called
matched filtering or template matching. Again, the underlying cross-correlation
algorithm is based on computing
the [Pearson Correlation Coefficient](https://en.wikipedia.org/wiki/Pearson_correlation_coefficient)
.

The module allows both single-stream and multi-stream earthquake detection.

In case the detection parameters exceed the configured thresholds, `scdetect-cc`
declares a new origin.

Besides, magnitudes may be estimated based on multiple magnitude estimation
methods (regression, amplitude ratios).

## Documentation

For user documentation please refer to [scdetect.readthedocs.io](https://scdetect.readthedocs.io/).

## Compiling and Installation

### Clone

Get a copy of
[SeisComP](https://github.com/SeisComP):

```bash
TAG='5.3.0'
# get a copy of SeisComP (follow the official documentation)
git clone --branch $TAG https://github.com/SeisComP/seiscomp.git 
git clone --branch $TAG https://github.com/SeisComP/common.git seiscomp/src/base/common
git clone --branch $TAG https://github.com/SeisComP/main.git seiscomp/src/base/main

# [... etc ...]

```

Next, clone SCDetect:

```bash
git clone https://github.com/swiss-seismological-service/scdetect.git seiscomp/src/extras/scdetect
```

### Dependencies

Besides of
the [SeisComP core dependencies](https://github.com/SeisComP/seiscomp#prerequisites)
the following packages must be installed to compile SCDetect:

- `libsqlite3-dev` (Debian, Ubuntu), `sqlite-devel` (RedHat, Fedora, CentOS),
  `dev-db/sqlite` (Gentoo)

E.g. on Ubuntu simply invoke:

```
sudo apt-get install libsqlite3-dev
```

### Compile and Install

For compiling and installing SeisComP (including SCDetect), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

> **NOTE**: executing SCDetect related tests requires SeisComP to be installed,
> beforehand.

In order to run all SeisComP tests (including those of `scdetect-cc` and
possibly additionally installed third party modules), either execute

```bash
make test
```

in the build directory, or use the
[ctest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) executable from
[cmake](https://cmake.org/) (also within the build directory). E.g. in order to
run only SCDetect related tests, invoke

```bash
ctest -R "^test_scdetect.*"
```

For additional information, please also refer to
SeisComP's [unit testing guide](https://docs.gempa.de/seiscomp/current/base/tests.html)
.

## Issues

Please report bugs, issues, feature requests, etc on
[GitHub](https://github.com/swiss-seismological-service/scdetect/issues).

## Contributions

Contributions are very welcome. Made with :two_hearts::rainbow:.

## Cite

*Armbruster, D., Mesimeri, M., Kästli, P., Diehl, T., Massin, F., and Wiemer,
S.* (2022)<br>
SCDetect: Near real-time computationally efficient waveform cross-correlation
based earthquake detection during intense earthquake sequences<br>
*EGU General Assembly 2022, Vienna, Austria, 23–27 May 2022*
EGU22-12443<br>
DOI: https://doi.org/10.5194/egusphere-egu22-12443

> **NOTE**: A manuscript is currently in preparation.



## License

Licensed under the the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.en.html).
For more information see the
[LICENSE](https://github.com/swiss-seismological-service/scdetect/tree/master/LICENSE) file.
