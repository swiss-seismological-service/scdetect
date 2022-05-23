# SCDetect - Computationally efficient earthquake detection

[![continuous-integration](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml/badge.svg)](https://github.com/damb/scdetect/actions/workflows/continuous-integration.yml) [![Documentation Status](https://readthedocs.org/projects/scdetect/badge/?version=latest)](https://scdetect.readthedocs.io/?badge=latest)

## Content

- [About](#about)
- [Installation](#compiling-and-installation)
- [Tests](#tests)
- [Issues](#issues)
- [Contributions](#contributions)
- [Cite](#cite)
- [License](#license)

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
# get a copy of SeisComP (follow the official documentation)
git clone https://github.com/SeisComP/seiscomp.git && cd seiscomp/src/base
git clone https://github.com/SeisComP/common.git
git clone https://github.com/SeisComP/main.git

# [... etc ...]

```

Next, clone SCDetect:

```bash
cd seiscomp/src/extras && git clone https://github.com/damb/scdetect.git
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

### Compile

For compiling SeisComP (including SCDetect), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

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
[GitHub](https://github.com/damb/scdetect/issues).

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
[LICENSE](https://github.com/damb/scdetect/tree/master/LICENSE) file.
