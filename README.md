# SCDetect - Computationally efficient earthquake detection

## Content

- [About](#about)
- [Installation](#compiling-and-installation)
- [Tests](#tests)
- [Issues](#issues)
- [Contributions](#contributions)
- [License](#license)


## About

TODO

## Compiling and Installation

Get a copy of
[SeisComP/seiscomp](https://github.com/SeisComP/seiscomp):

```bash
git clone https://github.com/SeisComP/seiscomp.git && cd seiscomp/src/extras/
```

Next, clone `scdetect`:

```bash
git clone https://github.com/damb/scdetect.git
```

For compiling SeisComP (including `scdetect`), please refer to
https://github.com/SeisComP/seiscomp#build.

## Tests

In order to run all SeisComP tests (including those of `scdetect` and possibly
additionally installed third party modules), either execute

```bash
make test
```

in the build directory, or use the
[ctest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) executable from
[cmake](https://cmake.org/) (also within the build directory). E.g. in order to
run only `scdetect` related tests, invoke

```bash
ctest -R "^test_scdetect.*"
```

For additional information, please also refer to SeisComP's [unit testing
guide](https://docs.gempa.de/seiscomp/4/current/base/tests.html).

## Issues

Please report bugs, issues, feature requests, etc on
[GitHub](https://github.com/damb/scdetect/issues).

## Contributions

Contributions are very welcome. Made with :two_hearts:.

## License

Licensed under the the [AGPLv3](https://www.gnu.org/licenses/agpl-3.0.en.html).
For more information see the
[LICENSE](https://github.com/damb/scdetect/tree/master/LICENSE) file.
