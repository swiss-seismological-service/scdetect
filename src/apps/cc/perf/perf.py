#!/usr/bin/env python3

# driver script for `scdetect-cc` benchmarking

import argparse
import itertools
import logging
import math
import platform
import subprocess
import re
import sys

from pathlib import Path, PurePath
from collections import defaultdict, namedtuple, Counter

import numpy as np
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.DEBUG)


def ge_one(i):
    try:
        ret = int(i)
    except ValueError as err:
        raise argparse.ArgumentError(f"invalid number: {i} ({str(err)})")

    if ret < 1:
        raise argparse.ArgumentParser(f"invalid number: must be >= 1")

    return ret


def file_path(p):
    try:
        ret = Path(p).resolve()
    except OSError as err:
        raise argparse.ArgumentError(f"invalid path: {p} ({str(err)})")

    if not ret.is_file():
        raise argparse.ArgumentError(f"invalid path: {p} (must be a file)")

    return ret


def get_cpu_info():
    if platform.system() == "Linux":
        cmd = ["cat", "/proc/cpuinfo"]
        info = subprocess.check_output(cmd).strip()
        for line in info.decode("utf-8").split("\n"):
            if "model name" in line:
                return re.sub(".*model name.*:", "", line, 1)
    else:
        return platform.platform()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Benchmark scdetect-cc",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--debug", action="store_true", default=False, help="debug mode"
    )
    parser.add_argument(
        "--plot", action="store_true", default=False, help="display a plot"
    )
    parser.add_argument(
        "--trials",
        type=ge_one,
        metavar="NUM",
        default=3,
        help="number of trials",
    )
    parser.add_argument(
        "--data-size",
        dest="data_size",
        metavar="N",
        type=int,
        default=10,
        choices=[10, 30],
        help="run benchmark based on a waveform data size of N minutes per stream",
    )
    parser.add_argument(
        "--estimate-overload-capacity",
        dest="estimate_overload_capacity",
        action="store_true",
        help="estimate scdetect-cc's real-time overload capacity",
    )
    parser.add_argument(
        "binary",
        type=file_path,
        metavar="PATH_BINARY",
        help="path to binary executable",
    )
    parser.add_argument(
        "data", type=Path, metavar="PATH_DATA", help="base path to data"
    )

    return parser


def run_perf_app_process(path_binary, trials, cmd):
    args = [str(path_binary), f"--trials={trials}", "--"]
    args.extend(cmd)
    logging.debug(f"Executing {' '.join(args)!r} ...")
    try:
        output = subprocess.check_output(args)
    except:
        logging.critical("failed to execute subprocess")
        raise

    t = 0
    records_total = 0
    samples_total = 0
    sampling_frequency = 0
    num_template_procs = 0
    samples_template_waveform = 0
    detector_config = Counter()
    for line in output.decode("utf8").split("\n"):
        if line.startswith("time:"):
            t = float(line.split(":")[1].split()[0])
        elif line.startswith("records (total):"):
            records_total = int(line.split(":")[1])
        elif line.startswith("samples (total):"):
            samples_total = int(line.split(":")[1])
        elif line.startswith("sampling frequency:"):
            sampling_frequency = float(line.split(":")[1].split()[0])
        elif line.startswith("template waveform samples ["):
            s = line.split(":")
            detector_config[s[0].split("[")[1]] += 1
            samples = int(s[-1])
            if 0 == samples_template_waveform:
                samples_template_waveform = samples
            elif (
                0 != samples_template_waveform
                and samples_template_waveform != samples
            ):
                raise ValueError(
                    "configuration error: template waveform lengths differ"
                )

    return Sample(
        time=t,
        records_total=records_total,
        samples_total=samples_total,
        sampling_frequency=sampling_frequency,
        samples_template_waveform=samples_template_waveform,
        detector_config=detector_config,
        template_waveform_length=samples_template_waveform
        / sampling_frequency,
    )


def run_benchmark(
    path_binary,
    trials,
    path_data,
    waveform_data_size,
    estimate_overload_capacity=True,
    debug_mode=False,
):
    report = ThreeStreamDetectorReport(
        waveform_data_size, estimate_overload_capacity
    )

    sampling_frequencies = [50, 100, 200]
    num_detectors = [1, 2, 4, 8]
    samples = [f"{x:04}" for x in range(0, 11)]

    fname_templates_json = "templates.json"
    fname_inventory = "inventory.scml"
    fname_catalog = "catalog.scml"
    fname_waveform_data = f"data.{waveform_data_size}.mseed"
    fname_config = "scdetect-cc.cfg"

    def create_cmd(path_sample_cfg, debug_mode):
        flags = [
            FlagOffline(),
            FlagPlayback(),
            FlagTemplatesReload(),
            FlagAmplitudesForce(),
            FlagConfigFile(path_sample_cfg / fname_config),
            FlagTemplatesJSON(path_sample_cfg / fname_templates_json),
            FlagInventoryDB(path_sample_cfg / fname_inventory),
            FlagEventDB(path_sample_cfg / fname_catalog),
            FlagRecordStreamURL(path_sample_cfg / fname_waveform_data),
        ]
        if debug_mode:
            flags.append(FlagDebug())

        return (f"{flag}" for flag in flags)

    for sample_cfg in itertools.product(sampling_frequencies, num_detectors):
        for sample in samples:
            path_sample_cfg = (
                path_data
                / f"perf-{sample_cfg[0]}hz"
                / f"perf-{sample_cfg[1]:02}-03"
                / f"perf-{sample}"
            )
            try:
                if not path_sample_cfg.resolve().is_dir():
                    logging.error(f"invalid path: {str(path_sample_cfg)}")
                    return report
            except OSError as err:
                logging.error(f"invalid path: {str(path_sample_cfg)}")
                continue

            sample = run_perf_app_process(
                path_binary, trials, create_cmd(path_sample_cfg, debug_mode)
            )

            report.add(sample)

    return report


Sample = namedtuple(
    "Sample",
    [
        "sampling_frequency",
        "detector_config",
        "time",
        "records_total",
        "samples_total",
        "samples_template_waveform",
        "template_waveform_length",
    ],
)


class ThreeStreamDetectorReport:
    _ESTIMATE_TEMPLATE_WAVEFORM_LENGTHS = [4, 8, 12]

    def __init__(
        self, fed_waveform_data_size, estimate_overload_capacity=True
    ):
        # convert to milliseconds
        self._fed_waveform_data_size = fed_waveform_data_size * 60 * 1000

        self._samples = []

        self._estimate_real_time_capacity = estimate_overload_capacity
        self._cached_models = None

    def add(self, sample):
        self._samples.append(sample)
        self._cached_models = None

    def plot(self):
        def get_subplot_idxs(row, col):
            return (row + 1, 0) if col % 2 == 1 else (row, col + 1)

        def get_ax(axs, row, col, fig_rows):
            if fig_rows == 1:
                return axs[col]
            else:
                return axs[row, col]

        sorted_samples = self._sort_samples_by_frequency(self._samples)

        fig_cols = 2
        fig_rows = round((len(sorted_samples) + 1) / float(fig_cols))
        fig, axs = plt.subplots(
            fig_rows, fig_cols, subplot_kw={"projection": "3d"}
        )
        fig.suptitle(f"scdetect-cc benchmark @ {get_cpu_info()}")
        ax_col = 1
        ax_row = -1
        for k, v in sorted_samples.items():
            converted = np.array(v)
            x = converted.T[1]
            y = converted.T[-1]
            z = converted.T[2]

            ax_row, ax_col = get_subplot_idxs(ax_row, ax_col)
            ax = get_ax(axs, ax_row, ax_col, fig_rows)
            ax.scatter(x, y, z, alpha=0.6)
            ax.set_xlabel("detectors (3 streams each)")
            ax.set_ylabel("template waveform length (s)")
            ax.set_zlabel("time (ms)")
            ax.set_title(f"{k} Hz")

            if not self._estimate_real_time_capacity:
                continue

            self._create_models()

            if not self._cached_models:
                continue

            data = np.array((x, y))
            mn = np.min(data, axis=0)
            mx = np.max(data, axis=1)
            _x, _y = np.meshgrid(
                np.linspace(mn[0], mx[0], 20),
                np.linspace(mn[1], mx[1], 20),
            )
            xx = _x.flatten()
            yy = _y.flatten()

            # evaluate model on the grid
            _z = np.dot(
                np.c_[xx, yy, xx * yy, xx * yy ** 2, yy ** 2],
                self._cached_models[k],
            ).reshape(_x.shape)

            ax.plot_surface(_x, _y, _z, rstride=1, cstride=1, alpha=0.2)

        ax_row, ax_col = get_subplot_idxs(ax_row, ax_col)
        ax = get_ax(axs, ax_row, ax_col, fig_rows)
        ax.set_title(f"normalized time")
        c = 0
        for k, v in sorted_samples.items():
            converted = np.array(v)
            x = converted.T[1]
            y = converted.T[-1]
            z = converted.T[2] / converted.T[0]

            ax.scatter(x, y, z, f"C{c}", label=f"{k} Hz", alpha=0.6)
            ax.set_xlabel("detectors (3 streams each)")
            ax.set_ylabel("template waveform length  (s)")
            ax.set_zlabel("time / sampling frequency (ms / Hz)")

            c += 1

        ax.legend()

    def _create_models(self):
        if self._cached_models:
            return

        sorted_samples = self._sort_samples_by_frequency(self._samples)
        for k, v in sorted_samples.items():
            converted = np.array(v)
            x = converted.T[1]
            y = converted.T[-1]
            z = converted.T[2]

            coeffs, r, rank, s = self._model_data(x, y, z)

            if self._cached_models is None:
                self._cached_models = dict()

            self._cached_models[k] = coeffs

    def _estimate_real_time_overload_capacity(
        self, coeffs, template_waveform_length
    ):
        """
        Estimate the number of detectors to be configured until `scdetect-cc`
        is overloaded.
        """
        t = self._fed_waveform_data_size
        y = template_waveform_length
        x = (t - coeffs[1] * y - coeffs[4] * y ** 2) / (
            coeffs[0] + coeffs[2] * y + coeffs[3] * y ** 2
        )
        return math.floor(x)

    def __str__(self):
        if not self._samples:
            return ""

        ret = "=== Station detector report ===\n"
        ret += (
            "sampling_frequency (Hz),num_detectors,time "
            "(ms),records_total,samples_total,samples_template_waveform,"
            "length_template_waveform (s)\n"
        )

        for sample in self._samples:
            ret += (
                f"{sample.sampling_frequency},"
                f"{len(sample.detector_config)},{sample.time},"
                f"{sample.records_total},{sample.samples_total},"
                f"{sample.samples_template_waveform},"
                f"{sample.template_waveform_length}"
                "\n"
            )

        if self._estimate_real_time_capacity:
            self._create_models()

        if not self._cached_models:
            return ret

        ret += "\n"
        ret += "=== Station detector report - real-time overload capacity estimation ===\n"
        ret += "sampling frequency (Hz),length_template_waveform (s),num_detectors\n"

        for freq, model in self._cached_models.items():
            for l in self._ESTIMATE_TEMPLATE_WAVEFORM_LENGTHS:
                num_detectors = self._estimate_real_time_overload_capacity(
                    model, l
                )
                ret += f"{freq},{l},{num_detectors}\n"

        return ret

    @staticmethod
    def _sort_samples_by_frequency(samples, reduce_detector_config=True):
        ret = defaultdict(list)
        for sample in samples:
            if reduce_detector_config:
                sample = sample._replace(
                    detector_config=len(sample.detector_config)
                )
            ret[f"{sample.sampling_frequency}"].append(sample)
        return ret

    @staticmethod
    def _model_data(x, y, z):
        """
        model the sampled data
        """
        A = np.array(
            [
                x,
                y,
                x * y,
                x * y ** 2,
                y ** 2,
            ]
        ).T
        B = z

        return np.linalg.lstsq(A, B, rcond=None)


class Flag:
    _SEP = "="
    _FLAG = None

    def __init__(self, arg=None):
        self._arg = arg

    def __str__(self):
        return "{}{}".format(
            self._FLAG,
            "{}".format(
                f"{self._SEP}{self._arg}"
                if self._arg not in (None, "")
                else ""
            ),
        )


class FlagInventoryDB(Flag):
    _FLAG = "--inventory"

    def __init__(self, uri):
        if isinstance(uri, PurePath):
            uri = uri.resolve().as_uri()
        super().__init__(uri)


class FlagRecordStreamURL(Flag):
    _FLAG = "--record-url"

    def __init__(self, uri):
        if isinstance(uri, PurePath):
            uri = uri.resolve().as_uri()
        super().__init__(uri)


class FlagEventDB(Flag):
    _FLAG = "--event-db"

    def __init__(self, uri):
        if isinstance(uri, PurePath):
            uri = uri.resolve().as_uri()
        super().__init__(uri)


class FlagTemplatesJSON(Flag):
    _FLAG = "--templates-json"

    def __init__(self, path):
        if isinstance(path, PurePath):
            path = path.resolve()
        super().__init__(path)


class FlagConfigFile(Flag):
    _FLAG = "--config-file"

    def __init__(self, path):
        if isinstance(path, PurePath):
            path = path.resolve()
        super().__init__(path)


class FlagDebug(Flag):
    _FLAG = "--debug"


class FlagOffline(Flag):
    _FLAG = "--offline"


class FlagPlayback(Flag):
    _FLAG = "--playback"


class FlagTemplatesReload(Flag):
    _FLAG = "--templates-reload"


class FlagAmplitudesForce(Flag):
    _FLAG = "--amplitudes-force"

    def __init__(self, enabled=False):
        super().__init__(str(enabled).lower())


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args()

    report = run_benchmark(
        args.binary,
        args.trials,
        args.data,
        args.data_size,
        args.estimate_overload_capacity,
        args.debug,
    )

    if args.plot:
        report.plot()
        plt.show()

    print(report)


if __name__ == "__main__":
    main(sys.argv[1:])
