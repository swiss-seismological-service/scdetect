#! /usr/bin/env python
"""
Utily script for testing the cross-correlation using obspy.
"""

import argparse
import json
import os
import sys

import numpy as np

from pathlib import Path
from copy import deepcopy

from jsonschema import ValidationError, validate
from obspy import read
from obspy.signal.cross_correlation import correlate_template
from rich.console import Console

# relative paths
PATH_SCDETECT_CACHE = "var/cache/scdetect/"
PATH_SCDETECT_TEMP = "var/tmp/scdetect/"

DEBUG_INFO_FNAME = "debug_info.json"

_DEBUG_INFO_CC_SCHEMA = {
    "type": "object",
    "properties": {
        "streamId": {"type": "string"},
        "pathTrace": {"type": "string"},
        "pathTemplate": {"type": "string"},
        "fit": {"type": "number", "minimum": -1, "maximum": 1},
        "lag": {"type": "number"},
    },
    "required": ["streamId", "pathTrace", "pathTemplate", "fit", "lag"],
}


console = Console()


def dir_path(p):
    p = os.path.abspath(os.path.expanduser(p))
    if not os.path.isdir(p):
        raise argparse.ArgumentError(f"Invalid path: {p}")
    return p


def main(argv=None):

    parser = argparse.ArgumentParser(
        description="Test `scdetect` cross-correlation using obspy"
    )
    # optional arguments
    parser.add_argument(
        "--path-seiscomp-install",
        metavar="PATH",
        dest="path_sc_install",
        default="~/seiscomp",
        type=dir_path,
        help=(
            "Path to the SeisComP installation direcory (default:"
            " %(default)s)."
        ),
    )
    parser.add_argument(
        "--xcorr-method",
        default="direct",
        choices=("direct", "fft", "auto"),
        type=str,
        help=(
            "Method to use to calculate the correlation (choices:"
            " {%(choices)s}, default: %(default)s)."
        ),
    )
    parser.add_argument(
        "--detector-ids",
        metavar="ID",
        dest="detector_ids",
        type=str,
        nargs="+",
        help="Restrict stream identifiers to detectors specified by ID",
    )
    parser.add_argument(
        "--stream-ids",
        metavar="STREAM_ID",
        dest="stream_ids",
        nargs="+",
        type=str,
        help="Stream identifiers to be tested.",
    )

    args = parser.parse_args(args=argv)

    path_cache = Path(os.path.join(args.path_sc_install, PATH_SCDETECT_CACHE))
    path_temp = Path(os.path.join(args.path_sc_install, PATH_SCDETECT_TEMP))

    if not all(p.is_dir() for p in [path_cache, path_temp]):
        # nothing to do
        return

    detector_ids = args.detector_ids
    if detector_ids is None:
        detector_ids = [
            p.name
            for p in path_temp.iterdir()
            if p.is_dir() and (p / DEBUG_INFO_FNAME).is_file()
        ]

    num_tests_ok = 0
    num_tests_failed = 0
    for detector_id in detector_ids:
        console.rule(f"Detector: [bold green]{detector_id}")
        print(f"Testing detector with id: {detector_id} ...")
        debug_info_json = None
        try:
            with open(str(path_temp / detector_id / DEBUG_INFO_FNAME)) as ifd:
                debug_info_json = json.load(ifd)

        except (OSError, json.JSONDecodeError) as err:
            print(err)
            continue

        cc_debug_info = debug_info_json.get("ccDebugInfo")
        if cc_debug_info is None:
            continue

        for obj in cc_debug_info:

            try:
                validate(instance=obj, schema=_DEBUG_INFO_CC_SCHEMA)
            except ValidationError as err:
                print(err)
                continue

            if not (
                args.stream_ids is None or obj["streamId"] in args.stream_ids
            ):
                continue

            path_template = obj["pathTemplate"]
            print(f"Loading template waveform from file: {path_template}")
            # there is only a single trace per stream available
            trace_template = None
            try:
                with open(str(path_template), "rb") as ifd:
                    trace_template = read(ifd)[0]
            except OSError as err:
                print(err)
                continue
            else:
                print(
                    f"Time window: [{trace_template.stats.starttime} -"
                    f" {trace_template.stats.endtime}]"
                )

            path_trace = obj["pathTrace"]
            print(f"Loading trace data from file: {path_trace}")
            trace_trace = None
            try:
                with open(str(path_trace), "rb") as ifd:
                    # there is only a single trace per stream available
                    trace_trace = read(ifd)[0]
            except OSError as err:
                print(err)
                continue

            cc = correlate_template(
                trace_trace, trace_template, method=args.xcorr_method
            )
            idx = np.argmax(cc)
            lag = idx / trace_trace.stats.sampling_rate

            result = deepcopy(obj)
            result["fit"] = round(cc[idx], 6)
            result["lag"] = round(lag, 6)

            obj_set = set(obj.items())
            result_set = set(result.items())

            diff = obj_set ^ result_set

            if diff:
                console.print(f"[bold red]Failed: diff={diff}")
                num_tests_failed += 1
            else:
                console.print("[bold green]Ok")
                num_tests_ok += 1

    console.rule("[bold white]Result")
    console.print(
        f"Total: [bold white]{num_tests_ok + num_tests_failed}[/]; [bold"
        f" green]Ok: {num_tests_ok}[/]; [bold red]Failed: {num_tests_failed}"
    )


if __name__ == "__main__":
    main(sys.argv[1:])
