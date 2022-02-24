#!/usr/bin/env python3

import argparse
import logging
import shutil
import subprocess
import sys

from pathlib import Path

logging.basicConfig(level=logging.DEBUG)


def resolved_exec_path(p):
    ret = Path(p)
    try:
        if len(ret.parts) == 1:
            ret = Path(shutil.which(ret)).resolve()
        else:
            ret.resolve()
    except OSError as err:
        raise argparse.ArgumentError(f"invalid path: {p} ({str(err)})")

    if not ret.is_file():
        raise argparse.ArgumentError(f"invalid path: {p} (must be a file)")

    return ret


def file_path(p):
    try:
        ret = Path(p).resolve()
    except OSError as err:
        raise argparse.ArgumentError(f"invalid path: {p} ({str(err)})")

    if not ret.is_file():
        raise argparse.ArgumentError(f"invalid path: {p} (must be a file)")

    return ret


def build_parser():
    parser = argparse.ArgumentParser(
        description="Prepare waveform data for scdetect-cc app benchmarks",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output",
        type=str,
        metavar="FNAME",
        default="data.mseed",
        help="dump waveform data to FNAME",
    )
    parser.add_argument(
        "--keep-tempfiles", action="store_true", help="keep temporary files"
    )
    parser.add_argument(
        "--target-frequency",
        type=int,
        metavar="FREQ",
        default=0,
        help="target sampling frequency",
    )
    parser.add_argument(
        "--starttime",
        type=str,
        metavar="TIME",
        help="trim waveform data to starttime TIME",
    )
    parser.add_argument(
        "--endtime",
        type=str,
        metavar="TIME",
        help="trim waveform data to endtime TIME",
    )
    parser.add_argument(
        "--binary-scmssort",
        type=resolved_exec_path,
        metavar="PATH",
        default="scmssort",
        help="path to scmssort binary executable",
    )
    parser.add_argument(
        "--binary-prepare",
        type=resolved_exec_path,
        metavar="PATH",
        default="perf_util_scdetect_cc_prepare_waveform_data",
        help="path to prepare backend binary executable",
    )

    parser.add_argument(
        "record_stream_uri",
        metavar="URI",
        type=str,
        help="archive recordStream URI",
    )
    parser.add_argument(
        "template_config_paths",
        type=file_path,
        metavar="PATH",
        nargs="+",
        help="path to JSON template configuration files",
    )

    return parser


def run_prepare_backend_process(path_binary, args):
    cmd = [str(path_binary)]
    cmd.extend(args)
    logging.debug(f"Executing {' '.join(cmd)!r} ...")
    try:
        output = subprocess.check_output(cmd)
    except:
        logging.critical(f"failed to execute subprocess: {' '.join(cmd)!r}")
        raise


def run_scmssort_process(path_binary, path_dest, keep_tempfiles=False):
    cat_cmd = ["cat"]

    paths_waveform_data = sorted(path_dest.parent.glob("*.*.*.*.mseed"))
    for p in paths_waveform_data:
        cat_cmd.append(f"{str(p)}")

    if len(cat_cmd) == 1:
        logging.critical(f"nothing to sort")
        return

    try:
        logging.debug(f"Executing {' '.join(cat_cmd)!r} ...")
        cat = subprocess.Popen(cat_cmd, stdout=subprocess.PIPE)
        scmssort_cmd = [str(path_binary), "-u", "-E"]
        logging.debug(
            f"Executing {' '.join(scmssort_cmd)!r} > {str(path_dest)} (reading stdin) ..."
        )
        with path_dest.open(mode="bw") as ofd:
            subprocess.run(
                scmssort_cmd, check=True, stdin=cat.stdout, stdout=ofd
            )
        cat.wait()
    except:
        logging.critical(f"failed to execute scmssort subprocess")
        raise

    if not keep_tempfiles:
        logging.debug("Removing temporary files ...")
        for p in paths_waveform_data:
            p.unlink(missing_ok=True)


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


class FlagStartTime(Flag):
    _FLAG = "--starttime"

    def __init__(self, arg):
        super().__init__(arg)


class FlagEndTime(Flag):
    _FLAG = "--endtime"

    def __init__(self, arg):
        super().__init__(arg)


class FlagTargetFrequency(Flag):
    _FLAG = "--target-frequency"

    def __init__(self, arg):
        super().__init__(arg)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args()

    for template_config_path in args.template_config_paths:
        cmd_args = [FlagTargetFrequency(args.target_frequency)]
        if args.starttime:
            cmd_args.append(FlagStartTime(args.starttime))
        if args.endtime:
            cmd_args.append(FlagEndTime(args.endtime))

        cmd_args.append(args.record_stream_uri)
        cmd_args.append(template_config_path)

        cmd_args = [f"{str(a)}" for a in cmd_args]
        run_prepare_backend_process(args.binary_prepare, cmd_args)
        run_scmssort_process(
            args.binary_scmssort,
            template_config_path.parent / args.output,
            args.keep_tempfiles,
        )


if __name__ == "__main__":
    main(sys.argv[1:])
