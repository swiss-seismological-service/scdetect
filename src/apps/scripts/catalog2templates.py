#! /usr/bin/env python
"""
Utily script converting a catalog into a scdetect compatible JSON template
configuration file.

Note that a catalog files may be fetched via e.g. fdsnws-event.
"""


import sys
import argparse
import functools
import json
import re

from collections import ChainMap

from obspy import read_events


def regex_str(pattern):
    try:
        return re.compile(pattern)
    except re.error as e:
        raise argparse.ArgumentError(e.msg)


def parse_catalog(catalog, exclude_pattern=None, phases=("Pg", "Sg")):
    def filter_phase(arrival, phases):
        return arrival.phase in phases

    def filter_picks(pick, pick_ids):
        return pick.resource_id in pick_ids

    config_dicts = []
    for event in catalog:

        _config_dicts = []
        for origin in event.origins:

            if not origin.arrivals:
                continue

            for phase in phases:
                arrivals = filter(
                    functools.partial(filter_phase, phases=[phase]),
                    origin.arrivals,
                )

                if not arrivals:
                    continue

                config_dict = {}
                config_dict["originId"] = str(origin.resource_id)
                config_dict["phase"] = phase
                config_dict["pick_ids"] = [
                    arrival.pick_id for arrival in arrivals
                ]
                _config_dicts.append(config_dict)

        for _dict in _config_dicts:
            if not _dict["pick_ids"]:
                continue

            picks = filter(
                functools.partial(filter_picks, pick_ids=_dict["pick_ids"]),
                event.picks,
            )

            # wrap streams into their dedicated stream-set; currently only a
            # single stream-set is supported
            _dict["streams"] = [
                [
                    {
                        "waveformId": p.waveform_id.get_seed_string(),
                        "templateWaveformId": p.waveform_id.get_seed_string(),
                        "templatePhase": _dict["phase"],
                    }
                    for p in picks
                    if (
                        exclude_pattern is None
                        or not exclude_pattern.match(
                            p.waveform_id.get_seed_string()
                        )
                    )
                ]
            ]

            if not _dict["streams"][0]:
                continue

            del _dict["pick_ids"]
            del _dict["phase"]

            config_dicts.append(_dict)

    return config_dicts


class TemplateConfig:
    def __init__(self, config_dicts, detector_defaults={}, stream_defaults={}):
        self._config_dicts = config_dicts
        self._detector_defaults = detector_defaults
        self._stream_defaults = stream_defaults

    def render(self):
        config_dicts = []
        if self._stream_defaults:
            for _dict in self._config_dicts:
                # consider stream-sets
                _dict["streams"] = [
                    [
                        dict(ChainMap(stream_config, self._stream_defaults))
                        for stream_config in _dict["streams"][0]
                    ]
                ]

                if self._detector_defaults:
                    config_dicts.append(
                        dict(ChainMap(_dict, self._detector_defaults))
                    )
                else:
                    config_dicts.append(_dict)

        elif self._detector_defaults:
            config_dicts = [
                dict(ChainMap(_dict, self._detector_defaults))
                for _dict in self._config_dicts
            ]

        else:
            config_dicts = self._config_dicts

        return json.dumps(config_dicts)


def main(argv=None):

    parser = argparse.ArgumentParser(
        description="Create template configurations from a seismic catalog"
    )
    # optional arguments
    parser.add_argument(
        "--format",
        choices=("sc3ml", "quakeml"),
        default="sc3ml",
        type=str,
        help="Catalog format.",
    )
    parser.add_argument(
        "--filter-events",
        type=str,
        help=(
            "Filter to be applied to events while parsing the catalog. The "
            " filter syntax follows"
            " https://docs.obspy.org/packages/autogen/"
            "obspy.core.event.Catalog.filter.html."
        ),
    )
    parser.add_argument(
        "-e",
        "--exclude-streams",
        type=regex_str,
        metavar="REGEX",
        default=None,
        dest="exclude_pattern",
        help=(
            "Exclude streams when the waveform stream identifier matches"
            " REGEX. For valid regular expressions visit"
            " https://docs.python.org/3/library/re.html."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        metavar="PATH",
        type=argparse.FileType("w"),
        default=sys.stdout,
        help="Write output to PATH instead of stdout.",
    )
    parser.add_argument(
        "--detector-init-time",
        metavar="SECS",
        type=int,
        default=60,
        help="Detector related initialization time.",
    )
    parser.add_argument(
        "--template-wf-start",
        metavar="SECS",
        type=int,
        default=-2,
        help=(
            "Template related waveform start time in seconds w.r.t. the pick"
            " time."
        ),
    )
    parser.add_argument(
        "--template-wf-end",
        metavar="SECS",
        type=int,
        default=2,
        help=(
            "Template related waveform end time in seconds w.r.t. the pick"
            " time."
        ),
    )

    # positional arguments
    parser.add_argument(
        "catalog",
        metavar="CATALOG",
        type=argparse.FileType("r"),
        default=sys.stdin,
        help="Path to catalog to be parsed.",
    )

    args = parser.parse_args(args=argv)

    # validate cli arguments
    if args.template_wf_start >= args.template_wf_end:
        parser.error(
            "Invalid parameter(s) for template waveform limits:"
            f" {args.template_wf_start} >= {args.template_wf_end}"
        )

    cat = read_events(args.catalog, args.format)

    if args.filter_events:
        try:
            cat = cat.filter(args.filter_events)
        except ValueError as err:
            print(parser.error(str(err)))

    detector_defaults = {"initTime": args.detector_init_time}
    stream_defaults = {
        "templateWaveformStart": args.template_wf_start,
        "templateWaveformEnd": args.template_wf_end,
    }

    template_config = TemplateConfig(
        parse_catalog(cat, exclude_pattern=args.exclude_pattern),
        detector_defaults=detector_defaults,
        stream_defaults=stream_defaults,
    )
    print(template_config.render(), file=args.output)


if __name__ == "__main__":
    main(sys.argv[1:])
