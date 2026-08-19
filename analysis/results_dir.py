import argparse
import glob
import os

RESULTS_BASE = "../results"


def resolve_results_root():
    """
    Resolve the results/<date>/ directory a sweep script should read raw
    JSON from and write figures into.

    Defaults to the most recently generated date dir under RESULTS_BASE
    (lexicographic sort works since dirs are named YYYY-MM-DD), so a new
    day of sweep data doesn't require editing every script by hand. Pass
    --results-root to point at a specific date instead.
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--results-root",
        default=None,
        help="Path to a results/<date> directory (e.g. ../results/2026-07-13). "
             "Defaults to the most recent date dir under ../results.",
    )
    args, _ = parser.parse_known_args()

    if args.results_root:
        return args.results_root

    candidates = sorted(
        d for d in glob.glob(os.path.join(RESULTS_BASE, "*"))
        if os.path.isdir(os.path.join(d, "raw"))
    )
    if not candidates:
        raise FileNotFoundError(
            f"No results/<date>/raw directories found under {RESULTS_BASE}"
        )
    return candidates[-1]


def resolve_profile():
    """
    Resolve which experiment profile ('legacy' or 'vineyard') a sweep
    script should read raw JSON from -- results are namespaced by profile
    under raw/<profile>/<sweep_name> since ExperimentProfile was added
    (farmbotsim-rs commit 6925e47), which predates these scripts. Pass
    --profile to override; defaults to 'vineyard' (matches the Rust CLI's
    own default in ExperimentProfile::from_flag).
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--profile",
        default="vineyard",
        choices=["legacy", "vineyard"],
        help="Which experiment profile's results to read (default: vineyard).",
    )
    args, _ = parser.parse_known_args()
    return args.profile


def results_root_tag(results_root):
    """
    Short, filesystem-safe tag identifying a results_root, e.g. '2026-07-13'
    for '../results/2026-07-13'. Used to namespace exports by date so
    reruns against different dates don't overwrite each other's CSVs.
    """
    return os.path.basename(os.path.normpath(results_root))
