"""Command line entry point:  python -m follow_me --help"""
import argparse
from pathlib import Path

from .config import Config
from .metrics import compute_metrics, print_metrics
from .sim import run


def main():
    ap = argparse.ArgumentParser(description="Drone follow-me teaching simulation")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=None, help="[s]")
    ap.add_argument("--occlusion", type=float, nargs=2, action="append", metavar=("T0", "T1"),
                    help="blind the camera between T0 and T1 [s] (repeatable)")
    ap.add_argument("--no-plots", action="store_true", help="only print the metrics")
    ap.add_argument("--save", type=Path, default=None, help="save figures to this folder")
    ap.add_argument("--animate", action="store_true", help="show a top-view animation")
    ap.add_argument("--speed", type=float, default=4.0, help="animation speed factor")
    args = ap.parse_args()

    cfg = Config()
    cfg.sim.seed = args.seed
    cfg.camera.occlusions = [tuple(o) for o in (args.occlusion or [])]
    if args.duration:
        cfg.sim.duration = args.duration

    log = run(cfg)
    metrics, extra = compute_metrics(log)
    print("Metrics:")
    print_metrics(metrics)
    if args.no_plots and not args.save:
        return

    import matplotlib
    if args.no_plots:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from . import plots

    figs = {"overview": plots.fig_overview(log, extra),
            "estimation": plots.fig_estimation(log, extra),
            "camera_control": plots.fig_camera_control(log)}
    if args.save:
        args.save.mkdir(parents=True, exist_ok=True)
        for name, f in figs.items():
            f.savefig(args.save / f"{name}.png", dpi=110)
        print(f"Figures saved in {args.save}/")
    if args.animate:
        _, anim = plots.animate(log, speed=args.speed)   # keep a reference or it is GC'd
    if not args.no_plots:
        plt.show()


if __name__ == "__main__":
    main()
