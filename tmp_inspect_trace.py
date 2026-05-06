import gzip
import json
import sys


def main() -> None:
    path = sys.argv[1]
    tail = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    with gzip.open(path, "rt", encoding="utf-8") as f:
        trace = json.load(f)
    print(f"frames {len(trace)}")
    for frame in trace[max(0, len(trace) - tail) :]:
        s = frame["state"]
        print(
            f"tick={frame['tick']} x={s['x']:.3f} y={s['y']:.3f} "
            f"vy={s['vy']:.3f} on_ground={s['on_ground']} on_slope={s['on_slope']} "
            f"slope_exit_vy={s['slope_exit_vy']:.3f} mode={s['mode']} gravity={s['gravity_sign']}"
        )


if __name__ == "__main__":
    main()
