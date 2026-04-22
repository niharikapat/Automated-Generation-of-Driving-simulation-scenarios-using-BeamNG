import os
import math
import argparse
import pandas as pd
from db_folder.db import get_conn

def load_session_points(session_id=None):
    conn = get_conn()

    if session_id is None:
        df_session = pd.read_sql(
            "SELECT id FROM sessions ORDER BY started_at DESC LIMIT 1;",
            conn,
        )
        if df_session.empty:
            conn.close()
            raise ValueError("No sessions found")
        session_id = int(df_session.iloc[0]["id"])

    df_points = pd.read_sql(
        """
        SELECT
            session_id,
            ts,
            lat,
            lon,
            gps_lat,
            gps_lon,
            x_local,
            y_local,
            z_local,
            dist_to_road_m
        FROM gps_points
        WHERE session_id = %s
        ORDER BY ts;
        """,
        conn,
        params=(session_id,),
    )

    conn.close()
    return session_id, df_points


def load_all_sessions():
    conn = get_conn()
    df_sessions = pd.read_sql(
        """
        SELECT
            id,
            scenario_name,
            vehicle_id,
            started_at
        FROM sessions
        ORDER BY started_at;
        """,
        conn,
    )
    conn.close()
    return df_sessions


def path_deviation_analysis(df):
    d = df["dist_to_road_m"].dropna().astype(float)

    if d.empty:
        return None

    result = {
        "GPS Points": len(d),
        "Avg Deviation (m)": d.mean(),
        "Max Deviation (m)": d.max(),
        "Min Deviation (m)": d.min(),
        "Std Deviation (m)": d.std(ddof=1) if len(d) > 1 else 0.0,
        "% Within 2 m": (d <= 2.0).mean() * 100.0,
        "% Within 3 m": (d <= 3.0).mean() * 100.0,
    }
    return result


def compute_total_distance(df):
    pts = df[["x_local", "y_local"]].dropna().astype(float).to_numpy()

    if len(pts) < 2:
        return 0.0, [], []

    segment_lengths = []
    headings = []

    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        dist = math.sqrt(dx * dx + dy * dy)
        segment_lengths.append(dist)

        if dist > 1e-9:
            headings.append(math.degrees(math.atan2(dy, dx)))

    return sum(segment_lengths), segment_lengths, headings


def trajectory_smoothness_analysis(df):
    total_distance, segment_lengths, headings = compute_total_distance(df)

    if len(segment_lengths) == 0 or len(headings) < 2:
        return None

    heading_changes = []
    for i in range(1, len(headings)):
        delta = headings[i] - headings[i - 1]

        while delta > 180:
            delta -= 360
        while delta < -180:
            delta += 360

        heading_changes.append(abs(delta))

    if not heading_changes:
        return None

    abrupt_turn_threshold_deg = 45.0
    abrupt_turns = sum(1 for x in heading_changes if x > abrupt_turn_threshold_deg)
    smooth_turns = len(heading_changes) - abrupt_turns
    smoothness_pct = (smooth_turns / len(heading_changes)) * 100.0

    result = {
        "Total Distance (m)": total_distance,
        "Segments": len(segment_lengths),
        "Avg Segment Length (m)": sum(segment_lengths) / len(segment_lengths),
        "Max Segment Length (m)": max(segment_lengths),
        "Avg Heading Change (deg)": sum(heading_changes) / len(heading_changes),
        "Max Heading Change (deg)": max(heading_changes),
        "Abrupt Turns": abrupt_turns,
        "Smooth Turns": smooth_turns,
        "Smoothness (%)": smoothness_pct,
    }
    return result


def session_duration_analysis(df):
    if df.empty or "ts" not in df.columns or len(df) < 2:
        return None

    ts = pd.to_datetime(df["ts"], errors="coerce").dropna()
    if len(ts) < 2:
        return None

    duration_s = (ts.iloc[-1] - ts.iloc[0]).total_seconds()

    intervals = ts.diff().dropna().dt.total_seconds()
    avg_interval = intervals.mean() if not intervals.empty else 0.0
    max_interval = intervals.max() if not intervals.empty else 0.0

    result = {
        "Duration (s)": duration_s,
        "Avg Sampling Interval (s)": avg_interval,
        "Max Sampling Interval (s)": max_interval,
    }
    return result


def classify_path_deviation(avg_dev):
    if avg_dev < 1.5:
        return "Excellent"
    elif avg_dev < 3.0:
        return "Good"
    elif avg_dev < 5.0:
        return "Acceptable"
    return "Poor"


def classify_smoothness(smoothness_pct):
    if smoothness_pct >= 95:
        return "Excellent"
    elif smoothness_pct >= 90:
        return "Good"
    elif smoothness_pct >= 80:
        return "Acceptable"
    return "Poor"


def print_metric_block(title, metrics):
    print(f"\n=== {title} ===")
    if not metrics:
        print("No data available.")
        return

    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"{key}: {value:.2f}")
        else:
            print(f"{key}: {value}")


def build_single_session_summary(session_id, df):
    path_metrics = path_deviation_analysis(df)
    smooth_metrics = trajectory_smoothness_analysis(df)
    duration_metrics = session_duration_analysis(df)

    summary = {
        "Session ID": session_id,
        "GPS Points": path_metrics["GPS Points"] if path_metrics else None,
        "Avg Deviation (m)": path_metrics["Avg Deviation (m)"] if path_metrics else None,
        "Max Deviation (m)": path_metrics["Max Deviation (m)"] if path_metrics else None,
        "% Within 2 m": path_metrics["% Within 2 m"] if path_metrics else None,
        "Total Distance (m)": smooth_metrics["Total Distance (m)"] if smooth_metrics else None,
        "Smoothness (%)": smooth_metrics["Smoothness (%)"] if smooth_metrics else None,
        "Duration (s)": duration_metrics["Duration (s)"] if duration_metrics else None,
    }

    if summary["Avg Deviation (m)"] is not None:
        summary["Path Result"] = classify_path_deviation(summary["Avg Deviation (m)"])
    else:
        summary["Path Result"] = None

    if summary["Smoothness (%)"] is not None:
        summary["Smoothness Result"] = classify_smoothness(summary["Smoothness (%)"])
    else:
        summary["Smoothness Result"] = None

    return summary


def repeatability_analysis(session_ids):
    rows = []

    for sid in session_ids:
        _, df = load_session_points(sid)
        if df.empty:
            continue

        summary = build_single_session_summary(sid, df)
        rows.append(summary)

    if not rows:
        return None, None

    df_runs = pd.DataFrame(rows)

    numeric_cols = [
        "GPS Points",
        "Avg Deviation (m)",
        "Max Deviation (m)",
        "% Within 2 m",
        "Total Distance (m)",
        "Smoothness (%)",
        "Duration (s)",
    ]

    repeatability = {}
    for col in numeric_cols:
        values = pd.to_numeric(df_runs[col], errors="coerce").dropna()
        if not values.empty:
            repeatability[f"{col} Range"] = values.max() - values.min()
            repeatability[f"{col} Mean"] = values.mean()

    return df_runs, repeatability


def save_csv(df, filename):
    os.makedirs("./output", exist_ok=True)
    path = os.path.join("./output", filename)
    df.to_csv(path, index=False)
    print(f"\nCSV written to {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-id", type=int, default=None, help="Evaluate one specific session")
    parser.add_argument(
        "--repeatability",
        action="store_true",
        help="Evaluate repeatability across all sessions",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Save summary tables to CSV in ./output",
    )
    args = parser.parse_args()

    if args.repeatability:
        df_sessions = load_all_sessions()
        if df_sessions.empty:
            print("No sessions found.")
            return

        session_ids = df_sessions["id"].astype(int).tolist()
        df_runs, repeatability = repeatability_analysis(session_ids)

        print("\n==============================")
        print("REPEATABILITY TESTING RESULTS")
        print("==============================")

        if df_runs is not None and not df_runs.empty:
            print("\nPer-run summary:")
            print(df_runs.to_string(index=False))

            if args.csv:
                save_csv(df_runs, "repeatability_summary.csv")

        print_metric_block("Repeatability Variation Summary", repeatability)

    else:
        session_id, df = load_session_points(args.session_id)

        if df.empty:
            print(f"No points found for session {session_id}.")
            return

        path_metrics = path_deviation_analysis(df)
        smooth_metrics = trajectory_smoothness_analysis(df)
        duration_metrics = session_duration_analysis(df)
        summary = build_single_session_summary(session_id, df)

        print("\n==========================")
        print(f"EVALUATION FOR SESSION {session_id}")
        print("==========================")

        print_metric_block("Path Deviation Analysis", path_metrics)
        print_metric_block("Trajectory Smoothness Analysis", smooth_metrics)
        print_metric_block("Timing / Sampling Analysis", duration_metrics)
        print_metric_block("Single Session Summary", summary)

        if args.csv:
            summary_df = pd.DataFrame([summary])
            save_csv(summary_df, f"session_{session_id}_evaluation_summary.csv")


if __name__ == "__main__":
    main()