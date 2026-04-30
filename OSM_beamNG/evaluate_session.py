import math
import argparse
import pandas as pd
from db_folder.db import get_conn


def run_query(query, params=None):
    conn = get_conn()
    try:
        return pd.read_sql(query, conn, params=params)
    finally:
        conn.close()


def load_session_points(session_id=None):
    if session_id is None:
        df_session = run_query("SELECT id FROM sessions ORDER BY started_at DESC LIMIT 1;")
        if df_session.empty:
            raise ValueError("No sessions found")
        session_id = int(df_session.iloc[0]["id"])

    df_points = run_query(
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
        params=(session_id,),
    )
    return session_id, df_points


def load_all_sessions():
    return run_query(
        """
        SELECT
            id,
            scenario_name,
            vehicle_id,
            started_at
        FROM sessions
        ORDER BY started_at DESC LIMIT 5;
        """
    )


def path_deviation_analysis(df):
    d = df["dist_to_road_m"].dropna().astype(float)
    if d.empty:
        return None

    return {
        "GPS Points": len(d),
        "Avg Deviation (m)": d.mean(),
        "Max Deviation (m)": d.max(),
        "Min Deviation (m)": d.min(),
        "Std Deviation (m)": d.std(ddof=1) if len(d) > 1 else 0.0,
        "% Within 2 m": (d <= 2.0).mean() * 100.0,
        "% Within 3 m": (d <= 3.0).mean() * 100.0,
    }


def compute_total_distance(df):
    pts = df[["x_local", "y_local"]].dropna().astype(float).to_numpy()
    if len(pts) < 2:
        return 0.0, [], []

    segment_lengths = []
    headings = []

    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        dist = math.hypot(dx, dy)
        segment_lengths.append(dist)

        if dist > 1e-9:
            headings.append(math.degrees(math.atan2(dy, dx)))

    return sum(segment_lengths), segment_lengths, headings


def normalize_angle(delta):
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def trajectory_smoothness_analysis(df):
    total_distance, segment_lengths, headings = compute_total_distance(df)
    if not segment_lengths or len(headings) < 2:
        return None

    heading_changes = [
        abs(normalize_angle(headings[i] - headings[i - 1]))
        for i in range(1, len(headings))
    ]

    if not heading_changes:
        return None

    abrupt_turn_threshold_deg = 45.0
    abrupt_turns = sum(change > abrupt_turn_threshold_deg for change in heading_changes)
    smooth_turns = len(heading_changes) - abrupt_turns
    smoothness_pct = (smooth_turns / len(heading_changes)) * 100.0

    return {
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


def session_duration_analysis(df):
    if df.empty or "ts" not in df.columns or len(df) < 2:
        return None

    ts = pd.to_datetime(df["ts"], errors="coerce").dropna()
    if len(ts) < 2:
        return None

    intervals = ts.diff().dropna().dt.total_seconds()

    return {
        "Duration (s)": (ts.iloc[-1] - ts.iloc[0]).total_seconds(),
        "Avg Sampling Interval (s)": intervals.mean() if not intervals.empty else 0.0,
        "Max Sampling Interval (s)": intervals.max() if not intervals.empty else 0.0,
    }


def classify_path_deviation(avg_dev):
    if avg_dev < 1.5:
        return "Excellent"
    if avg_dev < 3.0:
        return "Good"
    if avg_dev < 5.0:
        return "Acceptable"
    return "Poor"


def classify_smoothness(smoothness_pct):
    if smoothness_pct >= 95:
        return "Excellent"
    if smoothness_pct >= 90:
        return "Good"
    if smoothness_pct >= 80:
        return "Acceptable"
    return "Poor"


def print_metric_block(title, metrics):
    print(f"\n=== {title} ===")
    if not metrics:
        print("No data available.")
        return

    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"{key}: {value:.0f}" if "%" in key else f"{key}: {value:.2f}")
        else:
            print(f"{key}: {value}")


def build_single_session_summary(session_id, df):
    path_metrics = path_deviation_analysis(df) or {}
    smooth_metrics = trajectory_smoothness_analysis(df) or {}
    duration_metrics = session_duration_analysis(df) or {}

    avg_dev = path_metrics.get("Avg Deviation (m)")
    smoothness = smooth_metrics.get("Smoothness (%)")

    return {
        "Session ID": session_id,
        "GPS Points": path_metrics.get("GPS Points"),
        "Avg Deviation (m)": avg_dev,
        "Max Deviation (m)": path_metrics.get("Max Deviation (m)"),
        "% Within 2 m": path_metrics.get("% Within 2 m"),
        "Total Distance (m)": smooth_metrics.get("Total Distance (m)"),
        "Smoothness (%)": smoothness,
        "Duration (s)": duration_metrics.get("Duration (s)"),
        "Path Result": classify_path_deviation(avg_dev) if avg_dev is not None else None,
        "Smoothness Result": classify_smoothness(smoothness) if smoothness is not None else None,
    }


def repeatability_analysis(session_ids):
    rows = []
    for sid in session_ids:
        _, df = load_session_points(sid)
        if not df.empty:
            rows.append(build_single_session_summary(sid, df))

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


def format_display_df(df):
    df_display = df.copy()
    for col in df_display.columns:
        if "%" in col:
            df_display[col] = pd.to_numeric(df_display[col], errors="coerce").round(0)
    return df_display


def print_repeatability_results(df_runs, repeatability):
    print("\n==============================")
    print("REPEATABILITY TESTING RESULTS")
    print("==============================")

    if df_runs is not None and not df_runs.empty:
        print("\nPer-run summary:")
        print(format_display_df(df_runs).to_string(index=False))

    print_metric_block("Repeatability Variation Summary", repeatability)


def print_single_session_results(session_id, df):
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-id", type=int, default=None, help="Evaluate one specific session")
    parser.add_argument("--repeatability", action="store_true", help="Evaluate repeatability across all sessions")
    args = parser.parse_args()

    if args.repeatability:
        df_sessions = load_all_sessions()
        if df_sessions.empty:
            print("No sessions found.")
            return

        session_ids = df_sessions["id"].astype(int).tolist()
        df_runs, repeatability = repeatability_analysis(session_ids)
        print_repeatability_results(df_runs, repeatability)
        return

    session_id, df = load_session_points(args.session_id)
    print_single_session_results(session_id, df)


if __name__ == "__main__":
    main()