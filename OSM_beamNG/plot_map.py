import os
import pandas as pd
from gmplot import gmplot
from db_folder.db import get_conn
from osm_location_utils import (
    load_projected_graph,
    get_wgs84_transformer,
    get_osm_edge_latlon_lines,
    get_osm_nodes_latlon,
)

GOOGLE_API_KEY = "AIzaSyC65iRx1Jkybu7oW5DY0grAjuVmL-tq7Qk"

def load_latest_session_points():
    conn = get_conn()

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
        SELECT lat, lon, gps_lat, gps_lon
        FROM gps_points WHERE session_id = %s ORDER BY ts;
        """,
        conn,
        params=(session_id,),
    )

    conn.close()
    return session_id, df_points

def main():
    session_id, df_points = load_latest_session_points()
    if df_points.empty:
        print("No GPS data found.")
        return

    G, center = load_projected_graph()
    to_wgs84 = get_wgs84_transformer(G)

    road_lines = get_osm_edge_latlon_lines(G, to_wgs84)
    graph_node_lats, graph_node_lons = get_osm_nodes_latlon(G, to_wgs84)

    lat_center = float(df_points.iloc[0]["lat"])
    lon_center = float(df_points.iloc[0]["lon"])

    gmap = gmplot.GoogleMapPlotter(lat_center, lon_center, 17, apikey=GOOGLE_API_KEY)

    # --- Gray road edge lines ---
    for road_lats, road_lons in road_lines:
        gmap.plot(road_lats, road_lons, color="#808080", edge_width=15)

    # --- Dark blue graph nodes ---
    gmap.scatter(graph_node_lats, graph_node_lons, color="black", size=3, marker=False)

    # --- IMU data ---
    imu_df = df_points.dropna(subset=["lat", "lon"])
    imu_lats = imu_df["lat"].astype(float).tolist()
    imu_lons = imu_df["lon"].astype(float).tolist()

    # --- GPS data ---
    gps_df = df_points.dropna(subset=["gps_lat", "gps_lon"])
    gps_lats = gps_df["gps_lat"].astype(float).tolist()
    gps_lons = gps_df["gps_lon"].astype(float).tolist()

    if imu_lats and imu_lons:   # --- Black IMU trace line ---
        gmap.plot(imu_lats, imu_lons, color="#000000", edge_width=5)
    
    if gps_lats and gps_lons:   # --- Red GPS trace line ---
        gmap.plot(gps_lats, gps_lons, color="#FF0000", edge_width=5) 
    
    MARK_EVERY_N = 1 # --- Markers every 3 points ---

    # Red GPS markers
    for i in range(0, len(gps_lats), MARK_EVERY_N):
        gmap.marker(gps_lats[i], gps_lons[i], color="red", title=f"GPS point {i}")

    # Black IMU markers
    for i in range(0, len(imu_lats), MARK_EVERY_N):
        gmap.marker(imu_lats[i], imu_lons[i], color="black", title=f"IMU point {i}")

    # --- Start / End markers ---
    if gps_lats and gps_lons:
        gmap.marker(gps_lats[0], gps_lons[0], color="red", title=f"GPS Start (session {session_id})")
        gmap.marker(gps_lats[-1], gps_lons[-1], color="red", title=f"GPS End (session {session_id})")

    if imu_lats and imu_lons:
        gmap.marker(imu_lats[0], imu_lons[0], color="black", title=f"IMU Start (session {session_id})")
        gmap.marker(imu_lats[-1], imu_lons[-1], color="black", title=f"IMU End (session {session_id})")

    os.makedirs("./output", exist_ok=True)
    out_path = "./output/beamng_map.html"
    gmap.draw(out_path)

    print(f"Map written to {out_path} (session {session_id})")

if __name__ == "__main__":
    main()