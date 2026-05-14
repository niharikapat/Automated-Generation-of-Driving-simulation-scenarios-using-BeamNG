import os
import pandas as pd
import folium
from db_folder.db import get_conn
from osm_location_utils import (
    load_projected_graph,
    get_wgs84_transformer,
    get_osm_edge_latlon_lines,
    get_osm_nodes_latlon,
)

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
        SELECT
            lat,
            lon,
            gps_lat,
            gps_lon
        FROM gps_points
        WHERE session_id = %s
        ORDER BY ts;
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

    m = folium.Map(
        location=[lat_center, lon_center],
        zoom_start=17,
        tiles="OpenStreetMap",
    )

    #Gray coloring for road edges
    for road_lats, road_lons in road_lines:
        coords = list(zip(road_lats, road_lons))
        folium.PolyLine(
            locations=coords,
            color="gray",
            weight=4,
            opacity=0.8,
        ).add_to(m)

    #Graph nodes marked in dark blue coloring
    for lat, lon in zip(graph_node_lats, graph_node_lons):
        folium.CircleMarker(
            location=[lat, lon],
            radius=3,
            color="darkblue",
            fill=True,
            fill_color="darkblue",
            fill_opacity=1.0,
        ).add_to(m)

    #Vehicle trace for IMU added in black
    imu_df = df_points.dropna(subset=["lat", "lon"])
    imu_lats = imu_df["lat"].astype(float).tolist()
    imu_lons = imu_df["lon"].astype(float).tolist()

    if imu_lats and imu_lons:
        imu_coords = list(zip(imu_lats, imu_lons))
        folium.PolyLine(
            locations=imu_coords,
            color="black",
            weight=5,
            opacity=1.0,
        ).add_to(m)

    #Vehcile trace for GPS added in red
    gps_df = df_points.dropna(subset=["gps_lat", "gps_lon"])
    gps_lats = gps_df["gps_lat"].astype(float).tolist()
    gps_lons = gps_df["gps_lon"].astype(float).tolist()

    if gps_lats and gps_lons:
        gps_coords = list(zip(gps_lats, gps_lons))
        folium.PolyLine(
            locations=gps_coords,
            color="red",
            weight=5,
            opacity=1.0,
        ).add_to(m)

    #Markers for every 3 points
    MARK_EVERY_N = 3

    for i in range(0, len(gps_lats), MARK_EVERY_N):
        folium.Marker(
            location=[gps_lats[i], gps_lons[i]],
            tooltip=f"GPS point {i}",
            icon=folium.Icon(color="red"),
        ).add_to(m)

    for i in range(0, len(imu_lats), MARK_EVERY_N):
        folium.Marker(
            location=[imu_lats[i], imu_lons[i]],
            tooltip=f"IMU point {i}",
            icon=folium.Icon(color="black"),
        ).add_to(m)

    #Start and end markers for the session trace
    if gps_lats and gps_lons:
        folium.Marker(
            location=[gps_lats[0], gps_lons[0]],
            tooltip=f"GPS Start (session {session_id})",
            icon=folium.Icon(color="red", icon="play"),
        ).add_to(m)

        folium.Marker(
            location=[gps_lats[-1], gps_lons[-1]],
            tooltip=f"GPS End (session {session_id})",
            icon=folium.Icon(color="red", icon="stop"),
        ).add_to(m)

    if imu_lats and imu_lons:
        folium.Marker(
            location=[imu_lats[0], imu_lons[0]],
            tooltip=f"IMU Start (session {session_id})",
            icon=folium.Icon(color="black", icon="play"),
        ).add_to(m)

        folium.Marker(
            location=[imu_lats[-1], imu_lons[-1]],
            tooltip=f"IMU End (session {session_id})",
            icon=folium.Icon(color="black", icon="stop"),
        ).add_to(m)

    os.makedirs("./output", exist_ok=True)
    out_path = f"./output/beamng_map_osm_session_{session_id}.html"
    m.save(out_path)

    print(f"OSM map written to {out_path} (session {session_id})")


if __name__ == "__main__":
    main()
