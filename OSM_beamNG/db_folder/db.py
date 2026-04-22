import os
from dotenv import load_dotenv
from datetime import datetime
import mysql.connector

load_dotenv()


def get_conn():
    return mysql.connector.connect(
        host=os.getenv("DB_HOST", "127.0.0.1"),
        port=int(os.getenv("DB_PORT", "3306")),
        user=os.getenv("DB_USER", "beamng"),
        password=os.getenv("DB_PASSWORD", "beamngpass"),
        database=os.getenv("DB_NAME", "beamng_gps"),
        autocommit=True,
    )


def create_session(conn, scenario_name: str, vehicle_id: str) -> int:
    cur = conn.cursor()
    cur.execute(
        "INSERT INTO sessions (scenario_name, vehicle_id) VALUES (%s, %s)",
        (scenario_name, vehicle_id),
    )
    session_id = cur.lastrowid
    cur.close()
    return int(session_id)


def insert_gps_point(
    conn,
    session_id,
    lat,
    lon,
    gps_lat=None,
    gps_lon=None,
    x_local=None,
    y_local=None,
    z_local=None,
    dist_to_road_m=None,
):
    """
    Store one row of position data.

    lat/lon     -> IMU-derived lat/lon
    gps_lat/lon -> GPS sensor lat/lon
    """
    cur = conn.cursor()
    cur.execute(
        """
        INSERT INTO gps_points
        (
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
        )
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """,
        (
            session_id,
            datetime.utcnow(),
            lat,
            lon,
            gps_lat,
            gps_lon,
            x_local,
            y_local,
            z_local,
            dist_to_road_m,
        ),
    )
    cur.close()