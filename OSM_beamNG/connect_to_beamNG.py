from math import sqrt
import osmnx as ox
import networkx as nx
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon
from beamngpy import BeamNGpy, Scenario, Vehicle, Road, ProceduralCube
from beamngpy.sensors import AdvancedIMU, GPS
from db_folder.db import get_conn, create_session, insert_gps_point
from osm_location_utils import (
    load_projected_graph,
    get_wgs84_transformer,
    get_local_origin,
    to_local,
    to_projected,
)


def estimate_road_width(data):
    highway = data.get("highway")

    if isinstance(highway, list):
        highway = highway[0]

    if highway in ["motorway", "trunk", "primary"]:
        return 10
    elif highway in ["secondary", "tertiary"]:
        return 8
    elif highway in ["residential", "unclassified"]:
        return 6
    elif highway in ["service", "living_street"]:
        return 4.5
    else:
        return 5

def get_spawn_point_from_osm(G, center, ox0, oy0):
    to_proj = Transformer.from_crs("EPSG:4326", G.graph["crs"], always_xy=True)

    center_lat, center_lon = center
    center_x, center_y = to_proj.transform(center_lon, center_lat)

    node_id = ox.distance.nearest_nodes(G, X=center_x, Y=center_y)

    x_proj = float(G.nodes[node_id]["x"])
    y_proj = float(G.nodes[node_id]["y"])

    x_local, y_local, z_local = to_local(x_proj, y_proj, ox0, oy0, 0.5)

    return x_local, y_local, z_local

def add_gebaeude_a(scenario, G, ox0, oy0):
    gebaeude_a_latlon = [
        (48.380369, 10.007933),
        (48.380143, 10.007984),
        (48.380393, 10.010166),
        (48.380619, 10.010105),
    ]

    #Convert WGS84 lat/lon into projected CRS of OSM graph
    to_proj = Transformer.from_crs("EPSG:4326", G.graph["crs"], always_xy=True)

    local_pts = []
    for lat, lon in gebaeude_a_latlon:
        x_proj, y_proj = to_proj.transform(lon, lat)
        x_local, y_local, z_local = to_local(x_proj, y_proj, ox0, oy0, 0.0)
        local_pts.append((x_local, y_local, z_local))

    #Compute center point of the 4 corners
    center_x = sum(p[0] for p in local_pts) / 4
    center_y = sum(p[1] for p in local_pts) / 4

    #Estimate building dimensions from corner distances
    def dist2d(p1, p2):
        return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    width = dist2d(local_pts[0], local_pts[1])
    length = dist2d(local_pts[1], local_pts[2])

    height = 12.0 

    scenario.add_procedural_mesh(
        ProceduralCube(
            name="gebaeude_a",
            pos=(center_x, center_y, height / 2.0),
            size=(width, length, height),
            rot_quat=(0, 0, 0, 1),
        )
    )


def main():
    G, center = load_projected_graph()
    ox0, oy0 = get_local_origin(G, center)    
    to_wgs84 = get_wgs84_transformer(G)

    #Real lon/lat of your BeamNG map origin
    ref_lon, ref_lat = to_wgs84.transform(ox0, oy0)

    scenario = Scenario("tech_ground", "my_scenario_1")

    rid = 0
    for u, v, k, data in G.edges(keys=True, data=True):
        geom = data.get("geometry")
        if geom is None:
            geom = LineString(
                [
                    (float(G.nodes[u]["x"]), float(G.nodes[u]["y"])),
                    (float(G.nodes[v]["x"]), float(G.nodes[v]["y"])),
                ]
            )

        road_width = estimate_road_width(data)

        pts = [(*to_local(x, y, ox0, oy0, 0.0), road_width) for x, y in geom.coords]
        if len(pts) < 2:
            continue

        road = Road("road_asphalt_2lane", rid=f"osm_{rid}", default_width=road_width)
        road.add_nodes(*pts)
        scenario.add_road(road)
        rid += 1

    add_gebaeude_a(scenario, G, ox0, oy0)

    vehicle = Vehicle("ego", model="etk800", licence="HNU")
    spawn_pos = get_spawn_point_from_osm(G, center, ox0, oy0)


    scenario.add_vehicle(
    vehicle,
    pos=spawn_pos,
    rot_quat=(0, 0, 0, 1)
    )

    bng = BeamNGpy(
        "localhost",
        25252,
        home=r"C:\BeamNG.tech.v0.37.6.0\BeamNG.tech.v0.37.6.0",
    )
    bng.open(launch=True)

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()

    vehicle.ai.set_mode("random")
    vehicle.ai.set_speed(8.33)

    #Database setup
    conn = get_conn()
    session_id = create_session(conn, scenario.name, vehicle.vid)
    print("Created DB session:", session_id)

    imu = AdvancedIMU("imu1", bng, vehicle, gfx_update_time=0.01)

    gps = GPS(
        "gps1",
        bng,
        vehicle,
        gfx_update_time=0.01,
        physics_update_time=0.01,
        pos=(0, 0, 1.7),
        ref_lon=float(ref_lon),
        ref_lat=float(ref_lat),
        is_send_immediately=False,
        is_visualised=False,
    )

    print("Scenario running. Press Ctrl+C to stop.")

    try:
        while True:
            bng.step(30)

            # IMU reading
            imu_data = imu.poll()
            if not imu_data or "pos" not in imu_data[0]:
                continue

            x_local, y_local, z_local = imu_data[0]["pos"]

            #Convert IMU world/local meters to the projected meters into the original lon/lat
            x_proj, y_proj = to_projected(x_local, y_local, ox0, oy0)
            imu_lon, imu_lat = to_wgs84.transform(x_proj, y_proj)

            #GPS reading from vehicle trace
            gps_data = gps.poll()
            if not gps_data:
                continue

            latest_key = max(gps_data.keys())
            gps_reading = gps_data[latest_key]

            gps_lat = gps_reading.get("lat")
            gps_lon = gps_reading.get("lon")

            if gps_lat is None or gps_lon is None:
                print(f"Waiting for valid GPS reading... GPS=({gps_lat}, {gps_lon})")
                continue

            #Distance to nearest road segment
            u, v, key = ox.distance.nearest_edges(G, X=x_proj, Y=y_proj)
            edge_data = G.get_edge_data(u, v, key)
            edge_geom = edge_data.get("geometry")

            if edge_geom is None:
                edge_geom = LineString(
                    [
                        (float(G.nodes[u]["x"]), float(G.nodes[u]["y"])),
                        (float(G.nodes[v]["x"]), float(G.nodes[v]["y"])),
                    ]
                )

            dist_to_road_m = edge_geom.distance(Point(x_proj, y_proj))

            #Insert current session details into DB
            insert_gps_point(
                conn,
                session_id=session_id,
                lat=float(imu_lat),
                lon=float(imu_lon),
                gps_lat=float(gps_lat),
                gps_lon=float(gps_lon),
                x_local=float(x_local),
                y_local=float(y_local),
                z_local=float(z_local),
                dist_to_road_m=float(dist_to_road_m),
            )

            print(
                f"IMU lat/lon=({float(imu_lat):.7f}, {float(imu_lon):.7f}) | "
                f"GPS lat/lon=({float(gps_lat):.7f}, {float(gps_lon):.7f}) | "
                f"X/Y=({float(x_local):.2f},{float(y_local):.2f}) | "
                f"dist_to_road={float(dist_to_road_m):.2f} m"
            )

    except KeyboardInterrupt:
        pass
    finally:
        try:
            imu.remove()
        except Exception:
            pass

        try:
            gps.remove()
        except Exception:
            pass

        try:
            conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
