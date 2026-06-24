from math import sqrt
import osmnx as ox
import networkx as nx
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon
from beamngpy import BeamNGpy, Scenario, Vehicle, Road, ProceduralCube
from beamngpy.sensors import AdvancedIMU, GPS, Electrics

LOCATION = "Hochschule Neu-Ulm, Neu-Ulm, Bavaria, Germany"
DIST_METERS = 500
NETWORK_TYPE = "drive"


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

def get_local_origin(G, center=None):
    # If center is provided, use real OSM center as BeamNG (0,0)
    if center is not None:
        to_proj = Transformer.from_crs("EPSG:4326", G.graph["crs"], always_xy=True)

        center_lat, center_lon = center
        ox0, oy0 = to_proj.transform(center_lon, center_lat)

        return float(ox0), float(oy0)

    # Fallback: use average graph node position as the origin
    xs = [float(G.nodes[n]["x"]) for n in G.nodes]
    ys = [float(G.nodes[n]["y"]) for n in G.nodes]

    return sum(xs) / len(xs), sum(ys) / len(ys)

def to_local(x, y, ox0, oy0, z=0.0):
    return float(x - ox0), float(y - oy0), float(z)

def to_projected(x_local, y_local, ox0, oy0):
    return float(x_local + ox0), float(y_local + oy0)

def get_spawn_point_from_osm(G, center, ox0, oy0):
    to_proj = Transformer.from_crs("EPSG:4326", G.graph["crs"], always_xy=True)

    center_lat, center_lon = center
    center_x, center_y = to_proj.transform(center_lon, center_lat)

    node_id = ox.distance.nearest_nodes(G, X=center_x, Y=center_y)

    x_proj = float(G.nodes[node_id]["x"])
    y_proj = float(G.nodes[node_id]["y"])

    x_local, y_local, z_local = to_local(x_proj, y_proj, ox0, oy0, 0.5)

    return x_local, y_local, z_local

def main():

    center = ox.geocode(LOCATION)  
    G = ox.graph_from_point(center, dist=DIST_METERS, network_type=NETWORK_TYPE, simplify=True) 
    G = ox.project_graph(G)
    
    ox0, oy0 = get_local_origin(G, center) 

    proj_crs = G.graph.get("crs")
    to_wgs84 = Transformer.from_crs(proj_crs, "EPSG:4326", always_xy=True)  

    

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


    vehicle = Vehicle("ego", model="etk800", licence="HNU")
    vehicle.sensors.attach("electrics", Electrics())
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

            vehicle.sensors.poll("electrics")
            electrics_data = vehicle.sensors["electrics"]

            speed_mps = abs(float(electrics_data["wheelspeed"]))
            speed_kmh = speed_mps * 3.6

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

            print(
                f"IMU lat/lon=({float(imu_lat):.7f}, {float(imu_lon):.7f}) | "
                f"GPS lat/lon=({float(gps_lat):.7f}, {float(gps_lon):.7f}) | "
                f"X/Y=({float(x_local):.2f},{float(y_local):.2f}) | "
                f"speed={float(speed_kmh):.2f} km/h | "
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


if __name__ == "__main__":
    main()