import osmnx as ox
import networkx as nx
from shapely.geometry import LineString, Point
from pyproj import Transformer
from beamngpy import BeamNGpy, Scenario, Vehicle, Road
from beamngpy.sensors import AdvancedIMU


def main():
    # ---- OSM download + projection (meters) ----
    location = "Hochschule Neu-Ulm, Neu-Ulm, Bavaria, Germany"
    center = ox.geocode(location)  # (lat, lon)

    G = ox.graph_from_point(center, dist=200, network_type="drive", simplify=True)
    G = ox.project_graph(G)  # x/y now in meters (projected CRS)
    #ox.plot_graph(G)
    print(G)
    

    # ---- Choose a local origin so coordinates are near (0,0) ----
    xs = [float(G.nodes[n]["x"]) for n in G.nodes]
    ys = [float(G.nodes[n]["y"]) for n in G.nodes]
    ox0, oy0 = sum(xs) / len(xs), sum(ys) / len(ys)

    def to_local(x, y, z=0.0):
        return (float(x - ox0), float(y - oy0), float(z))

    def to_projected(x_local, y_local):
        """Convert BeamNG local coords back to OSM projected meters."""
        return float(x_local + ox0), float(y_local + oy0)

    # Transformer: projected CRS -> WGS84 lat/lon
    # Note: Transformer expects (x, y) and returns (lon, lat) with always_xy=True
    proj_crs = G.graph.get("crs")
    to_wgs84 = Transformer.from_crs(proj_crs, "EPSG:4326", always_xy=True)

    # ---- Build BeamNG scenario ----
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

        pts = [to_local(x, y, 0.0) for x, y in geom.coords]
        if len(pts) < 2:
            continue

        pts = [(float(x), float(y), float(z)) for (x, y, z) in pts]

        road = Road("road_asphalt_2lane", rid=f"osm_{rid}")
        road.add_nodes(*pts)
        scenario.add_road(road)
        rid += 1

    # ---- Vehicle ----
    vehicle = Vehicle("ego", model="etk800", licence="HNU")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0.2), rot_quat=(0, 0, 0, 1))

    # ---- BeamNG connection ----
    bng = BeamNGpy(
        "localhost",
        25252,
        home=r"C:\BeamNG.tech.v0.37.6.0\BeamNG.tech.v0.37.6.0",
    )
    bng.open(launch=True)

    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()

    imu = AdvancedIMU("accel1", bng, vehicle, gfx_update_time=0.01)

    nodes = list(G.nodes)
    if len(nodes) >= 2:
        start, goal = nodes[0], nodes[-1]
        _path_nodes = nx.shortest_path(G, start, goal, weight="length")
        vehicle.ai.set_mode("span") 

    print("Scenario running. Close BeamNG or Ctrl+C to stop.")

    i = 0
    try:
        while True:
            bng.step(60)
            i += 1

            #Print every 10 steps to avoid flooding
            if i % 10 != 0:
                continue

            data = imu.poll()
            if not data or "pos" not in data[0]:
                continue

            # BeamNG world/local position 
            x_local, y_local, z_local = data[0]["pos"]

            # Convert back to projected meters in the OSM graph CRS
            x_proj, y_proj = to_projected(x_local, y_local)

            # Convert projected -> lon/lat
            lon, lat = to_wgs84.transform(x_proj, y_proj)

            # Find nearest OSM road edge and compute distance to it (meters)
            u, v, key = ox.distance.nearest_edges(G, X=x_proj, Y=y_proj)
            edge_data = G.get_edge_data(u, v, key)
            edge_geom = edge_data.get("geometry")
            if edge_geom is None:
                edge_geom = LineString(
                    [(G.nodes[u]["x"], G.nodes[u]["y"]), (G.nodes[v]["x"], G.nodes[v]["y"])]
                )

            dist_to_road_m = edge_geom.distance(Point(x_proj, y_proj))

            print(
                f"local(xyz)=({x_local:.2f},{y_local:.2f},{z_local:.2f}) | "
                f"lat/lon=({lat:.7f},{lon:.7f}) | "
                #f"nearest-road-node={dist_to_road_m:.2f} m"
            )

    except KeyboardInterrupt:
        pass
    finally:
        try:
            imu.remove()
        except Exception:
            pass


if __name__ == "__main__":
    main()
