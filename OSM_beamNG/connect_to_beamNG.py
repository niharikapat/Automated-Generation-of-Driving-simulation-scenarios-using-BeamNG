import osmnx as ox
import networkx as nx
from shapely.geometry import LineString
from beamngpy import BeamNGpy, Scenario, Vehicle, Road
from beamngpy.sensors import AdvancedIMU



def main():
    # ---- OSM download + projection (meters) ----
    location = "Hochschule Neu-Ulm, Neu-Ulm, Bavaria, Germany"
    center = ox.geocode(location)  # (lat, lon)

    G = ox.graph_from_point(center, dist=1000, network_type="drive", simplify=True)
    G = ox.project_graph(G)  # x/y now in meters

    print(G)

    # ---- Choose a local origin so coordinates are near (0,0) ----
    xs = [float(G.nodes[n]["x"]) for n in G.nodes]
    ys = [float(G.nodes[n]["y"]) for n in G.nodes]
    ox0, oy0 = sum(xs) / len(xs), sum(ys) / len(ys)

    def to_local(x, y, z=0.0):
        return (float(x - ox0), float(y - oy0), float(z))

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

        # Build points in local BeamNG coords
        pts = [to_local(x, y, 0.0) for x, y in geom.coords]
        if len(pts) < 2:
            continue

        # Ensure pure python floats
        pts = [(float(x), float(y), float(z)) for (x, y, z) in pts]

        road = Road("road_asphalt_2lane", rid=f"osm_{rid}")

        # IMPORTANT: your BeamNGpy expects nodes as *args, not a list
        road.add_nodes(*pts)  # <-- this fixes the BNGValueError

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

    # Make/load/start scenario
    scenario.make(bng)
    bng.scenario.load(scenario)
    bng.scenario.start()
    imu = AdvancedIMU("accel1", bng, vehicle, gfx_update_time=0.01)
    # ---- Simple AI route (shortest path between two nodes) ----
    nodes = list(G.nodes)
    if len(nodes) >= 2:
        start, goal = nodes[0], nodes[-1]
        path_nodes = nx.shortest_path(G, start, goal, weight="length")
        route = [to_local(G.nodes[n]["x"], G.nodes[n]["y"], 0.0) for n in path_nodes]
        route = [(float(x), float(y), float(z)) for (x, y, z) in route]

        vehicle.ai.set_mode("span")
        # ---- vehicle.ai.set_route(route) ----

    print("Scenario running. Close BeamNG or Ctrl+C to stop.")
    try:
        while True:
            bng.step(60)
            data = imu.poll()
            if data and "pos" in data[0]:
                x, y, z = data[0]["pos"]
                print(f"pos: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
