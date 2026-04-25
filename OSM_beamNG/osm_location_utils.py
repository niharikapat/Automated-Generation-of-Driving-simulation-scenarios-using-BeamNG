import osmnx as ox
from pyproj import Transformer
from shapely.geometry import LineString

LOCATION = "Hochschule Neu-Ulm, Neu-Ulm, Bavaria, Germany"
DIST_METERS = 400
NETWORK_TYPE = "drive"

#Download OSM graph around the configured location and project it to a metric CRS
def load_projected_graph(location=LOCATION, dist=DIST_METERS, network_type=NETWORK_TYPE):
    center = ox.geocode(location)
    G = ox.graph_from_point(center, dist=dist, network_type=network_type, simplify=False)
    G = ox.project_graph(G)
    return G, center

#Returns transformer to convert projected graph coordinates -> lon/lat (WGS84)
def get_wgs84_transformer(G):
    proj_crs = G.graph.get("crs")
    return Transformer.from_crs(proj_crs, "EPSG:4326", always_xy=True)

#Compute average graph node position to use as BeamNG local origin
def get_local_origin(G):
    xs = [float(G.nodes[n]["x"]) for n in G.nodes]
    ys = [float(G.nodes[n]["y"]) for n in G.nodes]
    ox0 = sum(xs) / len(xs)
    oy0 = sum(ys) / len(ys)
    return ox0, oy0

#Convert projected OSM coordinates to BeamNG local coordinates.
def to_local(x, y, ox0, oy0, z=0.0): 
    return float(x - ox0), float(y - oy0), float(z)

#Convert BeamNG local coordinates back to projected OSM coordinates
def to_projected(x_local, y_local, ox0, oy0):
    return float(x_local + ox0), float(y_local + oy0)

#Return all graph nodes as lat/lon lists for plotting
def get_osm_nodes_latlon(G, to_wgs84=None):
    if to_wgs84 is None:
        to_wgs84 = get_wgs84_transformer(G)

    lats, lons = [], []
    for n in G.nodes:
        x = float(G.nodes[n]["x"])
        y = float(G.nodes[n]["y"])
        lon, lat = to_wgs84.transform(x, y)
        lats.append(lat)
        lons.append(lon)

    return lats, lons

#Return all OSM road edges as line coordinate pairs for plotting
def get_osm_edge_latlon_lines(G, to_wgs84=None):
    if to_wgs84 is None:
        to_wgs84 = get_wgs84_transformer(G)

    lines = []

    for u, v, k, data in G.edges(keys=True, data=True):
        geom = data.get("geometry")

        if geom is None:
            geom = LineString(
                [
                    (float(G.nodes[u]["x"]), float(G.nodes[u]["y"])),
                    (float(G.nodes[v]["x"]), float(G.nodes[v]["y"])),
                ]
            )

        lats = []
        lons = []

        for x, y in geom.coords:
            lon, lat = to_wgs84.transform(float(x), float(y))
            lats.append(lat)
            lons.append(lon)

        if len(lats) >= 2:
            lines.append((lats, lons))

    return lines