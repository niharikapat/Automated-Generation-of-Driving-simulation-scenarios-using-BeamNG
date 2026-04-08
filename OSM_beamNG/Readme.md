<h1>Automatic Scenario creation in BeamNG.Tech using OSM data</h1>

<h2>What it does</h2>

<p>This script downloads a drivable road network from <b>OpenStreetMap</b> around <b>Hochschule Neu-Ulm</b>, converts the road geometry into local coordinates, and creates a <b>BeamNG.tech</b> scenario from it. It also spawns a vehicle, reads its position using an IMU sensor, and converts the simulated position back into geographic coordinates. </p>

<h2>What it helps for</h2>
<ul>
	<li>Testing OSM-based road import into BeamNG.tech</li>
	<li>Map-based driving simulation</li>
	<li>Localization and GPS position checks</li>
	<li>Prototyping route planning and road-network experiments</li>
</ul>


<h2>Requirements</h2>
<ul>
	
</ul>
<li>Python 3.9+</li>
<li>BeamNG.tech installed locally</li>
<li>Python packages:</li>
<ul>
	<li>osmnx</li>
	<li>networkx</li>
	<li>shapely</li>
	<li>pyproj</li>
	<li>beamngpy</li>
</ul>

<ol>
	<h2><li>Install dependencies</li></h2>
	
<h3>Windows</h3> 
<p>Create and activate a virtual environment first</p>

```bash
python -m venv .venv
.venv\Scripts\activate
```
<h3>Install dependencies</h3>

```bash
pip install -r requirements.txt
```

<h2><li>Set Up the Database</li></h2>

<h3>Create the database</h3>

<p>Log into MySQL and run using the command</p>

```bash
docker exec -it beamng-mysql mysql -u beamng -p beamng_gps
```
</ol>
<b>Note</b>

<p>Update the BeamNG installation path in the script before running it:</p>

`home=r"C:\\BeamNG.tech.v0.37.6.0\\BeamNG.tech.v0.37.6.0"`

