<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>BeamNG + OSM Project Setup Guide</title>
</head>
<body>

  <h1>BeamNG + OSM Project Setup Guide</h1>

  <p>
    This project uses <strong>BeamNG.tech</strong>, <strong>Python</strong>, and
    <strong>MySQL via Docker</strong> to run a driving simulation scenario and store GPS-related data.
  </p>

  <hr />

  <h2>Prerequisites</h2>

  <p>Make sure the following are available before starting:</p>

  <ul>
    <li>BeamNG.tech package</li>
    <li>Valid BeamNG.tech license</li>
    <li>Python</li>
    <li>pip</li>
    <li>Docker Desktop</li>
    <li>GitHub project ZIP</li>
  </ul>

  <hr />

  <h2>1. Install BeamNG.tech</h2>

  <ol>
    <li>Download the <strong>BeamNG.tech</strong> package.</li>
    <li>Extract it to your preferred location.</li>
    <li>Install and activate the <strong>license</strong> by following the instructions provided with BeamNG.tech.</li>
  </ol>

  <hr />

  <h2>2. Download the Project Code</h2>

  <ol>
    <li>Download the ZIP package from GitHub.</li>
    <li>Extract the project contents into your desired folder.</li>
  </ol>

  <hr />

  <h2>3. Update the BeamNG Path</h2>

  <p>
    Open the file <code>connect_to_beamng.py</code> and change the BeamNG path based on where
    <code>BeamNG.tech</code> is installed on your system.
  </p>

  <p>Example:</p>

  <pre><code>BNG_HOME = "C:/Path/To/BeamNG.tech"</code></pre>

  <p>Make sure this path points to your local BeamNG.tech installation folder.</p>

  <hr />

  <h2>4. Install Python</h2>

  <ol>
    <li>Download and install <strong>Python</strong> on your system.</li>
    <li>During installation, make sure to enable the option to add Python to the system PATH.</li>
    <li>After installation, open Command Prompt or PowerShell and verify that Python is accessible.</li>
  </ol>

  <p>Run:</p>

  <pre><code>python --version
pip --version</code></pre>

  <p>If both commands return version numbers, Python and pip are installed correctly.</p>

  <hr />

  <h2>5. Install Project Requirements</h2>

  <ol>
    <li>Open Command Prompt or PowerShell.</li>
    <li>Navigate to the extracted project folder.</li>
  </ol>

  <p>Example:</p>

  <pre><code>cd path\to\your\project\folder</code></pre>

  <ol start="3">
    <li>Install all required Python packages using:</li>
  </ol>

  <pre><code>pip install -r requirements.txt</code></pre>

  <p>This command installs all dependencies required to run the project.</p>

  <hr />

  <h2>6. Set Up SQL Using Docker</h2>

  <ol>
    <li>Make sure Docker Desktop is installed and running.</li>
    <li>Open Command Prompt or PowerShell in the project folder.</li>
    <li>Start the MySQL container using:</li>
  </ol>

  <pre><code>docker compose up -d</code></pre>

  <p>This will start the SQL database container in detached mode.</p>

  <hr />

  <h2>7. Log In to the SQL Database</h2>

  <p>
    After the container is running, use the following command to log in to the MySQL database inside Docker:
  </p>

  <pre><code>docker exec -it beamng-mysql mysql -ubeamng -pbeamngpass beamng_gps</code></pre>

  <p>This connects you to the <code>beamng_gps</code> database.</p>

  <hr />

  <h2>8. Check the Tables</h2>

  <p>
    Once you are inside the MySQL shell, run the following command to check the available tables:
  </p>

  <pre><code>SHOW TABLES;</code></pre>

  <p>This helps verify that the database has been created correctly.</p>

  <hr />

  <h2>9. Exit MySQL</h2>

  <p>When you are done checking the database, exit the MySQL shell using:</p>

  <pre><code>exit</code></pre>

  <hr />

  <h2>Summary of Commands</h2>

  <p>Use the following commands during setup:</p>

  <pre><code>python --version
pip --version
cd path\to\your\project\folder
pip install -r requirements.txt
docker compose up -d
docker exec -it beamng-mysql mysql -ubeamng -pbeamngpass beamng_gps</code></pre>

  <p>Inside MySQL:</p>

  <pre><code>SHOW TABLES;
exit</code></pre>

  <hr />

  <h2>Notes</h2>

  <ul>
    <li>BeamNG.tech must be installed and licensed before running the simulation.</li>
    <li>The BeamNG path in <code>connect_to_beamng.py</code> must be updated correctly.</li>
    <li>Python and pip must be accessible from the command line.</li>
    <li>Docker Desktop must be running before starting the SQL container.</li>
  </ul>

</body>
</html>
