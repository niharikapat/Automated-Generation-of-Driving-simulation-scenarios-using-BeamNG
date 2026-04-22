USE beamng_gps;

CREATE TABLE IF NOT EXISTS sessions (
  id BIGINT AUTO_INCREMENT PRIMARY KEY,
  scenario_name VARCHAR(128) NOT NULL,
  vehicle_id VARCHAR(64) NOT NULL,
  started_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS gps_points (
  id BIGINT AUTO_INCREMENT PRIMARY KEY,
  session_id BIGINT NOT NULL,
  ts DATETIME NOT NULL,
  lat DOUBLE NOT NULL,
  lon DOUBLE NOT NULL,
  x_local DOUBLE NULL,
  y_local DOUBLE NULL,
  z_local DOUBLE NULL,
  dist_to_road_m DOUBLE NULL,
  gps_lat DOUBLE NULL,
  gps_lon DOUBLE NULL,
  INDEX(session_id),
  INDEX(ts),
  CONSTRAINT fk_session
    FOREIGN KEY (session_id) REFERENCES sessions(id)
    ON DELETE CASCADE
);