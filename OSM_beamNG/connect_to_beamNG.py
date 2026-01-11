from beamngpy import BeamNGpy, Scenario, Vehicle
import time

beamng = BeamNGpy('localhost',25252,home=r"C:\BeamNG.tech.v0.37.6.0\BeamNG.tech.v0.37.6.0")

beamng.open(launch=True)

scenario = Scenario('smallgrid', 'test')

print("Creating scenario...")

ego = Vehicle('ego_vehicle', model='scintilla', partConfig = 'vehicles/scintilla/gtx.pc', licence ='HNU')

scenario.add_vehicle(ego, pos =(0,0,0), rot_quat = (0,0,0,1))

scenario.make(beamng)

print("Scenario created.")

beamng.scenario.load(scenario)
beamng.scenario.start()

print("scenario started. ")

time.sleep(10)
