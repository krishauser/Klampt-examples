from pytest import fixture
from klampt import WorldModel, Simulator, set_friction_cone_approximation_edges
from klampt.math import se3,so3,vectorops
from klampt.model.contact import ContactPoint, sim_contact_map, contact_map_holds, force_closure, com_equilibrium, support_polygon, equilibrium_torques
from klampt import vis
import time

#these form a square + a downward facing point
@fixture
def fc_contacts():
	return [ContactPoint([-1,-1,0],[0,0,1],0.5),
					ContactPoint([1,-1,0],[0,0,1],0.5),
					ContactPoint([1,1,0],[0,0,1],0.5),
					ContactPoint([-1,1,0],[0,0,1],0.5),
					ContactPoint([0,0,1],[0,0,-1],0.5)]

#these form a square
@fixture
def stable_contacts():
	return [ContactPoint([-1,-1,0],[0,0,1],0.5),
					ContactPoint([1,-1,0],[0,0,1],0.5),
					ContactPoint([1,1,0],[0,0,1],0.5),
					ContactPoint([-1,1,0],[0,0,1],0.5)]

#these form two points pointing in strange directions
@fixture
def unstable_contacts():
	return [ContactPoint([-1,-1,0],[0,1,0],0.5),
						ContactPoint([1,1,0],[1,0,0],0.5)]


def test_force_closure(fc_contacts, stable_contacts, unstable_contacts):
	set_friction_cone_approximation_edges(4)
	assert force_closure(fc_contacts) == True,"force closure not detected in force closure contacts"
	assert force_closure(stable_contacts) == False,"force closure detected in stable contacts"
	assert force_closure(unstable_contacts) == False,"force closure detected in unstable contacts"

def test_com_equilibrium(fc_contacts, stable_contacts, unstable_contacts):
	forces = com_equilibrium(stable_contacts,[0,0,-1],(0,0,0))
	assert forces is not None
	assert len(forces) == len(stable_contacts)
	assert vectorops.distance(vectorops.add(*forces),[0,0,1]) < 1e-6
	forces2 = com_equilibrium(stable_contacts,[0,0,-1],(0,0,10))
	assert forces2 == forces2
	assert com_equilibrium(stable_contacts,[0,0,-1],(2,0,10)) == None, "Infeasible com equilibrium wasn't detected"
	assert com_equilibrium(stable_contacts,[0,0,-1],None) == True, "Any stable configuration wasn't detected"
	assert com_equilibrium(unstable_contacts,[0,0,-1],None) == False, "Unstable configuration wasn't detected"

def test_support_polygon(fc_contacts, stable_contacts, unstable_contacts):
	sp = support_polygon(stable_contacts)
	#keep only unique points
	nunique = 0
	for i in range(len(sp)):
		j = (i+1)%len(sp)  
		if vectorops.distance(sp[i],sp[j]) > 1e-7:
			nunique += 1
	assert nunique == 4,"Stable contacts didn't give a square"
	assert len(support_polygon(fc_contacts)) == 0,"Force closure contacts should have a support polygon that is the whole plane"
	sp = support_polygon(unstable_contacts)
	assert len(sp) == 1 and sp[0][0] == sp[0][1] == 0, "Unstable contacts should have a support polygon that is invalid"

"""
h = Hold()
h.contacts = fc_contacts
vis.add("Hold",h)
vis.dialog()
vis.clear();
h.contacts = unstable_contacts
vis.add("Hold",h)
vis.dialog()
vis.clear();
"""

def test_sim_contacts():
	#load a simulation, getting the contacts / holds from it
	world = WorldModel()
	world.readFile("../../../data/athlete_plane.xml")
	sim = Simulator(world)
	sim.enableContactFeedbackAll()

	for i in range(1):
		sim.simulate(0.0333)
		sim.updateWorld()

		cm = sim_contact_map(sim)
		holds = contact_map_holds(cm)
		print("Num contacts",sum(len(h.contacts) for h in holds))
		res = equilibrium_torques(world.robot(0),holds)
		if res is None:
			print("No equilibrium torques/contact forces")
		else:
			tinf,f = res
			print("TORQUE",tinf)
			print("FORCES",f)
			#res2 = equilibrium_torques(world.robot(0),holds,norm=1)
			res2 = equilibrium_torques(world.robot(0),holds)
			assert res2 is not None,"L1 equilibrium torques/contact forces failed"
			t1, f = res2
			print("Equilibrium forces:")
			for fi in f:
				print("   ",fi)
			print("Residual force on CM",vectorops.add(*f))
			print("Torques, L1 / Linf / simulation")
			ts = sim.getActualTorque(0)
			for i,(te1,teinf,tsi) in enumerate(zip(t1[6:],tinf[6:],ts)):
				print("   %s:\t%.3f / %.3f / %.3f"%(world.robot(0).link(6+i).getName(),te1,teinf,tsi))
			print("L1 norm solved",vectorops.norm_L1(t1),"sim",vectorops.norm_L1(ts))
			print("Linf norm solved",vectorops.norm_Linf(tinf),"sim",vectorops.norm_Linf(ts))
		print("Simulation time",sim.getTime())
		

def vis_sim_contacts():
	#load and show a simulation, getting the contacts / holds from it
	world = WorldModel()
	world.readFile("../../../data/athlete_plane.xml")
	sim = Simulator(world)
	sim.enableContactFeedbackAll()

	vis.add("world",world)
	vis.show()
	while vis.shown():
		vis.lock()
		sim.simulate(0.0333)
		sim.updateWorld()

		cm = sim_contact_map(sim)
		holds = contact_map_holds(cm)
		print("Num contacts",sum(len(h.contacts) for h in holds))
		res = equilibrium_torques(world.robot(0),holds)
		if res is None:
			print("No equilibrium torques/contact forces")
		else:
			tinf,f = res
			res2 = equilibrium_torques(world.robot(0),holds,norm=1)
			if res2 is not None:
				t1, f = res2
				print("Equilibrium forces:")
				for fi in f:
					print("   ",fi)
				print("Residual force on CM",vectorops.add(*f))
				print("Torques, L1 / Linf / simulation")
				ts = sim.getActualTorque(0)
				for i,(te1,teinf,tsi) in enumerate(zip(t1[6:],tinf[6:],ts)):
					print("   %s:\t%.3f / %.3f / %.3f"%(world.robot(0).link(6+i).getName(),te1,teinf,tsi))
				print("L1 norm solved",vectorops.norm_L1(t1),"sim",vectorops.norm_L1(ts))
				print("Linf norm solved",vectorops.norm_Linf(tinf),"sim",vectorops.norm_Linf(ts))
			else:
				print("L1 solve couldn't find forces???")
				input()
		print("Simulation time",sim.getTime())
		vis.unlock()
		
		for i,h in enumerate(holds):
			vis.add("hold "+str(i),h)
		time.sleep(0.01)
	vis.kill()
		
if __name__ == '__main__':
	vis_sim_contacts()
