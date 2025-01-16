from klampt import WorldModel,RobotModel,Geometry3D
from klampt.model import workspace
from klampt import vis 
from klampt.model import ik
from klampt.io import resource
from klampt.vis import editors 
import numpy as np
import time
import sys

fn = "../../data/tx90scenario0.xml"
if len(sys.argv) > 1:
    fn = sys.argv[1]

world = WorldModel()
if not world.loadFile(fn):
    print("Unable to load world file",fn)
    exit(1)
robot = world.robot(0)

q = resource.get('workspace_zero.config','Config',world=world)
robot.setConfig(q)

ed = editors.SelectionEditor('active_links',list(range(robot.numLinks())),'Select the active links for workspace calculation', world, robot)
res,links = editors.run(ed)
if len(links)==0:
    print("Need to select a link")
    exit(1)
links = sorted(links)

link = robot.link(links[-1])
print("Treating link {} ({}) as end effector".format(links[-1],link.getName()))
obj = ik.fixed_objective(link)
fixed_indices = []
for i in range(robot.numLinks()):
    if i not in links:
        fixed_indices.append(i)

#extract all objects and terrains as obstacles
obstacles = []
for i in range(world.numRigidObjects()):
    obstacles.append(world.rigidObject(i))
for i in range(world.numTerrains()):
    obstacles.append(world.terrain(i))

"""
pts = np.random.random((10000,3))
vals = np.random.random(10000)
t0 = time.time()
grid = workspace.compute_field_grid(pts,vals,aggregator='max')
t1 = time.time()
print("Computing field took time",t1-t0)
mesh = Geometry3D(grid).convert('TriangleMesh',0.5)
t2 = time.time()
print("Marching cubes took time",t2-t1)
vis.add("temp",mesh,(0,0,1,0.5))
vis.loop()
vis.remove("temp")
"""

#can play around with these parameters
payload_kg = 1     #if you want to support a payload, set this to
Nsamples = 20000   #the more samples you use, the more accurate the workspace will be.
resolution = 0.1   #the coarser the resolution, the fewer samples you need, but the less accurate it will be

print()
print()
print("Computing workspace that can support {} kg...".format(payload_kg))
res = workspace.compute_workspace(link,obj,Nsamples=Nsamples,resolution=resolution,
    fixed_links=fixed_indices,obstacles=obstacles,
    load=(0,0,-payload_kg*9.801),load_type='force')
print(res.keys())
workspace = res['workspace']
print("Reached:",np.sum(workspace.getValues()),"/",np.prod(workspace.getValues().shape))
for k in res.keys():
    if k == 'workspace': continue
    workspace2 = res[k]
    print("Reached:",np.sum(workspace2.getValues()),"/",np.prod(workspace.getValues().shape),"constraint",k)
print("Bound",[v for v in workspace.bbox[:3]],[v for v in workspace.bbox[3:]])
print()
print()
vis.add("world",world)
geom = Geometry3D(workspace).convert("TriangleMesh",0.5)
vis.add("reachable_all",geom,color=(1,0,0,0.5))

extras = []
workspace2 = res['self_collision_free']
geom2 = Geometry3D(workspace2).convert("TriangleMesh",0.5)
vis.add("reachable_self_collision_free",geom2,color=(1,1,0,0.5))
extras.append("reachable_self_collision_free")
if len(obstacles) > 0:
    workspace3 = res['obstacle_0_collision_free']
    geom3 = Geometry3D(workspace2).convert("TriangleMesh",0.5)
    vis.add("reachable_obstacle_0_collision_free",geom2,color=(1,1,0,0.5))
    extras.append("reachable_obstacle_0_collision_free")

extras_shown = True
def toggle_extras():
    global extras_shown
    extras_shown = not extras_shown
    for k in extras:
        vis.hide(k,not extras_shown)

def save_workspace():
    fn = "workspace.vol"
    geom.saveFile(fn)
    print("Saved workspace to",fn)

vis.addAction(toggle_extras, "Toggle details", 'd', 'Turn detailed view on/off')
vis.addAction(save_workspace, "Save to workspace.vol", 's', 'Saves as a VolumeGrid format')
vis.add("world",world)

vis.loop()
vis.kill()