print("===============================================")
print("objaverse_load.py")
print("Loads one or more objects from objaverse and displays it in Klampt.")
print("Requires objaverse, fuzzysearch, and trimesh.")
print("===============================================")

from klampt.io.download_objaverse import object_list, object_annotations, object_classes, load
from klampt import WorldModel
from klampt import vis
from klampt.math import vectorops
import sys

INCHES_TO_METERS = 0.0254
MM_TO_METERS = 0.001
FLATTENED = True

# Testing annotations
# import json
# annotations = object_annotations()
# for name, metadata in annotations.items():
#     print(name,":")
#     print(json.dumps(metadata, indent=2))
#     input()

if len(sys.argv) < 2:
    print("Usage:",sys.argv[0],"[QUERY_NAME or list] [INDEX or 'random' or 'all']")
    sys.exit(0)
query = sys.argv[1]
index = 0
if len(sys.argv) > 2:
    if sys.argv[2] in ['random','all']:
        index = sys.argv[2]
    else:
        index = int(sys.argv[2])

if query == 'list':
    print("Available object classes:")
    for obj in object_classes():
        print(obj)
    sys.exit(0)

objs = load(query,index=index,flattened=False)
if not isinstance(objs,list):
    objs = [objs]
world = WorldModel()
x = 0
for nobjects,obj in enumerate(objs):
    world.makeRigidObject("obj"+str(nobjects)).geometry().set(obj)
    bb = obj.getBBTight()
    
    if nobjects > 0:
        x += (bb[1][0]-bb[0][0])*0.5
    
    #recenter the objects and shift to the right
    center = vectorops.mul(vectorops.add(bb[0],bb[1]),0.5)
    center[0] -= x
    for i in range(nobjects,world.numRigidObjects()):
        T = world.rigidObject(i).getTransform()
        T = T[0],vectorops.sub(T[1],center)
        world.rigidObject(i).setTransform(*T)
    
    #shift the next object loaded
    x += (bb[1][0]-bb[0][0])*0.5 + 0.1

vis.debug(world)
