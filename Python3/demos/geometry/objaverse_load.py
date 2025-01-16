print("===============================================")
print("objaverse_load.py")
print("Loads an object from objaverse and displays it in Klampt.")
print("Requires objaverse, fuzzysearch, and trimesh.")
print("===============================================")

import objaverse
import fuzzysearch
import trimesh
from klampt.io.trimesh_convert import from_trimesh
from klampt.math import so3,se3,vectorops
from klampt.model.collide import bb_create,bb_union
from klampt import WorldModel,Geometry3D
from klampt import vis
import math
import sys

INCHES_TO_METERS = 0.0254
MM_TO_METERS = 0.001
FLATTENED = True

if len(sys.argv) < 2:
    print("Usage:",sys.argv[0],"QUERY_NAME [INDEX or 'random' or 'all']")
    sys.exit(0)
query = sys.argv[1]
index = 0
if len(sys.argv) > 2:
    if sys.argv[2] in ['random','all']:
        index = sys.argv[2]
    else:
        index = int(sys.argv[2])

lvis = objaverse.load_lvis_annotations()
ids = None
if query in lvis:
    ids = lvis[query]
else:
    print("Couldn't find exact match, finding best fuzzy match")
    best_match = None
    match_score = float('inf')
    for k, ids in lvis.items():
        res = fuzzysearch.find_near_matches(query, k, max_l_dist = 3)
        for match in res:
            start_penalty = match.start
            end_penalty = (len(k)-match.end)
            if k[match.start] == ' ' or k[match.start] == '_':
                start_penalty = 1
            if match.end < len(k):
                if k[match.end] == ' ' or k[match.end] == '_':
                    end_penalty = 1
            score = match.dist + start_penalty + end_penalty
            print("Match",k,"distance",match.dist,"range",match.start,match.end,"score",score)
            if score < match_score:
                best_match = k
                match_score = score
    if best_match is None:
        print("Failed to find a match to query",query)
        sys.exit(1)
    else:
        print("Best match: ",best_match,"with score",match_score)
        ids = lvis[best_match]

print("Found",len(ids),"objects for",query)
if isinstance(index,int):
    if index >= len(ids):
        print("Index",index,"is out of range")
        sys.exit(1)
    ids = [ids[index]]
elif index == 'random':
    import random
    ids = [random.choice(ids)]
elif index == 'all':
    pass
else:
    print("Invalid index",index)
    sys.exit(1)

objects = objaverse.load_objects(ids)

print("Downloaded object(s) to",list(objects.values()))
world = WorldModel()
x = 0
for i,objfile in enumerate(objects.values()):
    nobjects = world.numRigidObjects()

    s = trimesh.load(objfile)
    s.convert_units('meters')
    res = from_trimesh(s,flatten=FLATTENED)
    if isinstance(res,tuple):
        #result is just a single (mesh,appearance) tuple
        m,a = res
        g = Geometry3D(m)

        o = world.makeRigidObject("object"+str(i))
        o.geometry().set(g)
        o.appearance().set(a)
        o.appearance().refresh()
        
        bb = g.getBBTight()
    else:
        #result is a WorldModel
        for i in range(res.numRigidObjects()):
            world.add(res.rigidObject(i).getName(),res.rigidObject(i))
            world.rigidObject(nobjects+i).appearance().set(res.rigidObject(i).appearance())

    #Scale and shift the object(s), handle y-z flipping
    bb = bb_create()
    for i in range(nobjects,world.numRigidObjects()):
        bbi = world.rigidObject(i).geometry().getBBTight()
    bb = bb_union(bb,bbi)
    print("Bounding box",bb)
    maxdim = max(b-a for (a,b) in zip(bb[0],bb[1]))
    print("Max dimension",maxdim)
    if maxdim > 250:
        print("Assuming object scale is in millimeters")
        scale = MM_TO_METERS
    elif maxdim > 3:
        print("Assuming object scale is in inches")
        scale = INCHES_TO_METERS
    else:
        scale = 1.0
    #Need to flip the object around the x axis because y is up in objaverse
    flipyz = so3.from_axis_angle(([1,0,0],math.pi/2))
    bb = bb_create()
    for i in range(nobjects,world.numRigidObjects()):
        if scale != 1.0:
            g = world.rigidObject(i).geometry()
            g.transform(vectorops.mul(so3.identity(),scale),[0,0,0])
            T = world.rigidObject(i).getTransform()
            T = T[0],vectorops.mul(T[1],scale)
            world.rigidObject(i).setTransform(*T)
        #flip y and z 
        world.rigidObject(i).setTransform(*se3.mul((flipyz,[0,0,0]),world.rigidObject(i).getTransform()))
        world.rigidObject(i).appearance().refresh()
        bb = bb_union(bb,world.rigidObject(i).geometry().getBBTight())
    
    if i > 0:
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
