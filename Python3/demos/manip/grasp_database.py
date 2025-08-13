from klampt.model.gripperinfo import GripperInfo
from klampt.manip.grasp import Grasp
from klampt.manip.grasp_database import GraspDatabase
from klampt.math import se3
import os
import sys
from typing import Optional,List

def browse_database(gripper : GripperInfo, object_patterns : List[str], dbfile : Optional[str]=None):
    db = GraspDatabase(gripper)
    if dbfile is not None:
        if dbfile.endswith('.json'):
            db.load(dbfile)
        else:
            db.loadfolder(dbfile)

    def _find_object(name):
        for pat in object_patterns:
            fn = pat%(name,)
            if os.path.exists(fn):
                return fn
        return None

    w = WorldModel()
    if not w.readFile(gripper.klamptModel):
        print("Can't load gripper robot model",gripper.klamptModel)
        exit(1)
    robot = w.robot(0)
    if dbfile is None:
        #use glob to find objects
        from glob import glob
        for pat in object_patterns:
            for fn in glob(pat):
                if not os.path.exists(fn):
                    print("Object file",fn,"does not exist")
                    continue
                name = os.path.splitext(os.path.basename(fn))[0]
                if name in db.object_to_grasps:
                    print("Object",name,"already in database, skipping")
                    continue
                obj = w.makeRigidObject(name)
                if not obj.loadFile(fn):
                    if not obj.geometry().loadFile(fn):
                        print("Couldn't load object",name,"from",fn)
                        exit(1)
                db.addObject(name)
    else:
        for o in db.objects:
            fn = _find_object(o)
            if fn is None:
                print("Can't find object",o,"in usual paths...")
                continue
            if w.numRigidObjects()==0:
                obj = w.makeRigidObject(o)
                if not obj.loadFile(fn):
                    if not obj.geometry().loadFile(fn):
                        print("Couldn't load object",o,"from",fn)
                        exit(1)
                obj.setTransform(*se3.identity())
    if len(db.objects)==0:
        print("Can't show anything, no objects")
        print("Try adding some grasps to the database using grasp_edit.py")
        exit(0)

    data = dict()
    data['cur_object'] = 0
    data['cur_grasp'] = -1
    data['shown_grasps'] = []
    vis.add(db.objects[data['cur_object']],w.rigidObject(0))
    def shift_object(amt,data=data):
        vis.remove(db.objects[data['cur_object']])
        data['cur_object'] += amt
        if data['cur_object'] >= len(db.objects):
            data['cur_object'] = 0
        elif data['cur_object'] < 0:
            data['cur_object'] = len(db.objects)-1
        if data['cur_object'] >= w.numRigidObjects():
            for i in range(w.numRigidObjects(),data['cur_object']+1):
                o = db.objects[i]
                fn = _find_object(o)
                obj = w.makeRigidObject(o)
                if not obj.loadFile(fn):
                    if not obj.geometry().loadFile(fn):
                        print("Couldn't load object",o,"from",fn)
                        exit(1)
                obj.setTransform(*se3.identity())
        obj = w.rigidObject(data['cur_object'])
        vis.add(db.objects[data['cur_object']],obj)
        shift_grasp(None)

    def shift_grasp(amt,data=data):
        for i,grasp in data['shown_grasps']:
            grasp.removeFromVis("grasp"+str(i))
        data['shown_grasps'] = []
        all_grasps = db.object_to_grasps[db.objects[data['cur_object']]]
        if amt == None:
            data['cur_grasp'] = -1
        else:
            data['cur_grasp'] += amt
            if data['cur_grasp'] >= len(all_grasps):
                data['cur_grasp'] = -1
            elif data['cur_grasp'] < -1:
                data['cur_grasp'] = len(all_grasps)-1
        if data['cur_grasp']==-1:
            for i,grasp in enumerate(all_grasps):
                grasp.ikConstraint.robot = robot
                grasp.addToVis("grasp"+str(i))
                data['shown_grasps'].append((i,grasp))
            print("Showing",len(data['shown_grasps']),"grasps")
        else:
            grasp = all_grasps[data['cur_grasp']]
            grasp.ikConstraint.robot = robot
            grasp.addToVis("grasp"+str(data['cur_grasp']))
            Tbase = grasp.ikConstraint.closestMatch(*se3.identity())
            grasp.addToVis(robot,animate=False,base_xform=Tbase)
            robot.setConfig(grasp.setFingerConfig(robot.getConfig()))
            data['shown_grasps'].append((data['cur_grasp'],grasp))
            if grasp.score is not None:
                vis.addText("score","Score %.3f"%(grasp.score,),position=(10,10))
            else:
                vis.addText("score","",position=(10,10))

    vis.addAction(lambda: shift_object(1),"Next object",'.')
    vis.addAction(lambda: shift_object(-1),"Prev object",',')
    vis.addAction(lambda: shift_grasp(1),"Next grasp",'=')
    vis.addAction(lambda: shift_grasp(-1),"Prev grasp",'-')
    vis.addAction(lambda: shift_grasp(None),"All grasps",'0')
    vis.add("gripper",w.robot(0))
    vis.run()

if __name__ == '__main__':
    import sys
    from klampt import WorldModel
    from klampt import vis
    if len(sys.argv) < 3:
        print("Usage: python grasp_database.py GRIPPER_INFO.json OBJECT_PATTERN [DB_FILE]")
        exit(0)
    gripper = GripperInfo.load(sys.argv[1])
    object_patterns = sys.argv[2]
    dbfile = None
    if len(sys.argv) >= 4:
        dbfile = sys.argv[3]
    browse_database(gripper,[object_patterns],dbfile)
