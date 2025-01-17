from klampt import vis
from klampt.math import vectorops
from klampt import GeometricPrimitive
from klampt.model.trajectory import Trajectory,HermiteTrajectory
import math
import time
from typing import List,Sequence,Tuple,Union

def svg_to_trajectories(fn : str, scale='auto', center=False,dims=3,
    fit=1, want_attributes=False) -> List[Union[Trajectory,HermiteTrajectory]]:
    """Reads one or more trajectories from a SVG file.  The svgpathtools
    library must be installed.

    To debug, try::

        trajs,attrs = svg_to_trajectories('test.svg',center=True,want_attributes=True)
        for i,(traj,attr) in enumerate(zip(trajs,attrs)):
            name = attr.get('name',"path %d"%i)
            vis.add(name,traj)
            for a,v in attr.items():
                if a != 'name':
                    vis.setAttribute(name,a,v)
        vis.loop()
    
    Args:
        fn (str): the filename
        scale (float, optional): if given, scales the units in fn to world
            units.  'auto' will fit the drawing to the box [0,fit]x[0,fit].
        center (bool or tuple, optional): if True, sets the drawing's origin
            to its center. If a 2-tuple, this is a center in world units.
            If false, no centering is done.
        fit (float, optional): if scale = 'auto', the drawing will be resized
            to fit into a box this size.
        dims (int, optional): either 2 or 3, depending on whether you want
            a 2D or 3D trajectory.
        want_attributes (bool, optional): if given, also tries parsing the
            attributes of the paths in the SVG file.  The attributes can
            be passed directly to the vis module.

    Returns:
        list of Trajectory: zero or more trajectories from the file.  If
        want_attributes = True, this also returns a list of dicts giving attributes
        parsed from the SVG file that can be given to the vis module.
    """
    from svgpathtools import svg2paths
    from klampt.model.collide import bb_create,bb_union
    paths, attributes = svg2paths(fn)
    trajs = []
    attrs = []
    for i,p in enumerate(paths):
        traj = svg_path_to_trajectory(p)
        traj.checkValid()
        if len(traj.milestones) == 0:
            print("Path",i,"is invalid, has",len(traj.milestones),"milestones")
            continue
        trajs.append(traj)
        attrs.append(attributes[i])        
    print("Read",len(trajs),"paths")
    if scale == False:
        scale = 1
    shift = [0,0]
    if isinstance(center,(tuple,list)):
        shift = [-center[0],-center[1]]
    if center == True or scale == 'auto':
        bounds = bb_create()
        for traj in trajs:
            if isinstance(traj,HermiteTrajectory):
                traj = traj.discretize(10)
            bounds = bb_union(bounds,bb_create(*traj.milestones))
        print("Bounds:",bounds)
        if center == True and scale == 'auto':
            scale = fit / max(bounds[1][0]-bounds[0][0],bounds[1][1]-bounds[0][1])
            shift = [-0.5*(bounds[1][0]+bounds[0][0])*scale,-0.5*(bounds[1][1]+bounds[0][1])*scale]
        elif center == True:
            shift = [-0.5*(bounds[1][0]+bounds[0][0])*scale,-0.5*(bounds[1][1]+bounds[0][1])*scale]
        elif scale == 'auto':
            scale = fit / max(bounds[1][0],bounds[1][1])
    print("Shift",shift,"scale",scale)
    for traj in trajs:
        if len(traj.milestones[0]) == 2:
            for j,m in enumerate(traj.milestones):
                traj.milestones[j] = vectorops.add(shift,vectorops.mul(m,scale))
        else:
            for j,m in enumerate(traj.milestones):
                traj.milestones[j] = vectorops.add(shift,vectorops.mul(m[:2],scale)) + vectorops.mul(m[2:],scale)
    if dims == 3:
        for traj in trajs:
            if len(traj.milestones[0]) == 2:
                for j,m in enumerate(traj.milestones):
                    traj.milestones[j] = m + [0.0]
            else:
                for j,m in enumerate(traj.milestones):
                    traj.milestones[j] = m[:2] + [0.0] + m[2:] + [0.0]
    if want_attributes:
        parsed_attrs = []
        for a in attrs:
            parsed = dict()
            styledict = a
            if 'id' in a:
                parsed['name'] = a['id']
            if 'style' in a:
                styledict = dict(v.split(':') for v in a['style'].split(';'))
            else:
                print(a)
            if 'stroke' in styledict:
                rgb = styledict['stroke'].strip().strip('#')
                a = 1
                if 'opacity' in styledict:
                    a = float(styledict['opacity'])
                if 'stroke-opacity' in styledict:
                    a = float(styledict['stroke-opacity'])
                if len(rgb)==3:
                    r,g,b = rgb
                    r = int(r,16)/0xf
                    g = int(g,16)/0xf
                    b = int(b,16)/0xf
                    parsed["color"] = (r,g,b,a)
                elif len(rgb)==6:
                    r,g,b = rgb[0:2],rgb[2:4],rgb[4:6]
                    r = int(r,16)/0xff
                    g = int(g,16)/0xff
                    b = int(b,16)/0xff
                    parsed["color"] = (r,g,b,a)
            if 'stroke-width' in styledict:
                parsed['width'] = float(styledict['stroke-width'].strip('px'))
            parsed_attrs.append(parsed)
        return trajs,parsed_attrs
    return trajs


def svg_path_to_trajectory(p) -> Union[Trajectory,HermiteTrajectory]:
    """Produces either a Trajectory or HermiteTrajectory according to an SVG
    Path.  The path must be continuous.
    """
    from svgpathtools import Path, Line, QuadraticBezier, CubicBezier, Arc
    if not p.iscontinuous():
        raise ValueError("Can't do discontinuous paths")
    pwl = not any(isinstance(seg,(CubicBezier,QuadraticBezier)) for seg in p)
    if pwl:
        milestones = []
        for seg in p:
            a,b = seg.start,seg.end
            if not milestones:
                milestones.append([a.real,a.imag])
            milestones.append([b.real,b.imag])
        return Trajectory(list(range(len(milestones))),milestones)
    times = []
    milestones = []
    for i,seg in enumerate(p):
        if isinstance(seg,CubicBezier):
            a,c1,c2,b = seg.start,seg.control1,seg.control2,seg.end
            vstart = (c1.real-a.real)*3,(c1.imag-a.imag)*3
            vend = (b.real-c2.real)*3,(b.imag-c2.imag)*3
            if not milestones:
                milestones.append([a.real,a.imag,vstart[0],vstart[1]])
                times.append(i)
            elif vectorops.distance(milestones[-1][2:],vstart) > 1e-4:
                milestones.append([a.real,a.imag,0,0])
                times.append(i)
                milestones.append([a.real,a.imag,vstart[0],vstart[1]])
                times.append(i)
            milestones.append([b.real,b.imag,vend[0],vend[1]])
            times.append(i+1)
        elif isinstance(seg,Line):
            a,b = seg.start,seg.end
            if not milestones:
                milestones.append([a.real,a.imag,0,0])
                times.append(i)
            elif vectorops.norm(milestones[-1][2:]) > 1e-4:
                milestones.append([a.real,a.imag,0,0])
                times.append(i)
            milestones.append([b.real,b.imag,0,0])
            times.append(i+1)
        else:
            raise NotImplementedError("Can't handle pieces of type {} yet".format(seg.__class.__name__))
    return HermiteTrajectory(times,milestones)

def svg_path_to_polygon(p,dt=0.1) -> GeometricPrimitive:
    traj = svg_path_to_trajectory(p)
    if isinstance(traj,HermiteTrajectory):
        traj = traj.discretize(dt)
    verts = sum(traj.milestones,[])
    g = GeometricPrimitive()
    g.setPolygon(verts)
    return g

if __name__ == '__main__':
    try:
        import svgpathtools
    except ImportError:
        print('svgpathtools module could not be found.  Try "pip install svgpathtools".')
        exit(1)
    import sys
    fn = 'test.svg'
    if len(sys.argv) > 1:
        fn = sys.argv[1]

    from klampt import WorldModel
    from klampt.model import ik
    w = WorldModel()
    w.readFile("../../data/robots/kinova_with_robotiq_85.urdf")
    robot = w.robot(0)

    trajs,attrs = svg_to_trajectories(fn,center=True,want_attributes=True)
    for i,(traj,attr) in enumerate(zip(trajs,attrs)):
        name = attr.get('name',"path %d"%i)
        vis.add(name,traj)
        for a,v in attr.items():
            if a != 'name':
                vis.setAttribute(name,a,v)

    #place on a reasonable height and offset
    scale = 0.4
    xshift = 0.35
    tableh = 0.1
    for traj in trajs:
        for m in traj.milestones:
            m[0] = m[0]*scale + xshift
            m[1] = m[1]*scale
            m[2] = tableh
            if len(m) == 6:  #hermite, m[3:6] are the tangents
                m[3] *= scale
                m[4] *= scale

    vis.add("world",w)
    data={
        'config':robot.getConfig(),
        'state':'moveto',
        'queue':[],
        'traj_index':0,
        'traj_start':vis.animationTime()
        }
    def callback(data=data):
        #implement a finite state machine that "draws" the paths in trajs using kinematic simulation
        t = vis.animationTime()
        state = data['state']
        idx = data['traj_index']
        ts = data['traj_start']
        #parameters for end effector pen
        ee = robot.link('EndEffector_Link')
        ee_offset = 0.2
        lifth = 0.05
        #state machine logic
        robot.setConfig(data['config'])
        vis.addText("state",state,(20,20))
        h = 0.1
        if state == 'moveto':
            if not data['queue']:
                pt = trajs[idx].eval(0)
                ee_offset += lifth
                #objective is an axis constrained via two points
                obj = ik.objective(ee,local=[(0,0,ee_offset-h),(0,0,ee_offset)],world=[(pt[0],pt[1],pt[2]+h),pt])
                ik.solve(obj)
                data['queue'] = [robot.getConfig()]
            elif robot.distance(data['queue'][-1],data['config']) < 1e-4:
                data['state'] = 'movedown'
        elif data['state'] == 'movedown':
            wp = ee.getWorldPosition((0,0,ee_offset))
            if data['queue']:
                robot.setConfig(data['queue'][-1])
            wp[2] = max(wp[2]-0.01,tableh)
            obj = ik.objective(ee,local=[(0,0,ee_offset-h),(0,0,ee_offset)],world=[(wp[0],wp[1],wp[2]+h),wp])
            ik.solve_nearby(obj,maxDeviation=0.2)
            data['queue'].append(robot.getConfig())
            if wp[2] <= tableh:
                data['state'] = 'trace'
                data['traj_start'] = vis.animationTime()
                data['queue'] = []
        elif data['state'] == 'trace':
            pt = trajs[idx].eval(t-ts)
            if data['queue']:
                robot.setConfig(data['queue'][-1])
            obj = ik.objective(ee,local=[(0,0,ee_offset-h),(0,0,ee_offset)],world=[(pt[0],pt[1],pt[2]+h),pt])
            ik.solve_nearby(obj,maxDeviation=0.2)
            data['queue'].append(robot.getConfig())
            if t-ts > trajs[idx].endTime():
                idx = (idx + 1)%len(trajs)
                data['traj_index'] = idx
                data['state'] = 'moveup'
        elif data['state'] == 'moveup':
            wp = ee.getWorldPosition((0,0,ee_offset))
            if data['queue']:
                robot.setConfig(data['queue'][-1])
            wp[2] = min(wp[2]+0.01,tableh+lifth)
            obj = ik.objective(ee,local=[(0,0,ee_offset-h),(0,0,ee_offset)],world=[(wp[0],wp[1],wp[2]+h),wp])
            ik.solve_nearby(obj,maxDeviation=0.2)
            data['queue'].append(robot.getConfig())
            if wp[2] >= tableh+lifth:
                data['state'] = 'moveto'
                data['queue'] = []

        #simulate motion queue
        qcur = data['config']
        while len(data['queue']) > 0:
            qdest = data['queue'][0]
            amt = 1
            vmax = 0.03
            for i in range(len(qcur)):
                di = qcur[i]-qdest[i]
                if robot.getJointType(i) == 'spin':
                    di = di%(math.pi*2)
                    if di > math.pi:
                        di -= math.pi*2
                    if di < -math.pi:
                        di += math.pi*2
                #print("joint diff",di)
                if abs(di)*amt > vmax:
                    amt = vmax/abs(di)
            #print("Move amount",amt)
            #input()
            if amt == 1:
                if len(data['queue'])==1:
                    data['config'] = data['queue'][0]
                    break
                else:
                    data['queue'].pop(0)
            else:
                data['config'] = robot.interpolate(qcur,qdest,amt)
                robot.setConfig(data['config'])
                break
        time.sleep(0.01)
    vis.loop(callback=callback)
