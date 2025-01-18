#include <KrisLibrary/geometry/AnyGeometry.h>
#include <iostream>
using namespace Geometry;

void PrimitiveDistanceTest()
{
    Sphere3D s;
    Box3D b;
    b.dims.set(0.5,0.5,1.0);
    b.origin.set(0,0,0);
    b.xbasis.set(1,0,0);
    b.ybasis.set(0,1,0);
    b.zbasis.set(0,0,1);
    s.center.set(1.0,1.0,0.5);
    s.radius = 0.1;
    GeometricPrimitive3D gs(s);
    GeometricPrimitive3D gb(b);
    Assert(gs.SupportsDistance(gb.type));
    Assert(gs.SupportsClosestPoints(gb.type));
    printf("Distance %f\n",gs.Distance(gb));
    Vector3 cp,dir;
    Real d = gs.ClosestPoints(gb,cp,dir);
    cout<<"Closest point distance "<<d<<" closest point "<<cp<<" direction "<<dir<<endl;

    AnyGeometry3D ags(gs),agb(b);
    AnyCollisionGeometry3D cgs(ags),cgb(agb);
    RigidTransform T;
    T.R.setRotateY(0.5);
    T.t.set(4.0,0,-10.4);
    cgs.SetTransform(T);
    cgb.SetTransform(T);
    cout<<"Distance "<<cgs.Distance(cgb)<<endl;
}

void TriBoxTest()
{
    AABB3D bb;
    Triangle3D tri;
    bb.bmin.set(-0.5,-0.5,-0.5);
    bb.bmax.set(0.5,0.5,0.5);
    tri.a.set(1.5,0.5,0);
    tri.b.set(2,2,0);
    tri.c.set(0.5,1.5,0);
    Segment3D s;
    s.a = tri.a;
    s.b = tri.c;
    Real uclosest;
    Vector3 tclosest,bclosest;
    Real d;
    d = s.distance(bb,uclosest,bclosest);
    cout<<"Segment closest point "<<s.eval(uclosest)<<", box closest point "<<bclosest<<", distance "<<d<<endl;
    d = tri.distance(bb,tclosest,bclosest);
    cout<<"Triangle closest point "<<tclosest<<", box closest point "<<bclosest<<", distance "<<d<<endl;
    Box3D b;
    RigidTransform T;
    T.R.setIdentity();
    T.t.set(-2.0,0.0,0.0);
    b.setTransformed(bb,T);
    d = b.distance(tri,bclosest,tclosest);
    cout<<"Triangle - box closest point "<<tclosest<<", box closest point "<<bclosest<<", distance "<<d<<endl;

    //triangle - box intersection with witness
    Vector3 witness;
    bool coll = tri.intersects(bb,witness);
    cout<<"Triangle intersects box (should be false)? "<<coll<<", separating direction "<<witness<<endl;
    bb.bmax.set(1.1,1.1,0.5);
    coll = tri.intersects(bb,witness);
    cout<<"Triangle intersects box (should be true)? "<<coll<<", witness point "<<witness<<endl;
    bb.bmax.set(0.5,0.5,0.5);
    tri.a.set(-1.0,-0.1,0); //triangle pierces box
    tri.b.set(1.0,0,0);
    tri.c.set(-1.0,0.1,0);
    coll = tri.intersects(bb,witness);
    cout<<"Triangle intersects box (should be true)? "<<coll<<", witness point "<<witness<<endl;
    //box through triangle
    tri.a.set(-10,-10,0.7);
    tri.b.set(10,-10,0.7);
    tri.c.set(10,10,0.3);
    coll = tri.intersects(bb,witness);
    cout<<"Triangle intersects box (should be true)? "<<coll<<", witness point "<<witness<<endl;
    tclosest = tri.closestPoint(witness);
    Assert(tclosest.distance(witness) < 1e-4);

    tri.a.set(-0.01,0.1,0.3);
    tri.b.set(-0.1,0,0.3);
    tri.c.set(-0.9,0,0.3);
    bb.bmin.set(0,0,0);
    bb.bmax.set(0.5,0.25,0.75);
    coll = tri.intersects(bb,witness);
    cout<<"Triangle intersects box (should be false)? "<<coll<<", separating direction "<<witness<<endl;
}

int main(int argc,const char** argv)
{
    PrimitiveDistanceTest();
    TriBoxTest();
    return 0;
}