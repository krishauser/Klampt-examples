#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/math/math.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/meshing/Rasterize.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math/random.h>
#include <Klampt/Interface/WorldViewProgram.h>
using namespace Geometry;
using namespace GLDraw;


void ConvertTest(const AnyGeometry3D& geom,const char* name)
{
    printf("Geometry of type %s\n",geom.TypeName());
    printf("Geometry data of type %s\n",geom.data->TypeName(geom.data->GetType()));
    printf("  Convert / save to %s.geom, %s.off, %s.pcd, %s.sdf, %s.occ, %s_heightmap.json\n",name,name,name,name,name,name);
    stringstream ss;
    AnyGeometry3D temp;
    if(!geom.Convert(Geometry3D::Type::Primitive,temp)) {
        printf("Couldn't convert to primitive\n");
    }
    else {
        cout<<"primitive type "<<temp.TypeName()<<endl;
        cout<<"primitive data type "<<temp.data->TypeName(temp.data->GetType())<<endl;
        cout<<"primitive bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<".geom";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
    if(!geom.Convert(Geometry3D::Type::TriangleMesh,temp)) {
        printf("Couldn't convert to triangle mesh\n");
    }
    else {
        cout<<"triangle mesh type "<<temp.TypeName()<<endl;
        cout<<"triangle data type "<<temp.data->TypeName(temp.data->GetType())<<endl;
        cout<<"triangle mesh bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<".off";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
    if(!geom.Convert(Geometry3D::Type::PointCloud,temp)) {
        printf("Couldn't convert to point cloud\n");
    }
    else {
        cout<<"point cloud type "<<temp.TypeName()<<endl;
        cout<<"point cloud bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<".pcd";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
    if(!geom.Convert(Geometry3D::Type::ImplicitSurface,temp,0.01)) {
        printf("Couldn't convert to SDF\n");
    }
    else {
        cout<<"SDF type "<<temp.TypeName()<<endl;
        cout<<"SDF bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<".sdf";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
    if(!geom.Convert(Geometry3D::Type::OccupancyGrid,temp,0.01)) {
        printf("Couldn't convert to occupancy grid\n");
    }
    else {
        cout<<"occupancy grid type "<<temp.TypeName()<<endl;
        cout<<"occupancy grid bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<".occ";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
    if(!geom.Convert(Geometry3D::Type::Heightmap,temp,0.005)) {
        printf("Couldn't convert to heightmap\n");
    }
    else {
        cout<<"heightmap type "<<temp.TypeName()<<endl;
        cout<<"heightmap bb "<<temp.GetAABB()<<endl;
        ss.str(""); ss<<name<<"_heightmap.json";
        if(!temp.Save(ss.str().c_str())) printf("Couldn't save to %s\n",ss.str().c_str());
    }
}


class RayHoverProgram : public Klampt::WorldViewProgram
{
public:
    RayHoverProgram(Klampt::WorldModel* world) : Klampt::WorldViewProgram(world) {}

    virtual void Handle_Click(int button,int state,int x,int y) override {
        if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
            Ray3D r;
            ClickRay(x,y,r);
            Vector3 worldpt;
            int res = world->RayCast(r,worldpt);
            if(res >= 0)
                cout<<"Hit object "<<res<<" at "<<worldpt<<endl;
        }
        else
            Klampt::WorldViewProgram::Handle_Click(button,state,x,y);
    }
};

void ShowConversionMatrix(const vector<AnyGeometry3D>& geoms,const vector<string>& labels)
{
    vector<AnyCollisionGeometry3D> cgeoms(geoms.size());
    vector<vector<shared_ptr<AnyGeometry3D> > > conversionResults;
    RigidTransform T0;
    T0.R.setRotateX(DtoR(30.0));
    T0.t.set(1.0,0.0,0.0);
    //initialization

    for(size_t i=0;i<geoms.size();i++) {
        cgeoms[i] = AnyCollisionGeometry3D(geoms[i]);
        cgeoms[i].SetTransform(T0);
    }
    for(int i=0;i<(int)Geometry3D::Type::Group;i++) {
        conversionResults.resize(conversionResults.size()+1);
        for(size_t j=0;j<geoms.size();j++) {
            shared_ptr<AnyGeometry3D> temp = make_shared<AnyGeometry3D>();
            bool success;
            success = geoms[j].Convert((Geometry3D::Type)i,*temp);
            if(!success)
                temp.reset();
            conversionResults.back().push_back(temp);
        }
    }

    Klampt::WorldModel world;
    for(size_t i=0;i<geoms.size();i++) {
        world.AddRigidObject(labels[i].c_str(),new Klampt::RigidObjectModel());
        world.rigidObjects.back()->geometry.CreateEmpty();
        *world.rigidObjects.back()->geometry = cgeoms[i];
        world.rigidObjects.back()->geometry.OnGeometryChange();
        world.rigidObjects.back()->geometry.Appearance()->silhouetteRadius = 0;
        world.rigidObjects.back()->geometry.Appearance()->faceColor.set(1,1,0,1.0);
        world.rigidObjects.back()->geometry.Appearance()->edgeColor.set(0,0,0,0.5);
        if(i >= 7)
            world.rigidObjects.back()->geometry.Appearance()->drawEdges = false;
        else
            world.rigidObjects.back()->geometry.Appearance()->drawEdges = true;

        RigidTransform T = T0;
        T.t.set((Real)i,0.0,0.0);
        world.rigidObjects.back()->T = T;
        world.rigidObjects.back()->geometry->SetTransform(T);
    }
    for(size_t i=0;i<geoms.size();i++) {
        for(size_t j=0;j<conversionResults.size();j++) {
            if(!conversionResults[j][i]) continue;
            string label = labels[i] + "->" + string(Geometry3D::TypeName(Geometry3D::Type(j)));
            world.AddRigidObject(label.c_str(),new Klampt::RigidObjectModel());
            world.rigidObjects.back()->geometry.CreateEmpty();
            *world.rigidObjects.back()->geometry = *conversionResults[j][i];
            world.rigidObjects.back()->geometry.OnGeometryChange();
            world.rigidObjects.back()->geometry.Appearance()->silhouetteRadius = 0;
            world.rigidObjects.back()->geometry.Appearance()->creaseAngle = 0;
            world.rigidObjects.back()->geometry.Appearance()->faceColor.set(1,0.5,0,0.75);
            world.rigidObjects.back()->geometry.Appearance()->edgeColor.set(0,0,0,0.5);
            if(i >= 7 && j >= (int)Geometry3D::Type::ImplicitSurface)
                world.rigidObjects.back()->geometry.Appearance()->drawEdges = false;
            else
                world.rigidObjects.back()->geometry.Appearance()->drawEdges = true;

            RigidTransform T=T0;
            T.t.set((Real)i,0,-(Real)j-1);
            world.rigidObjects.back()->T = T;
            world.rigidObjects.back()->geometry->SetTransform(T);
        }
    }
    Klampt::WorldViewProgram gui(&world);
    gui.Run();
}

void VisualizeConversions(const char** argv, int argc)
{
    if(argc == 0) {
        vector<AnyGeometry3D> geoms(8);
        vector<string> labels(8);
        Sphere3D s;
        s.radius = 0.4;
        s.center.set(-1.5,0.4,2.0);
        geoms[0] = AnyGeometry3D(GeometricPrimitive3D(s));
        labels[0] = "Primitive";
        bool res = geoms[0].Convert(Geometry3D::Type::ConvexHull,geoms[1]);
        Assert(res);
        labels[1] = "ConvexHull";
        res = geoms[0].Convert(Geometry3D::Type::TriangleMesh,geoms[2]);
        Assert(res);
        labels[2] = "TriangleMesh";
        res = geoms[2].Convert(Geometry3D::Type::PointCloud,geoms[3]);
        Assert(res);
        labels[3] = "PointCloud";
        res = geoms[2].Convert(Geometry3D::Type::ImplicitSurface,geoms[4]);
        Assert(res);
        labels[4] = "ImplicitSurface";
        res = geoms[0].Convert(Geometry3D::Type::OccupancyGrid,geoms[5]);
        Assert(res);
        labels[5] = "OccupancyGrid";
        res = geoms[0].Convert(Geometry3D::Type::Heightmap,geoms[6]);
        Assert(res);
        labels[6] = "Heightmap (orthographic)";
        geoms[7] = geoms[6];
        auto& hm = geoms[7].AsHeightmap();
        hm.viewport.perspective = true;
        hm.Shift(2.0);
        for(int i=0;i<hm.heights.m;i++)
            for(int j=0;j<hm.heights.n;j++)
                if(hm.heights(i,j) == 2.0)
                    hm.heights(i,j) = 0;
        hm.viewport.pose.t.z -= 2.0;
        hm.viewport.fx *= 2.0;
        hm.viewport.fy *= 2.0;
        hm.viewport.ori = Camera::CameraConventions::OpenCV;
        labels[7] = "Heightmap (perspective)";
        ShowConversionMatrix(geoms,labels);
    }
    else {
        vector<AnyGeometry3D> geoms(argc);
        vector<string> labels(argc);
        for(int i=0;i<argc;i++) {
            if(!geoms[i].Load(argv[i])) {
                printf("Couldn't load %s\n",argv[i]);
                return;
            }
            labels[i] = argv[i];
        }
        ShowConversionMatrix(geoms,labels);
    }
}

void ConvexHullTest()
{
    ConvexHull3D ch;
    vector<Vector3> pts(4);
    pts[0].set(-1,-1,-1);
    pts[1].set(1,-1,-1);
    pts[2].set(0,1,-1);
    pts[3].set(0,0,1);
    ch.SetPoints(pts);
    Ray3D r;
    r.source.set(0,0,-3.0);
    r.direction.set(0,0,1);
    Real dist;
    cout<<"Collides? "<<ch.RayCast(r,&dist)<<" should be 1"<<endl;
    cout<<"dist "<<dist<<" should be 2"<<endl;
}




#include <KrisLibrary/geometry/SpiralIterator.h>

void SpiralIteratorTest()
{
    { //basic test
        Array2D<int> visited(3,3,0);
        SpiralIterator it(IntPair(0,0),IntPair(-1,-1),IntPair(1,1));
        for(int i=0;i<5*5;i++) {
            if(it.isDone()) { 
                printf("finished after %d cells\n",i);
                break;
            }
            if(abs(it->a) > 1 || abs(it->b) > 1) {
                printf("ERROR: cell %d %d out of bounds\n",it->a,it->b);
            }
            else {
                if(visited(it->a+1,it->b+1)) {
                    printf("ERROR: cell %d %d visited twice\n",it->a,it->b);
                }
                visited(it->a+1,it->b+1) += 1;
            }
            //cout<<"("<<it->a<<","<<it->b<<")"<<endl;
            ++it;
        }
        for(int i=0;i<visited.m;i++)
            for(int j=0;j<visited.n;j++)
                if(visited(i,j) != 1)
                    printf("ERROR: cell %d %d visited %d times\n",i,j,visited(i,j));
    }
    {
        //SpiralIterator it(IntPair(0,0),IntPair(-7,-7),IntPair(-3,-3));
        //SpiralIterator it(IntPair(0,0));
        SpiralIterator it(IntPair(0,0),IntPair(0,0),IntPair(4,4));
        for(int i=0;i<5*5;i++) {
            if(it.isDone()) { 
                printf("finished after %d cells\n",i);
                break;
            }
            cout<<"("<<it->a<<","<<it->b<<")"<<endl;
            ++it;
        }
    }
    { //speed test
        SpiralIterator it(IntPair(0,0),IntPair(100,100),IntPair(400,400));
        Timer timer;
        int sum = 0;
        for(int i=0;i<300;i++) 
            for(int j=0;j<300;j++) 
                sum += j - i;
        printf("Summing 300x300 grid with normal iteration took %gs\n",timer.ElapsedTime());
        timer.Reset();
        sum = 0;
        while(!it.isDone()) {
            ++it;
            sum += it->a - it->b;
        }
        printf("Summing 300x300 grid with spiral iteration took %gs\n",timer.ElapsedTime());
    }
    { //basic test
        Array3D<int> visited(3,3,3, 0);
        SpiralIterator3D it(IntTriple(0,0,0));
        for(int i=0;i<27;i++) {
            if(it.isDone()) {
                printf("finished after %d cells\n",i);
                break;
            }
            if(abs(it->a) > 1 || abs(it->b) > 1 || abs(it->c) > 1) {
                printf("ERROR: cell %d %d %d out of bounds\n",it->a,it->b,it->c);
            }
            else {
                if(visited(it->a+1,it->b+1,it->c+1)) {
                    printf("ERROR: cell %d %d %d visited twice\n",it->a,it->b,it->c);
                }
                visited(it->a+1,it->b+1,it->c+1) += 1;
            }
            //cout<<"("<<it->a<<","<<it->b<<","<<it->c<<")"<<endl;
            ++it;
        }
        for(int i=0;i<visited.m;i++)
            for(int j=0;j<visited.n;j++)
                for(int k=0;k<visited.p;k++)
                    if(visited(i,j,k) != 1)
                        printf("ERROR: cell %d %d %d visited %d times\n",i,j,k,visited(i,j,k));
    }
    if(0)
    {
        SpiralIterator3D it(IntTriple(0,0,0),IntTriple(0,0,0),IntTriple(2,2,2));
        for(int i=0;i<27;i++) {
            if(it.isDone()) {
                printf("finished after %d cells\n",i);
                break;
            }
            cout<<"("<<it->a<<","<<it->b<<","<<it->c<<")"<<endl;
            ++it;
        }
    }
    { //speed test
        SpiralIterator3D it(IntTriple(0,0,0),IntTriple(100,100,100),IntTriple(400,400,400));
        Timer timer;
        int sum = 0;
        for(int i=0;i<300;i++) 
            for(int j=0;j<300;j++) 
                for(int k=0;k<300;k++)
                    sum += j - i + k;
        printf("Summing 300x300x300 grid with normal iteration took %gs\n",timer.ElapsedTime());
        timer.Reset();
        sum = 0;
        while(!it.isDone()) {
            ++it;
            sum += it->a - it->b;
        }
        printf("Summing 300x300x300 grid with spiral iteration took %gs\n",timer.ElapsedTime());
    }
}

int main(int argc,const char** argv)
{
    //SpiralIteratorTest();
    //ConvexHullTest();
    printf("=========================================================\n");
    printf("Usage: bin/GeometryTest [OP] [file1] [file2] ...\n");
    printf("  OP = showconvert: converts a file to all possible formats and shows them\n");
    printf("  OP = convert: converts a file to various formats\n");
    printf("  OP = merge: tests merging a file into a heightmap\n");
    printf("  other: loads the given files and shows them\n");
    printf("=========================================================\n");
    printf("\n");

    Klampt::WorldModel world;
    RayHoverProgram gui(&world);
    if(argc <= 1) {
        Sphere3D s;
        s.radius = 0.4;
        s.center.set(1.5,0.4,2.0);
        GeometricPrimitive3D prim(s);
        AnyGeometry3D gprim(prim);
        ConvertTest(gprim,"geometry_test_save/sphere");
    }
    else if(0==strcmp(argv[1],"convert")) {
        if(argc <= 2) {
            printf("geometrytest convert [file]\n");
            return 1;
        }
        AnyGeometry3D geom;
        if(!geom.Load(argv[2])) {
            printf("Couldn't load %s\n",argv[2]);
            return 1;
        }
        string prefix = argv[2];
        prefix = prefix.substr(0,prefix.rfind('.'));
        ConvertTest(geom,prefix.c_str());
    }
    else if(0==strcmp(argv[1],"showconvert")) {
        VisualizeConversions(argv+2,argc-2);
    }
    else if(0==strcmp(argv[1],"merge")) {
        if(argc <= 2) {
            printf("geometrytest merge [file]\n");
            return 1;
        }
        AnyGeometry3D geom;
        if(!geom.Load(argv[2])) {
            printf("Couldn't load %s\n",argv[2]);
            return 1;
        }
        AnyCollisionGeometry3D cgeom(geom);
        Meshing::Heightmap hm;
        hm.SetSize(1.0,1.0);
        hm.heights.resize(256,256);
        hm.heights.set(0.0);
        AnyGeometry3D sdf(hm);
        /*
        Meshing::VolumeGrid vg;
        //vg.bb.bmin.set(-2,-2,-2);
        //vg.bb.bmax.set(2,2,2);
        vg.bb.bmin.set(-0.5,-0.5,-0.5);
        vg.bb.bmax.set(0.5,0.5,0.5);
        vg.value.resize(128,128,128);
        //implicit surface
        vg.value.set(Inf);
        AnyGeometry3D sdf(vg,Geometry::VolumeGridImplicitSurface);
        //occupancy grid
        //vg.value.set(0);
        //AnyGeometry3D sdf(vg,Geometry::VolumeGridOccupancyGrid);
        */
        AnyCollisionGeometry3D csdf(sdf);
        RigidTransform dT;
        //Vector3 c(-1,-1,0);
        Vector3 c(-0.25,-0.25,0);
        Vector3 dx(0,0.01,0);
        dT.R.setRotateZ(Math::DtoR(30.0)*0);
        dT.t = c - dT.R*c + dx;
        cgeom.InitCollisionData();
        csdf.InitCollisionData();
        if(!csdf.Merge(cgeom)) {
            printf("Couldn't merge %s into %s\n",argv[2],sdf.TypeName());
            return 1;
        }
        RigidTransform T = cgeom.GetTransform();
        for(int steps=0;steps<10;steps++) {
            T = T*dT;
            cgeom.SetTransform(T);
            csdf.Merge(cgeom);
        }
        
        world.AddTerrain("terrain",new Klampt::TerrainModel());
        world.terrains[0]->geometry.CreateEmpty();
        *world.terrains[0]->geometry = csdf;
        world.terrains[0]->geometry.OnGeometryChange();
        world.terrains[0]->geometry.Appearance()->silhouetteRadius = 0;
        return gui.Run("Geometry Test");        
    }
    else {
        for(int i=1;i<argc;i++) {
            world.AddTerrain("terrain",new Klampt::TerrainModel());
            if(!world.terrains[i-1]->LoadGeometry(argv[i])) {
                printf("Couldn't load %s\n",argv[i]);
                return 1;
            }
            world.terrains[i-1]->geometry.Appearance()->silhouetteRadius = 0;
            if(i==1) world.terrains[i-1]->geometry.Appearance()->SetColor(1,1,1,1);
            else world.terrains[i-1]->geometry.Appearance()->SetColor(1,float(i)/float(argc),0,1);
            //world.terrains[i-1]->geometry.Appearance()->drawFaces = false;
            //world.terrains[i-1]->geometry.Appearance()->drawEdges = true;
        }
        return gui.Run("Geometry Test");
    }
    return 0;
}
