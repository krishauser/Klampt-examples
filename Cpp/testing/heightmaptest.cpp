#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/meshing/MeshPrimitives.h>
#include <KrisLibrary/math/random.h>
#include <Klampt/Interface/WorldViewProgram.h>
using namespace Geometry;
using namespace GLDraw;

static const bool TEST_LOAD = true; 
static const bool TEST_INVALID = true; 
static const bool TEST_PERSPECTIVE = true;
static const bool TEST_COLORS = true;
static const bool TEST_FUSION = false;
static const bool TEST_CONVERSIONS = false;
    
void HeightmapTest()
{
    Meshing::Heightmap hm;
    if(TEST_LOAD) {
        const char* fn = "../../data/terrains/lunar_gld100_dtm2_2970.json";
        //const char* fn = "../../data/terrains/mars_2020_ctx_dtm_elevation_1024.json";
        //const char* fn = "../../data/terrains/sunrgbdv2.json";
        if(!hm.Load(fn)) {
            printf("Couldn't load from %s\n",fn);
            return;
        }
        hm.viewport.pose.setIdentity();
        if(hm.viewport.perspective) {
            //place in a more standard forward orientation
            Matrix3 I; I.setIdentity();
            if(hm.viewport.pose.R == I)
                hm.viewport.pose.R.setRotateX(DtoR(-90.0));
            cout<<"Pose rotation"<<endl;
            cout<<hm.viewport.pose.R<<endl;
            cout<<"RIGHT DIRECTION "<<hm.viewport.right()<<endl;
            cout<<"UP DIRECTION "<<hm.viewport.up()<<endl;
            cout<<"FORWARD DIRECTION "<<hm.viewport.forward()<<endl;
        }
    }
    else {
        hm.Resize(200,100);
        hm.SetSize(2.0,1.0);
        hm.heights.set(0.0);
        for(int i=0;i<hm.heights.m;i++)
            for(int j=0;j<hm.heights.n;j++)
                hm.heights(i,j) = Real(j)/Real(hm.heights.n)*1.0 + 0.5*sin(2.0*Pi*Real(i)/Real(hm.heights.m));
        for(int i=0;i<hm.heights.m;i++)
            hm.heights(i,hm.heights.n-1) = -1.0;  //bottom row
        //invalid values
        if(TEST_INVALID) {
            for(int i=50;i<60;i++)
                for(int j=20;j<80;j++)
                    hm.heights(i,j) = Inf;
        }
        //TEST PERSPECTIVE
        if(TEST_PERSPECTIVE) {
            hm.SetFOV(DtoR(90.0));
            for(int i=0;i<hm.heights.m;i++)
                for(int j=0;j<hm.heights.n;j++)
                    hm.heights(i,j) += 2.1;
            hm.viewport.pose.R.setRotateX(DtoR(-90.0));
            // hm.viewport.pose.t.x += 5.0;
            // hm.viewport.pose.t.z -= 1.5;
        }
        //TEST COLORS
        if(TEST_COLORS) {
            hm.colors.initialize(hm.heights.m,hm.heights.n,Image::R8G8B8);
            for(int i=0;i<hm.heights.m;i++)
                for(int j=0;j<hm.heights.n;j++)
                    hm.SetVertexColor(i,j,Vector3(Real(i)/hm.heights.m,Real(j)/hm.heights.n,0.0));
        }
    }
    if(TEST_FUSION) {
        Meshing::TriMesh mesh;
        Meshing::MakeTriSphere(8,16,mesh);
        for(size_t i=0;i<mesh.verts.size();i++) {
            mesh.verts[i] *= 0.25;
            mesh.verts[i].z += 1.5;
        }
        // mesh.verts.resize(3);
        // mesh.tris.resize(1);
        // mesh.verts[0] = Vector3(0,-0.7,1.0);
        // mesh.verts[1] = Vector3(0.5,-0.7,2.0);
        // mesh.verts[2] = Vector3(0,-0.25,2.0);
        // mesh.tris[0].set(0,1,2);
        hm.FuseMesh(mesh);
    }
    hm.viewport.pose.t.z += 0.0;
    // printf("Focal lengths %f %f\n",hm.viewport.fx,hm.viewport.fy);
    AnyGeometry3D geom(hm);
    auto bb = geom.GetAABB();
    cout<<"Bounding box "<<bb<<endl;
    
    if(TEST_CONVERSIONS) {
        //mesh test
        AnyGeometry3D gmesh;
        if(!geom.Convert(AnyGeometry3D::Type::TriangleMesh,gmesh)) {
            printf("Can't convert to mesh?\n");
        }
        auto bb_m = gmesh.GetAABB();
        cout<<"Mesh bounding box "<<bb_m<<endl;

        //PC test
        Meshing::PointCloud3D pc;
        hm.GetPointCloud(pc,false);
        AnyGeometry3D geompc(pc);
        auto bb_pc = geompc.GetAABB();
        cout<<"PC bounding box "<<bb_pc<<endl;

        AnyGeometry3D gvox;
        if(!geom.Convert(AnyGeometry3D::Type::ImplicitSurface,gvox,0.0))
            printf("Can't convert to implicit surface?\n");
    }        
    Klampt::WorldModel world;
    world.AddTerrain("terrain",new Klampt::TerrainModel());
    world.terrains[0]->geometry.CreateEmpty();
    *world.terrains[0]->geometry = geom;
    world.terrains[0]->geometry.OnGeometryChange();
    world.terrains[0]->geometry.Appearance()->silhouetteRadius = 0;
    world.terrains[0]->geometry.Appearance()->creaseAngle = 0;
    // world.terrains[0]->geometry.Appearance()->edgeColor.setBlack();
    // world.terrains[0]->geometry.Appearance()->drawEdges = true;

    Klampt::WorldViewProgram gui(&world);
    gui.Run();
}


int main(int argc,const char** argv)
{
    HeightmapTest();
    return 0;
}