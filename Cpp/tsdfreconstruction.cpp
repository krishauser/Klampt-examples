#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLRenderToImage.h>
#include <KrisLibrary/image/image.h>
#include <KrisLibrary/image/ppm.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/geometry/TSDFReconstruction.h>
#include <Klampt/Modeling/World.h>
#include <GL/gl.h>
#include <fstream>
#include <thread>

using namespace GLDraw;
using namespace Geometry;
using namespace Meshing;
using namespace Klampt;

const static Real DEPTH_MAX = 4.0;
const static bool DO_REGISTRATION = true;

#define SPARSE 1
#if SPARSE
  #define RECONSTRUCTION_METHOD SparseTSDFReconstruction
#else
  #define RECONSTRUCTION_METHOD DenseTSDFReconstruction
#endif

class MyProgram: public GLUTNavigationProgram
{
public:
    WorldModel world;
    WorldModel scans;
    WorldModel tsdf;
    int numSavedFrames;
    int toDraw;
    shared_ptr<RECONSTRUCTION_METHOD> reconstruction;
    RigidTransform lastFrameTransform;
    vector<pair<Vector3,Vector3> > correspondences;

    MyProgram() {
        //set up OpenGL viewport and lighting parameters
        viewport.n = 0.1;
        viewport.f = 20.0;
        camera.dist = 2.0;

        world.lights.resize(1);
        world.lights[0].setColor(GLColor(1,1,1));
        world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));

        toDraw = 0;
        numSavedFrames = 0;

        ClearReconstruction();
    }
    virtual void SetWorldLights() override {
        world.SetGLLights();
        glClearColor(world.background.rgba[0],world.background.rgba[1],world.background.rgba[2],world.background.rgba[3]);
    }
    virtual void RenderWorld() override 
    {   
        //draw each scan's camera frame
        glDisable(GL_LIGHTING);
        drawCoords(0.1);
        for(size_t i=0;i<scans.rigidObjects.size();i++) {
            glPushMatrix();
            glMultMatrix(Matrix4(scans.rigidObjects[i]->T));
            drawCoords(0.2);
            glPopMatrix();
        }
        //draw the world, scans, or reconstructed world depennding 
        glEnable(GL_LIGHTING);
        if(toDraw == 0)
            world.DrawGL();
        else if(toDraw == 1)
            scans.DrawGL();
        else {
            tsdf.DrawGL();
            //draw boxes
            #if SPARSE
            glDisable(GL_LIGHTING);
            glColor3f(1,1,0);
            for(auto i : reconstruction->tsdf.hash.buckets) {
                AABB3D bb = reinterpret_cast<SparseVolumeGrid::Block*>(i.second)->grid.channels[0].bb;
                drawWireBoundingBox(bb.bmin,bb.bmax);
            }
            /*
            //draw points
            glPointSize(3.0);
            glBegin(GL_POINTS);
            glColor3f(0,0,0);
            for(auto i : reconstruction->tsdf.hash.buckets) {
                VolumeGridTemplate<float>& vg = reinterpret_cast<SparseVolumeGrid::Block*>(i.second)->grid.channels[0];
                for(auto i=vg.getIterator();!i.isDone();++i) {
                    if(Abs(*i) < reconstruction->truncationDistance*0.5) {
                        Vector3 c;
                        i.getCellCenter(c);
                        glVertex3v(c);
                    }
                }
            }
            glEnd();
            */
            #endif
        }
        /* //draw ICP correspondences
        if(!correspondences.empty()) {
            glDisable(GL_LIGHTING);
            glDisable(GL_DEPTH_TEST);
            glBegin(GL_LINES);
            for(size_t i=0;i<correspondences.size();i++) {
                glColor3f(0,0,0);
                glVertex3v(correspondences[i].first);
                glColor3f(1,1,1);
                glVertex3v(correspondences[i].second);
            }
            glEnd();
            glEnable(GL_DEPTH_TEST);
        }
        */
    }
    virtual void Handle_Keypress(unsigned char key,int x,int y)
    {
        if(key == 's') {
            GLRenderToImage renderer;
            if(!renderer.Setup(viewport.w,viewport.h)) {
                printf("Can't setup render-to-image?"); 
            }
            else {
                //save current viewport's color frame, depth frame, and camera transform to disk
                numSavedFrames += 1;
                
                renderer.Begin(viewport);
                SetWorldLights();
                world.DrawGL();
                renderer.End();
                Image color,depth;
                renderer.GetRGBA(color);
                renderer.GetDepth(viewport,depth);
                Image color_rgb,depth_a8;
                color_rgb.initialize(color.w,color.h,Image::R8G8B8);
                color.blit(color_rgb);
                depth_a8.initialize(color.w,color.h,Image::A8);
                //scale the depth to have a max of 1
                int k=0;
                for(int i=0;i<depth.w;i++)
                    for(int j=0;j<depth.h;j++,k+=4) {
                        float& d = *reinterpret_cast<float*>(&depth.data[k]);
                        d = Min(1.0f,d/float(DEPTH_MAX));
                    }
                depth.blit(depth_a8);
                char buf[256],buf2[256],buf3[256];
                sprintf(buf,"color%04d.ppm",numSavedFrames);
                sprintf(buf2,"depth%04d.ppm",numSavedFrames);
                sprintf(buf3,"xform%04d.txt",numSavedFrames);
                WritePPM_RGB_ASCII(color_rgb.data,color.w,color.h,buf);
                WritePPM_Grayscale_ASCII(depth_a8.data,color.w,color.h,buf2);
                //openGL has y up and z backward, flip y and z to get standard camera xform
                Matrix3 flipYZ;
                flipYZ.setZero();
                flipYZ(0,0) = 1;
                flipYZ(1,1) = -1;
                flipYZ(2,2) = -1;
                RigidTransform Tcamera;
                Tcamera.R = viewport.xform.R*flipYZ;
                Tcamera.t = viewport.xform.t;
                ofstream out(buf3,ios::out);
                out<<Tcamera<<endl;
                out.close();
                printf("Saved to %s, %s, and %s\n",buf,buf2,buf3);
            }
        }
        else if(key == 'l') {
            //load frams from disk
            numSavedFrames += 1;
            char buf[256],buf2[256],buf3[256];
            sprintf(buf,"color%04d.ppm",numSavedFrames);
            sprintf(buf2,"depth%04d.ppm",numSavedFrames);
            sprintf(buf3,"xform%04d.txt",numSavedFrames);
            
            RigidTransform T;
            PointCloud3D pc;
            unsigned char* color_data;
            unsigned char* depth_data;
            int w,h;
            int dw,dh;
            if(!ReadPPM_RGB(&color_data,&w,&h,buf)) {
                printf("Done reading images.\n");
                return;
            }
            if(!ReadPPM_Grayscale(&depth_data,&dw,&dh,buf2)) {
                printf("Error reading depth image?.\n");
                delete [] color_data;
                return;
            }
            if(w != dw || h != dh) {
                printf("Hmm... unequal size images? %d x %d vs %d x %d \n",w,h,dw,dh);
                delete [] color_data;
                delete [] depth_data;
                return;
            }
            if(w != viewport.w || h != viewport.h) {
                printf("Warning: wrong viewport size?\n");
            }
            vector<unsigned int> color(w*h);
            vector<float> depth(w*h);
            for(int i=0;i<w*h;i++) {
                color[i] = color_data[i*3] << 16 | color_data[i*3+1] << 8 | color_data[i*3+2];
                depth[i] = depth_data[i] / 255.0*DEPTH_MAX;
                if(depth_data[i]==0)
                    depth[i] = DEPTH_MAX;
            }
            delete [] color_data;
            delete [] depth_data;
            Real wfov = viewport.getFOV();
            Real hfov = viewport.getHFOV();
            Timer pctimer;
            pc.FromDepthImage(w,h,wfov,hfov,depth,color,DEPTH_MAX);
            printf("Time to convert from depth image %gs\n",pctimer.ElapsedTime());

            bool estimateTransform = false;
            ifstream in(buf3);
            if(!in) {
                printf("Could not read camera transform\n");
                estimateTransform = true;
            }
            else {
                in >> T;
            }

            //add to reconstruction. Register then fuse.
            reconstruction->NewScan();
            if(numSavedFrames > 1 && (estimateTransform || DO_REGISTRATION)) {
                printf("=============================================================\n");
                Timer timer;
                ICPParameters params;
                reconstruction->Register(pc,lastFrameTransform,params);
                printf("ICP computed in time %g, with %d inliers, rmse %f\n",timer.ElapsedTime(),params.numInliers,params.rmseDistance);
                if(!estimateTransform) {
                    Matrix3 Rt;
                    Rt.mulTransposeB(T.R,params.Tcamera.R);
                    MomentRotation m;
                    m.setMatrix(Rt);
                    printf("  Ground truth distance %f translation, %f rotation\n",T.t.distance(params.Tcamera.t),m.norm());
                }
                cout<<"  Standard error "<<params.standardError<<endl;
                T = params.Tcamera;
                correspondences = params.correspondences;
                for(size_t i=0;i<correspondences.size();i++)
                    correspondences[i].first = params.Tcamera*correspondences[i].first;
            }
            printf("=============================================================\n");
            Timer timer;
            reconstruction->Fuse(T,pc);
            printf("Fuse time %gs, memory usage %d MB\n",timer.ElapsedTime(),reconstruction->MemoryUsage()/(1024*1024));
            lastFrameTransform = T;

            //create a new rigid object to store the scan
            RigidObjectModel* obj = new RigidObjectModel;
            ManagedGeometry::GeometryPtr geom = obj->geometry.CreateEmpty();
            *geom = pc;
            obj->T = T;
            obj->UpdateGeometry();
            obj->geometry.OnGeometryChange();
            obj->geometry.Appearance()->silhouetteRadius = 0;
            obj->geometry.Appearance()->creaseAngle = 0;
            
            stringstream ss;
            ss<<"Scan "<<scans.rigidObjects.size()+1;
            scans.AddRigidObject(ss.str(),obj);                
            cout<<"Adding "<<ss.str()<<endl;

            UpdateMesh();
            Refresh();
        }
        else if(key == ' ') {
            //add viewport to scans without saving to disk
            GLRenderToImage renderer;
            Timer timer;
            printf("0: Setting up renderer\n");
            if(!renderer.Setup(viewport.w,viewport.h)) {
                printf("Can't setup render-to-image?"); 
            }
            else {
                printf("%f: Rendering world\n",timer.ElapsedTime());
                renderer.Begin(viewport);
                SetWorldLights();
                world.DrawGL();
                renderer.End();
                vector<unsigned int> color;
                vector<float> depth;
                renderer.GetRGBA(color);
                renderer.GetDepth(viewport,depth);
                //cap the depth to 1
                for(size_t i=0;i<depth.size();i++)
                    depth[i] = Min(depth[i],float(DEPTH_MAX));
     
                //create a new rigid object to store the scan
                RigidObjectModel* obj = new RigidObjectModel;
                ManagedGeometry::GeometryPtr geom = obj->geometry.CreateEmpty();
            
                printf("%f: Creating PointCloud3D\n",timer.ElapsedTime());

                //create the point cloud
                *geom = PointCloud3D();
                PointCloud3D & pc = geom->AsPointCloud();
                Real wfov = viewport.getFOV();
                Real hfov = viewport.getHFOV();
                pc.FromDepthImage(viewport.w,viewport.h,wfov,hfov,depth,color,DEPTH_MAX);
                //pc.SavePCL("test.pcd");
                //printf("%f: OnGeometryChange\n",timer.ElapsedTime());
                //obj->geometry.OnGeometryChange();

                stringstream ss;
                ss<<"Scan "<<scans.rigidObjects.size()+1;
                scans.AddRigidObject(ss.str(),obj);                
                cout<<"Adding "<<ss.str()<<endl;

                Matrix3 flipYZ;
                flipYZ.setZero();
                flipYZ(0,0) = 1;
                flipYZ(1,1) = -1;
                flipYZ(2,2) = -1;
                obj->T.R = viewport.xform.R*flipYZ;
                obj->T.t = viewport.xform.t;
                printf("%f: Updating geometry and appearance\n",timer.ElapsedTime());
                obj->UpdateGeometry();
                obj->geometry.OnGeometryChange();
                obj->geometry.Appearance()->silhouetteRadius = 0;
                obj->geometry.Appearance()->creaseAngle = 0;

                printf("%f: Done.\n",timer.ElapsedTime());
                Refresh();
            }
        }
        else if(key == 'r') {
            //perform reconstruction from scratch
            ClearReconstruction();

            for(size_t i=0;i<scans.rigidObjects.size();i++) {
                reconstruction->NewScan();
                RigidTransform Ti = scans.rigidObjects[i]->T;
                const PointCloud3D& pc = scans.rigidObjects[i]->geometry->AsPointCloud();
                if(i > 0 && DO_REGISTRATION) {
                    printf("=============================================================\n");
                    Timer timer;
                    ICPParameters params;
                    reconstruction->Register(pc,Ti,params);
                    printf("ICP computed in time %g, with %d inliers, rmse %f\n",timer.ElapsedTime(),params.numInliers,params.rmseDistance);
                    Matrix3 Rt;
                    Rt.mulTransposeB(Ti.R,params.Tcamera.R);
                    MomentRotation m;
                    m.setMatrix(Rt);
                    printf("  Ground truth distance %f translation, %f rotation\n",Ti.t.distance(params.Tcamera.t),m.norm());
                    cout<<"  Standard error "<<params.standardError<<endl;
                    Ti = params.Tcamera;

                    correspondences = params.correspondences;
                    for(size_t c=0;c<correspondences.size();c++)
                        correspondences[c].first = params.Tcamera*correspondences[c].first;
                }
                printf("=============================================================\n");
                Timer timer;
                reconstruction->Fuse(Ti,pc);
                printf("Fuse time %gs, memory usage %d MB\n",timer.ElapsedTime(),reconstruction->MemoryUsage()/(1024*1024));
            }
            UpdateMesh();
            Refresh();
        }
        else if(key == 'd') {
            //toggle item to draw
            toDraw = (toDraw+1)%3;
            Refresh();
        }
    }
    void ClearReconstruction()
    {
        AABB3D volume;
        volume.bmin.set(-2,-2,-0.2);
        volume.bmax.set(2,2,1.8);
        IntTriple res(200,200,200);
        #if SPARSE
          reconstruction.reset(new SparseTSDFReconstruction(Vector3(0.02),0.08));
          reconstruction->numThreads = std::thread::hardware_concurrency();
        #else
        //4 / 200 = 0.02, truncation range 0.1 => average 5 blocks traversed (approximately multiply this by 2 )
          reconstruction.reset(new DenseTSDFReconstruction(volume,res,0.08));
        #endif
        lastFrameTransform.setIdentity();
    }
    void UpdateMesh()
    {
        //update the TSDF extracted mesh
        if(tsdf.rigidObjects.size() == 0) {
            RigidObjectModel* obj = new RigidObjectModel;
            obj->geometry.CreateEmpty();
            *obj->geometry = TriMesh();
            //rigid objects are by default drawn creased and with a silhouette, but this adds significantly to the draw time.
            obj->geometry.Appearance()->silhouetteRadius = 0;
            obj->geometry.Appearance()->creaseAngle = 0;
            tsdf.AddRigidObject("tsdf",obj);
        }
        TriMesh& mesh = tsdf.rigidObjects[0]->geometry->AsTriangleMesh();
        Timer timer;
        //reconstruction->ExtractMesh(mesh); // < extracts an uncolored mesh
        reconstruction->ExtractMesh(mesh,*tsdf.rigidObjects[0]->geometry.Appearance());
        printf("Mesh built in %fs, %d vertices and %d triangles\n",timer.ElapsedTime(),mesh.verts.size(),mesh.tris.size());
        AABB3D bb;
        mesh.GetAABB(bb.bmin,bb.bmax);
        cout<<"Bounds "<<bb.bmin<<"  ->  "<<bb.bmax<<endl;
        tsdf.rigidObjects[0]->geometry.OnGeometryChange();
        //tsdf.rigidObjects[0]->geometry.Appearance()->faceColor.set(0,1,0,0.5);
    }
};

int main(int argc,const char** argv)
{
    printf("Usage: TSDFReconstruction [world XML]\n");
    if(argc <= 1) {
        return 1;
    }
    MyProgram program;
    if(!program.world.LoadXML(argv[1])) {
        printf("Error loading file %s\n",argv[1]);
        return 1;
    }
    printf("Controls:\n");
    printf(" - [space]: take a snapshot from the camera's perspective\n");
    printf(" - r: perform reconstruction from prior snapshots\n");
    printf(" - s: save a snapshot to disk\n");
    printf(" - l: load prior snapshots from disk and perform reconstruction\n");
    printf(" - d: toggle between drawing world, scans, or reconstruction\n");

    program.Run(); 
    return 0;
}