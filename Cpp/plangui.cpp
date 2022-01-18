#include <Klampt/Planning/RobotCSpace.h>
#include <Klampt/Interface/WorldViewProgram.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <Klampt/IO/XmlWorld.h>
#include <Klampt/Modeling/Paths.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <GL/glut.h>
#include <string.h>
#include <time.h>
#include <fstream>
using namespace Klampt;

struct KlamptPlannerProgram : public WorldViewProgram
{
  WorldModel world;
  ViewRobot vr;
  WorldPlannerSettings settings;
  Config start,goal;
  MilestonePath path;
  MotionPlannerFactory factory;
  shared_ptr<SingleRobotCSpace> cspace;
  shared_ptr<MotionPlannerInterface> planner;
  Real cumulativeTime;
  vector<int> ees;
  bool drawRoadmap;

  KlamptPlannerProgram()
    :WorldViewProgram(&world)
  {
    world.lights.resize(1);
    world.lights[0].setColor(GLColor(1,1,1));
    world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
    world.lights[0].setColor(GLColor(1,1,1));
    drawRoadmap = true;
  }

  bool LoadProblem(const char* worldfile,const char* configsfile)
  {
    XmlWorld xmlWorld;
    if(!xmlWorld.Load(worldfile)) return false;
    xmlWorld.GetWorld(world);
    settings.InitializeDefault(world);

    //get end effectors for visualization
    RobotModel* robot = world.robots[0].get();
    vector<vector<int> > children;
    robot->GetChildList(children);
    //delete welded joints
    for(size_t i=0;i<robot->joints.size();i++) {
      if(robot->joints[i].type == RobotModelJoint::Weld) {
	int link = robot->joints[i].linkIndex;
	int p=robot->parents[link];
	if(p < 0) continue;
	children[p].erase(find(children[p].begin(),children[p].end(),(int)i));
	//mark as being non-ee
	children[i].push_back(-1);
      }
    }
    for(size_t i=0;i<children.size();i++) 
      if(children[i].empty()) ees.push_back(i);


    cspace = make_shared<SingleRobotCSpace>(world,0,&settings); 

    //Read in the configurations specified in configsfile
    vector<Config> configs;
    ifstream in(configsfile);
    if(!in) {
      printf("Error opening configs file %s\n",configsfile);
      return false;
    }
    while(in) {
      Config temp;
      in >> temp;
      if(in) configs.push_back(temp);
    }
    if(configs.size() < 2) {
      printf("Configs file does not contain 2 or more configs\n");
      return false;
    }
    start=configs[0];
    goal=configs[1];
    if(start.n != world.robots[0]->q.n) {
      printf("Incorrect size on start configuration\n");
      return false;
    }
    if(goal.n != world.robots[0]->q.n) {
      printf("Incorrect size on goal configuration\n");
      return false;
    }
    return true;
  }

  bool LoadPlannerSettings(const string& settingsString) 
  {
    if(!settingsString.empty()) {
      bool res = factory.LoadJSON(settingsString);
      if(!res) 
	printf("Warning, incorrectly formatted planner settings file\n");
      return res;
    }
    return false;
  }

  virtual bool Initialize() {
    if(!WorldViewProgram::Initialize()) return false;
    InitPlanner();
    return true;
  }

  void InitPlanner()
  {
    planner.reset(factory.Create(cspace.get(),start,goal));
    cumulativeTime = 0;
    path.edges.clear();
  }

  string PlanAll(MilestonePath& path,HaltingCondition& cond)
  {
    string res = planner->Plan(path,cond);
    cout<<"Planner result \""<<res<<"\",";
    if(!path.edges.empty()) 
      cout<<" path length "<<path.Length()<<",";
    cout<<" stats: ";
    PropertyMap stats;
    planner->GetStats(stats);
    stats.Print(cout);
    cout<<endl;
    return res;
  }

  void PlanStep()
  {
    planner->PlanMore();
  }

  void PlanMore(int num=100,FILE* csvout=stdout)
  {
    Timer timer;
    for(int i=0;i<num;i++) 
      PlanStep();
    cumulativeTime += timer.ElapsedTime();

    if(csvout == NULL) return;

    //extract minimum cost path
    bool hasPath = planner->IsSolved();
    if(hasPath) {
      planner->GetSolution(path);
      /*
      //TODO: fix this problem
      if(planner.spp.epsilon == 0)
	Assert(path.IsFeasible());
      */
    }

    Real cost = Inf;
    if(!path.edges.empty()) cost = path.Length();
    PropertyMap stats;
    planner->GetStats(stats);
    if(csvout == stdout) {
      printf("Plan step %d, time %g, resulting cost %g\n",planner->NumIterations(),cumulativeTime,cost);
      cout<<"Stats: ";
      stats.Print(cout);
      cout<<endl;
    }
    else {
      fprintf(csvout,"%d,%g,%g",planner->NumIterations(),cumulativeTime,cost);
      for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++)
	fprintf(csvout,",%s",i->second.c_str());
      fprintf(csvout,"\n");
    }
  }

  void BatchTest(const char* filename,int numTrials=10,int maxIters=100000,Real tmax = 10,Real printIncrement=0.1)
  {
    printf("Batch test, saving to %s:\n",filename);
    FILE* f = NULL;
    f = fopen(filename,"w");
    if(!f) {
      fprintf(stderr,"Unable to open file %s for writing\n",filename);
      return;
    }
    fprintf(f,"trial,plan iters,plan time,best cost");
    PropertyMap stats;
    planner->GetStats(stats);
    for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++)
      fprintf(f,",%s",i->first.c_str());
    fprintf(f,"\n");

    for(int trials=0;trials<numTrials;trials++) {
      InitPlanner();
      Assert(cumulativeTime == 0);
      Timer timer;
      Real lastPrintTime = -Inf;
      for(int i=0;i<maxIters;i++) {
	if(cumulativeTime >= lastPrintTime + printIncrement) {
	  lastPrintTime = timer.ElapsedTime();
	  fprintf(f,"%d,",trials);
	  PlanMore(1,f);
	}
	else 
	  PlanMore(1,NULL);

	if(cumulativeTime > tmax) {
	  printf("Time %g, cumulative time %g\n",timer.ElapsedTime(),cumulativeTime);
	  break;
	}
      }
    }
    fclose(f);
  }


  virtual void RenderWorld() {
    RobotModel* robot = world.robots[0].get();
    robot->UpdateConfig(start);
    ::WorldViewProgram::RenderWorld();

    //draw goal configuration, transparent
    robot->UpdateConfig(goal);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    vr.robot = robot;
    vr.SetColors(GLColor(1,0,0,0.5));
    vr.Draw();

    //render path
    if(!path.edges.empty()) {
      Real dt = 0.05;
      Real len = path.Length();
      //draw end effector path
      glDisable(GL_LIGHTING);
      glColor3f(1,1,0);
      glLineWidth(2.0);
      glBegin(GL_LINES);
      Config q;
      int numSteps = (int)Ceil(len / dt);
      for(int i=0;i<=numSteps*2-1;i+=2) {
	Real t1=Real(i)/Real(2*numSteps);
	Real t2=Real(i+1)/Real(2*numSteps);
	if(t2 > 1) t2 = 1;
	path.Eval(t1,q);
	robot->UpdateConfig(q);
	vector<Vector3> eepos(ees.size());
	for(size_t e=0;e<ees.size();e++)
	  eepos[e] = robot->links[ees[e]].T_World.t;
	path.Eval(t2,q);
	robot->UpdateConfig(q);
	for(size_t e=0;e<ees.size();e++) {
	  glVertex3v(eepos[e]);
	  glVertex3v(robot->links[ees[e]].T_World.t);
	}
      }
      glEnd();
    }

    // render roadmap
    if(drawRoadmap) {
      glDisable(GL_LIGHTING);
      glColor3f(1,0.5,0);
      glLineWidth(1.0);
      RoadmapPlanner roadmap(cspace.get());
      planner->GetRoadmap(roadmap.roadmap);
      Config q;
      for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {  
	Graph::EdgeIterator<shared_ptr<EdgePlanner> > e;
	for(roadmap.roadmap.Begin(i,e);!e.end();e++) {
	  Real dt = 0.2;
	  Real len = cspace->Distance((*e)->Start(),(*e)->End());
	  //draw end effector path
	  for(size_t k=0;k<ees.size();k++) {
	    glBegin(GL_LINE_STRIP);
	    int numSteps = (int)Ceil(len / dt);
	    for(int i=0;i<=numSteps;i++) {
	      Real t=Real(i)/Real(numSteps);
	      (*e)->Eval(t,q);
	      robot->UpdateConfig(q);
	      glVertex3v(robot->links[ees[k]].T_World.t);
	    }
	    glEnd();
	  }
	}
      }
    }
  }
  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == '1') {
      PlanMore(1);
    }
    else if(key == ' ') {
      PlanMore();
    }
    else if(key == 'r') { 
      path.edges.clear();
    }
    else if(key == 't') {
      string outputfile = string("results/")+factory.type+string("csv");
      BatchTest(outputfile.c_str());
    }
    else if(key == 'd') {
      drawRoadmap = !drawRoadmap;
    }
    else {
      printf("Commands:\n");
      printf("- 1: do one step of planning\n");
      printf("- [space]: do 100 steps of planning\n");
      printf("- r: clear the current path\n");
      printf("- t: perform batch testing\n");
      printf("- d: toggle drawing the roadmap\n");
    }
    Refresh();
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
      PlanMore();
      Refresh();
    }
    else WorldViewProgram::Handle_Click(button,state,x,y);
  }
};



int main(int argc,const char** argv)
{
  if(argc <= 2) {
    printf("USAGE: PlanGUI [options] world_file configs_file\n");
    printf("OPTIONS:\n");
    printf("-o filename: the output path (default none)\n");
    printf("-p settings: set the planner configuration file\n");
    printf("-v: visualize the planner rather than just plan\n");
    printf("-opt: do optimal planning (do not terminate on the first found solution)\n");
    printf("-n iters: set the default number of iterations (default 1000)\n");
    printf("-t time: set the planning time limit (default infinity)\n");
    printf("-b n statsfile: batch test with n iterations, save planning data\n     to the given file (CSV format)\n");
    printf("-m margin: use the given collision avoidance margin\n");
    printf("-r res: use the given collision checking resolution (default 0.01)\n");
    return 0;
  }
  Srand(time(NULL));
  const char* outputfile = NULL;
  HaltingCondition termCond;
  string plannerSettings;
  bool visualize = false;
  int batchSize = 0;
  Real collisionResolution = 0, collisionMargin = 0;
  int i;
  //parse command line arguments
  for(i=1;i<argc;i++) {
    if(argv[i][0]=='-') {
      if(0==strcmp(argv[i],"-n")) {
	termCond.maxIters = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-t")) {
	termCond.timeLimit = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-opt")) {
	termCond.foundSolution = false;
      }
      else if(0==strcmp(argv[i],"-p")) {
	if(!GetFileContents(argv[i+1],plannerSettings)) {
	  printf("Unable to load planner settings file %s\n",argv[i+1]);
	  return 1;
	}
	i++;
      }
      else if(0==strcmp(argv[i],"-o")) {
	outputfile = argv[i+1];
	i++;
      }
      else if(0==strcmp(argv[i],"-v")) {
	visualize = true;
      }
      else if(0==strcmp(argv[i],"-b")) {
	batchSize = atoi(argv[i+1]);
	outputfile = argv[i+2];
	i += 2;
      }
      else if(0==strcmp(argv[i],"-m")) {
	collisionMargin = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-r")) {
	collisionResolution = atof(argv[i+1]);
	i++;
      }
      else {
	printf("Invalid option %s\n",argv[i]);
	return 1;
      }
    }
    else break;
  }
  if(i+2 < argc) {
    printf("Too few arguments provided\n");
    printf("USAGE: PlanGUI [options] worldfile configsfile\n");
    return 1;
  }
  if(i+2 > argc) {
    printf("Warning: extra arguments provided\n");
  }
  const char* worldfile = argv[i];
  const char* configsfile = argv[i+1];

  KlamptPlannerProgram program;
  program.LoadPlannerSettings(plannerSettings);
  if(!program.LoadProblem(worldfile,configsfile)) {
    printf("Error reading problem from %s and %s\n",worldfile,configsfile);
    return 1;
  }
  if(collisionResolution > 0)
    program.settings.robotSettings[0].collisionEpsilon = collisionResolution;
  if(collisionMargin > 0) {
    for(auto g:program.world.robots[0]->geometry)
      g->margin += collisionMargin;
  }
  if(visualize) {
    string name = string("Motion Planner (")+program.factory.type+string(")");
    return program.Run(name.c_str());
  }
  else {
    //just plan on command line
    program.InitPlanner();
    if(batchSize > 0) {
      program.BatchTest(outputfile,batchSize,termCond.maxIters,termCond.timeLimit);
    }
    else {
      //single plan, output the path
      MilestonePath path;
      program.PlanAll(path,termCond);
      //save to disk
      if(!path.edges.empty() && outputfile != NULL) {
	cout<<"Saving result to "<<outputfile<<endl;
	ofstream f(outputfile,ios::out);
	for(int i=0;i<path.NumMilestones();i++) {
	  f << i<< "\t" << path.GetMilestone(i)<< endl;
	}
	f.close();
      }
    }
  }
  return 0;
}
