#include <ode/ode.h>
#include <drawstuff/drawstuff.h>



// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;

dsFunctions fn;

typedef struct{
  dBodyID body;
  dGeomID geom;
  dReal l,r,m;
} MyObject;
MyObject ball, leg;

dJointID joint;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
   static const int N = 10;
   dContact contact[N];

   int isGround = ((ground == o1) || (ground == o2));

   int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
   if(isGround){
      for(int i=0;i<n;i++){
         //contact[i].surface.mu = dInfinity;
         contact[i].surface.mode = dContactBounce;
         contact[i].surface.bounce = 1.0;
         contact[i].surface.bounce_vel = 0.0;

         dJointID c = dJointCreateContact(world,contactgroup, &contact[i]);
         dJointAttach(c,
                      dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
      }
   }
}


// simulation loop
static void simLoop (int pause)
{
   // find collisions and add contact joints
   dSpaceCollide (space,0,&nearCallback);
   // step the simulation
   //dWorldQuickStep (world,0.001);
   dWorldStep (world,0.001);
   // remove all contact joints
   dJointGroupEmpty (contactgroup);

   dsSetColor(1,0,1);
   dsDrawSphereD(dBodyGetPosition(ball.body),
                 dBodyGetRotation(ball.body), ball.r);

   dsDrawCapsuleD(dBodyGetPosition(leg.body),
                  dBodyGetRotation(leg.body), leg.l, leg.r);

   getchar();
}




void makeMonoBot()
{
    dMass mass;
    dReal x0 = 0, y0 = 0, z0 = 2.5;

    //body
    ball.r = 0.2;
    ball.m = 1;
    ball.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass,ball.m,ball.r);
    dBodySetMass(ball.body,&mass);
    dBodySetPosition(ball.body,x0,y0,z0);
    ball.geom = dCreateSphere(space,ball.r);
    dGeomSetBody(ball.geom, ball.body);

    //leg
    leg.r = 0.025;
    leg.m = 0.01;
    leg.l = 1;
    leg.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,leg.m,3,leg.r,leg.l);
    dBodySetMass(leg.body,&mass);
    dBodySetPosition(leg.body, x0, y0, z0-ball.r-0.5*leg.l);
    leg.geom = dCreateCapsule(space,leg.r,leg.l);
    dGeomSetBody(leg.geom,leg.body);

    //joint
    joint = dJointCreateHinge(world,0);
    dJointAttach(joint,ball.body,leg.body);
    dJointSetHingeAnchor(joint,x0,y0,z0-ball.r);
    dJointSetHingeAxis(joint,1,0,0);
    dJointSetHingeParam(joint,dParamLoStop,-0.25*M_PI);
    dJointSetHingeParam(joint,dParamLoStop,0.25*M_PI);
}


// start simulation - set viewpoint
static void start()
{
   float xyz[3] = {2.0f,-2.0f,1.7600f};
   float hpr[3] = {140.000f,-17.0000f,0.0000f};
   dsSetViewpoint (xyz,hpr);
}


setDrawStuff()
{
   // setup pointers to drawstuff callback functions
   fn.version = DS_VERSION;
   fn.start = &start;
   fn.step = &simLoop;
   fn.stop = 0;
   fn.command = 0;
   fn.path_to_textures = "../../drawstuff/textures";

}

int main (int argc, char **argv)
{
   dInitODE ();
   setDrawStuff();

   // create world
   world = dWorldCreate ();
   space = dHashSpaceCreate (0);
   contactgroup = dJointGroupCreate (0);
   dWorldSetGravity (world,0,0,-0.2);
   ground = dCreatePlane (space,0,0,1,0);
   dWorldSetCFM (world,1e-5);

   makeMonoBot();

   // run simulation
   dsSimulationLoop (argc,argv,352,288,&fn);

   // clean up
   dSpaceDestroy (space);
   dWorldDestroy (world);
   dCloseODE();
   return 0;
}


