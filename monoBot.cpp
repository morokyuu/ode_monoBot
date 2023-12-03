#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

// dynamics and collision objects
dWorldID world;
dSpaceID space;
dGeomID ground;
dJointGroupID contactgroup;

dsFunctions fn;

typedef struct{
  dBodyID body;
  dGeomID geom;
  dReal l,r,m;
} MyObject;

MyObject body1,body2,body3,leg1,leg2;
dJointID joint1,joint2;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10;
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode = dContactBounce;
      contact[i].surface.bounce = 1.0;
      contact[i].surface.bounce_vel = 0.0;
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
                     dGeomGetBody(contact[i].geom.g2));
    }
  }
}


// simulation loop
static void simLoop(int pause)
{
  // find collisions and add contact joints
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.001);
  // remove all contact joints
  dJointGroupEmpty(contactgroup);

  dsSetColor(1.0,0.0,0.0);
  dsDrawCylinderD(dBodyGetPosition(body1.body),
                dBodyGetRotation(body1.body), body1.l, body1.r);

  dsDrawCapsuleD(dBodyGetPosition(leg1.body),
                 dBodyGetRotation(leg1.body), leg1.l, leg1.r);

  dsDrawCylinderD(dBodyGetPosition(body2.body),
                dBodyGetRotation(body2.body), body1.l, body2.r);

//  dsDrawCapsuleD(dBodyGetPosition(leg2.body),
//                 dBodyGetRotation(leg2.body), leg2.l, leg2.r);
}




void makeMonoBot()
{
  dMass mass;
  dReal x0 = 0, y0 = 0, z0 = 2.5;

  //body1
  body1.r = 0.2;
  body1.m = 1.0;
  body1.l = 0.3;
  body1.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass,body1.m,3,body1.r,body1.l);
  dBodySetMass(body1.body,&mass);
  dBodySetPosition(body1.body, x0, y0, z0);
  body1.geom = dCreateSphere(space,body1.r);
  dGeomSetBody(body1.geom,body1.body);

  //leg
  leg1.r = 0.025;
  leg1.m = 0.001;
  leg1.l = 0.2;
  leg1.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,leg1.m,3,leg1.r,leg1.l);
  dBodySetMass(leg1.body,&mass);
  dBodySetPosition(leg1.body, x0, y0, z0-body1.r-0.5*leg1.l);
  leg1.geom = dCreateCapsule(space,leg1.r,leg1.l);
  dGeomSetBody(leg1.geom,leg1.body);

  //body2
  body2.r = 0.2;
  body2.m = 1.0;
  body2.l = 0.3;
  body2.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass,body2.m,3,body2.r,body1.l);
  dBodySetMass(body2.body,&mass);
  dBodySetPosition(body2.body, x0, y0, z0-body1.r-leg1.l);
  body2.geom = dCreateSphere(space,body2.r);
  dGeomSetBody(body2.geom,body2.body);

  //leg
  leg2.r = 0.025;
  leg2.m = 0.001;
  leg2.l = 0.2;
  leg2.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,leg2.m,3,leg2.r,leg2.l);
  dBodySetMass(leg2.body,&mass);
  dBodySetPosition(leg2.body, x0, y0,
          body1.r-0.5*leg1.l + z0-body2.r-0.5*leg2.l);
  leg2.geom = dCreateCapsule(space,leg2.r,leg2.l);
  dGeomSetBody(leg2.geom,leg2.body);

  //joint
  joint1 = dJointCreateHinge(world, 0);
  dJointAttach(joint1, body1.body,leg1.body);
  dJointSetHingeAnchor(joint1,x0,y0,z0-body1.r);
  dJointSetHingeAxis(joint1,1,0,0);
  dJointSetHingeParam(joint1, dParamLoStop, -0.25*M_PI);
  dJointSetHingeParam(joint1, dParamHiStop,  0.25*M_PI);

  //joint
  joint2 = dJointCreateHinge(world, 0);
  dJointAttach(joint2, leg1.body,body2.body);
  dJointSetHingeAnchor(joint2,x0,y0,
          z0 - body1.r - leg1.l);
  dJointSetHingeAxis(joint2,1,0,0);
  dJointSetHingeParam(joint2, dParamLoStop, -0.25*M_PI);
  dJointSetHingeParam(joint2, dParamHiStop,  0.25*M_PI);
}


// start simulation - set viewpoint
static void start()
{
   float xyz[3] = {4.0f,-4.0f,1.7600f};
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
   dWorldSetGravity (world,0,0.0,-9.8);
   ground = dCreatePlane (space,0,0,1,0);
   dWorldSetCFM (world,1e-5);

   makeMonoBot();

   // run simulation
   dsSimulationLoop (argc,argv,640,480,&fn);

   // clean up
   dSpaceDestroy (space);
   dWorldDestroy (world);
   dCloseODE();
   return 0;
}


