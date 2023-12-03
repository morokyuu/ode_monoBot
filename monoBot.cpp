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

static MyObject torso, leg[2];
static dJointID h_joint,s_joint;
static int STEPS = 0;

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

// ロボットの描画
static void drawMonoBot()
{
  const dReal *pos1,*R1,*pos2,*R2;

  // 胴体部(球)の描画
  dsSetColor(1.0,0.0,0.0);                       // 赤色
  pos1 = dBodyGetPosition(torso.body);
  R1   = dBodyGetRotation(torso.body);
  dsDrawSphereD(pos1,R1,torso.r);

  // 脚部(カプセル）の描画
  for (int i = 0; i < 2; i++) {
    pos2 = dBodyGetPosition(leg[i].body);
    R2   = dBodyGetRotation(leg[i].body);
    if (i == 0) {
      dsSetColor(0.0,0.0,1.0);                    // 青色
      dsDrawCylinderD(pos2,R2,leg[i].l,leg[i].r);
    }
    else {
      dsSetColor(1.2,1.2,1.2);                   // 白色
      dsDrawCapsuleD(pos2,R2,leg[i].l,leg[i].r);
    }
  }
}


// スライダの制御 プログラム2.4
static void controlSlider(dReal target)
{
  static dReal kp   = 25.0;                       // 比例定数
  static dReal fmax = 400;                        // 最大力[N]

  dReal tmp  = dJointGetSliderPosition(s_joint);  // スライダの現在位置
  dReal u    = kp * (target - tmp);               // 残差

  dJointSetSliderParam(s_joint, dParamVel,  u);
  dJointSetSliderParam(s_joint, dParamFMax, fmax);
}


// simulation loop
static void simLoop(int pause)
{
  int s = 200;
  STEPS++;
  //slider expand
  if ((0 <= (STEPS % s)) && ((STEPS % s) <= 10)){
      controlSlider(0.5);
  }
  else{
      controlSlider(0.0);
  }
  dSpaceCollide(space,0,&nearCallback);
  dWorldStep(world,0.001);
  dJointGroupEmpty(contactgroup);
  drawMonoBot();

  dReal sj = dJointGetSliderPosition(s_joint);
  printf("%d, %f\n",STEPS, sj);
}

//  dsSetColor(1.0,0.0,0.0);
//  dsDrawCylinderD(dBodyGetPosition(body1.body),
//                dBodyGetRotation(body1.body), body1.l, body1.r);
//
//  dsDrawCapsuleD(dBodyGetPosition(leg1.body),
//                 dBodyGetRotation(leg1.body), leg1.l, leg1.r);
//
//  dsDrawCylinderD(dBodyGetPosition(body2.body),
//                dBodyGetRotation(body2.body), body1.l, body2.r);
//
//  dsDrawCapsuleD(dBodyGetPosition(leg2.body),
//                 dBodyGetRotation(leg2.body), leg2.l, leg2.r);




void createMonoBot()
{
  dMass mass;
  dReal x0 = 0, y0 = 0, z0 = 3.5;

  //torso
  torso.r = 0.25; torso.m = 14.0;
  torso.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,torso.m,torso.r);
  dBodySetMass(torso.body,&mass);
  dBodySetPosition(torso.body, x0, y0, z0);
  torso.geom = dCreateSphere(space,torso.r);
  dGeomSetBody(torso.geom,torso.body);

  //leg
  leg[0].l = 0.75;  leg[1].l = 0.75;    // 長さ
  leg[0].r = 0.05;  leg[1].r = 0.03;    // 半径
  for (int i = 0; i < 2; i++) {
    leg[i].m   = 3.0;
    leg[i].body   = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,leg[i].m,3,leg[i].r,leg[i].l);
    dBodySetMass(leg[i].body,&mass);
    if (i == 0)
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l);
    else
      dBodySetPosition(leg[i].body, x0, y0, z0-0.5*leg[0].l-0.5);
    leg[i].geom = dCreateCapsule(space,leg[i].r,leg[i].l);
    dGeomSetBody(leg[i].geom,leg[i].body);
  }

  // ヒンジジョイント
  h_joint = dJointCreateHinge(world, 0);
  dJointAttach(h_joint, torso.body,leg[0].body);
  dJointSetHingeAnchor(h_joint, x0, y0, z0);
  dJointSetHingeAxis(h_joint, 1, 0, 0);

  // スライダージョイント
  s_joint = dJointCreateSlider(world, 0);
  dJointAttach(s_joint, leg[0].body,leg[1].body);
  dJointSetSliderAxis(s_joint, 0, 0, 1);
  dJointSetSliderParam(s_joint, dParamLoStop, -0.25);
  dJointSetSliderParam(s_joint, dParamHiStop,  0.25);
}

static void start()
{
   float xyz[3] = {3.5f,0.0f,1.0f};
   float hpr[3] = {-180.000f,0.0f,0.0f};
   dsSetViewpoint (xyz,hpr);
   dsSetSphereQuality(3);
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

   createMonoBot();

   // run simulation
   dsSimulationLoop (argc,argv,640,480,&fn);

   // clean up
   dSpaceDestroy (space);
   dWorldDestroy (world);
   dCloseODE();
   return 0;
}


