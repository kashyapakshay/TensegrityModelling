#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#include <math.h>
#include <vector>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define DENSITY (0.5)
#define FORCE_K (0.005)
#define PI (3.14159265)

dMatrix3 mat, mat2;
dQuaternion q, q2;
float a1 = PI/2;
float a2 = 0;

struct Object {
    dBodyID body;
    dGeomID geom;
    dReal radius = 0.05, mass = 1.0, length = 1.0;
    dMass massObj;
    dReal x = 0.0, y = 0.0, z = 0.0;
    dMatrix3 rot;
    dReal color[3] = {1.0, 1.0, 1.0};
};

const bool collisionOn = false;
const float gravity = 0;

// WORLD
static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static int count = 0;

// OBJECTS
struct Object capsule, capsule2;

static dJointGroupID contactgroup;

// METHODS
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    static int MAX_CONTACTS = 10;
    int i;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnected (b1,b2)) return;

    dContact contact[MAX_CONTACTS];
    int numc = dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom, sizeof(dContact));
    if (numc > 0) {
      for (i=0; i < numc; i++) {
        contact[i].surface.mode  =  dContactSoftCFM | dContactSoftERP;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.soft_cfm = 1e-8;
        contact[i].surface.soft_erp = 1.0;
        dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
        dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
            dGeomGetBody(contact[i].geom.g2));
     }
   }
}

/*static std::vector<int> getVector(int p1[3], int p2[3]) {
    std::vector<int> vector;
    vector.push_back(p1[0]);

    return vector;
}*/

static float* getVector(float p1[3], float p2[3]) {
    float vector[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    //printf("func: %f %f %f\n", vector[0], vector[1], vector[2]);

    return vector;
}

static float getDist(float p1[3], float p2[3]) {
    float dist = sqrt(pow(p1[0] - p2[0], 2) +
                    pow(p1[1] - p2[1], 2) +
                    pow(p1[2] - p2[2], 2));

    return dist;
}

static float getHorizontal(float l, float theta) {return l * cos(theta);}
static float getVertical(float l, float theta)   {return l * sin(theta);}

static float getForce(float x, float k) {return k*x;}

// Simulation loop
static void simLoop (int pause)
{
    count++;

    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]
    dJointGroupEmpty(contactgroup);

    // ----- VECT -----

    /*float p1[3] = {capsule.x + capsule.length/2,
                    capsule.y + capsule.length/2,
                    capsule.z + capsule.length/2};

    float p2[3] = {capsule2.x + capsule2.length/2,
                    capsule2.y + capsule2.length/2,
                    capsule2.z + capsule2.length/2};*/

    //float* vect = getVector(p1, p2);

    dQuaternion qg, qg2;
    dRtoQ(dBodyGetRotation(capsule.body), qg);
    dRtoQ(dBodyGetRotation(capsule2.body), qg2);

//    printf("1: %f %f %f %f\n", PI/2 - 2*acos(qg[0]), PI/2 - 2*asin(qg[1]), PI/2 - 2*asin(qg[2]), PI/2 - 2*asin(qg[3]));
//    printf("2: %f %f %f %f\n", qg2[0], qg2[1], qg2[2], qg2[3]);

    dQuaternion qt, qt2;
    dRtoQ(dBodyGetRotation(capsule.body), qt);
    dRtoQ(dBodyGetRotation(capsule2.body), qt2);
    float angle = 2*asin(qt[1]);
    float angle2 = 2*asin(qt2[1]);
    float angle_y = PI/2 - angle;
    float angle2_y = PI/2 - angle2;

    float p1[3] = {
        capsule.x,
        capsule.y - getHorizontal(capsule.length/2, angle_y),
        capsule.z + getVertical(capsule.length/2, angle_y)
    };
    float p2[3] = {
        capsule2.x,
        capsule2.y - getHorizontal(capsule2.length/2, angle2_y),
        capsule2.z + getVertical(capsule2.length/2, angle2_y)
    };

    printf("CAPSULE [1]: (%f, %f, %f)\n", p1[0], p1[1], p1[2]);
    printf("CAPSULE [2]: (%f, %f, %f)\n", p2[0], p2[1], p2[2]);

    printf("DIST: %f\n", getDist(p1, p2));
    printf("ANGLE: %f %f\n", angle_y, angle2_y);

    printf("\n");

    // ----- DRAW -----

    // Capsule 1 - Red
    dsSetColor(1.0, 0.0, 0.0); // Set color (red, green, blue) value is from 0 to 1.0

    dsDrawCapsule(dBodyGetPosition(capsule.body),
        dBodyGetRotation(capsule.body), capsule.length, capsule.radius);

    // Capsule 2 - White
    dsSetColor(1.0, 1.0, 1.0);

    dsDrawCapsule(dBodyGetPosition(capsule2.body),
        dBodyGetRotation(capsule2.body), capsule2.length, capsule2.radius);

    dsDrawLine(p1, p2);

    /*dBodyAddForceAtPos(
        capsule.body,
        0, 0, 0.0001,
        p1[0], p1[1], p1[2]
    );

    dBodyAddForceAtPos(
        capsule.body,
        0, 0, -0.0001,
        p1[0], p1[1] - capsule.length/2, p1[2]
    );*/

    /*dBodyAddForceAtRelPos(
        capsule.body,
        0, 0, 0.0001,
        0, -capsule.length/2, 0
    );*/

    if(count % 200 == 0) {
        dQuaternion qr, qrs;
        dRtoQ(dBodyGetRotation(capsule.body), qr);
        float angler = 2*asin(qr[1]) + PI/6;
        dQFromAxisAndAngle(qrs, 1, 0, 0, angler);
        dQtoR(qrs, capsule.rot);
        dBodySetRotation(capsule.body, capsule.rot);
    }

    dBodyAddForceAtPos(
        capsule.body,
        (p2[0]-p1[0])*FORCE_K, (p2[1]-p1[1])*FORCE_K, (p2[2]-p1[2])*FORCE_K,
        p1[0], p1[1], p1[2]
    );

    dBodyAddForceAtPos(
        capsule.body,
        (p2[0]-p1[0])*FORCE_K, (p2[1]-p1[1])*FORCE_K, (p2[2]-p1[2])*FORCE_K,
        p1[0], p1[1] - capsule.length/2, p1[2]
    );

    dBodyAddForceAtPos(
        capsule.body,
        -(p2[0]-p1[0])*FORCE_K, -(p2[1]-p1[1])*FORCE_K, -(p2[2]-p1[2])*FORCE_K,
        p1[0], p1[1], p1[2]
    );

    dBodyAddForceAtPos(
        capsule.body,
        -(p2[0]-p1[0])*FORCE_K, -(p2[1]-p1[1])*FORCE_K, -(p2[2]-p1[2])*FORCE_K,
        p1[0], p1[1] - capsule.length/2, p1[2]
    );

    dBodyAddForceAtPos(
        capsule2.body,
        (p1[0]-p2[0])*FORCE_K, (p1[1]-p2[1])*FORCE_K, (p1[2]-p2[2])*FORCE_K,
        p2[0], p2[1], p2[2]
    );

    dBodyAddForceAtPos(
        capsule2.body,
        (p1[0]-p2[0])*FORCE_K, (p1[1]-p2[1])*FORCE_K, (p1[2]-p2[2])*FORCE_K,
        p2[0], p2[1], p2[2] - capsule2.length/2
    );

    dBodyAddForceAtPos(
        capsule2.body,
        -(p1[0]-p2[0])*FORCE_K, -(p1[1]-p2[1])*FORCE_K, -(p1[2]-p2[2])*FORCE_K,
        p2[0], p2[1], p2[2]
    );

    dBodyAddForceAtPos(
        capsule2.body,
        -(p1[0]-p2[0])*FORCE_K, -(p1[1]-p2[1])*FORCE_K, -(p1[2]-p2[2])*FORCE_K,
        p2[0], p2[1], p2[2] - capsule2.length/2
    );

    /*dBodyAddForceAtRelPos(capsule2.body, 0, 0, -0.001, 0, 0, capsule2.length/2);
    dBodyAddForceAtRelPos(capsule2.body, 0, 0.001, 0, 0, 0, capsule2.length/2);
    dBodyAddForceAtRelPos(capsule2.body, 0, -0.001, 0, 0, 0, capsule2.length/2);

    dBodyAddForceAtRelPos(capsule2.body, 0, 0, 0.001, 0, 0, -1*capsule2.length/2);
    dBodyAddForceAtRelPos(capsule2.body, 0, 0.001, 0, 0, 0, -1*capsule2.length/2);
    dBodyAddForceAtRelPos(capsule2.body, 0, -0.001, 0, 0, 0, -1*capsule2.length/2);*/
}

// Start function void start()
static void start()
{
    // Set a camera
    static float xyz[3] = {2.0,0.0,1.0};     // View position (x, y, z [m])
    static float hpr[3] = {180,0.0,0.0};    // View direction head, pitch, roll[]

    dsSetViewpoint (xyz,hpr);// Set a view point
}

// main function
int main (int argc, char **argv) {
    // for drawstuff
    dsFunctions fn; //drawstuff structure
    fn.version = DS_VERSION;    // the version of the drawstuff
    fn.start = &start;            // start function
    fn.step = &simLoop;               // step function
    fn.command = NULL;     // no command function for keyboard
    fn.stop    = NULL;         // no stop function
    fn.path_to_textures = "../../drawstuff/textures";//path to the texture

    dInitODE();            // Initialize ODE
    world = dWorldCreate();    // Create a dynamic world
    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space,0,0,1,0);

    if(collisionOn)
        dSpaceCollide(space,0, &nearCallback);

    dWorldSetGravity(world, 0, 0, gravity);// Set gravity (x, y, z)

    // --- Capsule 1: RED ---

    dQFromAxisAndAngle(q, 1, 0, 0, a1);
    //float theta = 0;
    //float thetaQuart = PI/2 - theta;

    //const dQuaternion qx = {cos(thetaQuart/2), sin(thetaQuart/2), 0, 0};
    dQtoR(q, capsule.rot);

    capsule.body = dBodyCreate(world);
    capsule.x = 0;
    capsule.z = 1;
    capsule.geom = dCreateCapsule(space, capsule.radius, capsule.length);

    dMassSetZero(&(capsule.massObj));
    dMassSetCapsule(&(capsule.massObj), DENSITY, 3, capsule.radius,
        capsule.length);
    dBodySetMass(capsule.body, &(capsule.massObj));
    dBodySetPosition(capsule.body, capsule.x, capsule.y, capsule.z);
    dBodySetRotation(capsule.body, capsule.rot);
    dGeomSetBody(capsule.geom, capsule.body);
    //dBodySetForce(capsule.body, -0.01, 0, 0);

    // --- Capsule 2: WHITE ---

    dQFromAxisAndAngle(q2, 1, 0, 0, a2);
    dQtoR(q2, mat2);

    capsule2.body = dBodyCreate(world);
    capsule2.x = 0;
    capsule2.z = 1;
    capsule2.geom = dCreateCapsule(space, capsule2.radius, capsule2.length);

    dMassSetZero(&(capsule2.massObj));
    dMassSetCapsule(&(capsule2.massObj), DENSITY, 3, capsule2.radius,
        capsule2.length);
    dBodySetMass(capsule2.body, &(capsule2.massObj));
    dBodySetPosition(capsule2.body, capsule2.x, capsule2.y, capsule2.z);
    dBodySetRotation(capsule2.body, mat2);
    dGeomSetBody(capsule2.geom, capsule2.body);

    // +++ JOINT +++

    contactgroup = dJointGroupCreate(2);

    dJointID joint = dJointCreateBall(world, 0);
    dJointAttach(joint, capsule.body, capsule2.body);
    dJointSetBallAnchor(joint, capsule.x, capsule.y, capsule.z);
    //dJointSetHingeAxis(joint, 1, 0, 0);

    // Simulation loop
    // argc, argv are argument of main function.
    // fn is a structure of drawstuff
    dsSimulationLoop (argc,argv,1024,800,&fn);

    dWorldDestroy(world);// Destroy the world
    dCloseODE();                                 // Close ODE

    return 0;
}
