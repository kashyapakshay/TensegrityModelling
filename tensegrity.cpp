#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#include <math.h>
#include <vector>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

#define DENSITY (0.5)
#define FORCE_K (0.005)
#define PI (3.14159265)

struct Strut {
    dBodyID body; dGeomID geom;

    dMass massObj;

    dReal radius, mass, length;
    dReal x, y, z;
    dMatrix3 rot;

    dReal *color;
};

const float gravity = 0;

// WORLD
dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;
int count = 0;
float origin[] = {0, 0, 0};

// OBJECTS
struct Strut capsule, capsule2, capsule3;

float getVector(float vect[3], float vect2[3]) {
    float vector[] = {vect2[0] - vect[0], vect2[1] - vect[1], vect2[2] - vect[2]};

    return *vector;
}

float getDist(dReal vect[3], dReal vect2[3]) {
    float dist = sqrt(pow(vect[0] - vect2[0], 2) +
                    pow(vect[1] - vect2[1], 2) +
                    pow(vect[2] - vect2[2], 2));

    return dist;
}

float getHorizontal(float l, float theta) { return l * cos(theta); }
float getVertical(float l, float theta)   { return l * sin(theta); }

float getForce(float r, float x, float k) { return k * (x - r); }

static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact;
    contact.surface.mode = dContactMu2;
    // friction parameter
    contact.surface.mu = dInfinity;
    if (int numc = dCollide (o1,o2,1,&contact.geom,sizeof(dContact))) {
        dJointID c = dJointCreateContact (world,contactgroup,&contact);
        dJointAttach (c,b1,b2);
    }
}

void addForce(Strut c1, Strut c2, dVector3 cp1, dVector3 cp2, int d1, int d2) {
    float pdist, vdist[3];
    float force;

    pdist = getDist(cp1, cp2);
    force = getForce(0.2, pdist, 0.001);
    vdist[0] = (cp2[0] - cp1[0]);
    vdist[1] = (cp2[1] - cp1[1]);
    vdist[2] = (cp2[2] - cp1[2]);

    dBodyAddForceAtRelPos(c1.body,
        vdist[0] * force / pdist,
        vdist[1] * force / pdist,
        vdist[2] * force / pdist,
        0, 0, d1 * c1.length/2);

    dBodyAddForceAtRelPos(c2.body,
        -vdist[0] * force / pdist,
        -vdist[1] * force / pdist,
        -vdist[2] * force / pdist,
        0, 0, d2 * c2.length/2);
}

void drawStrut(Strut strut) {
    dReal *color = strut.color;
    dsSetColor(color[0], color[1], color[2]);

    dsDrawCapsule(dBodyGetPosition(strut.body),
        dBodyGetRotation(strut.body), strut.length, strut.radius);
}

void createStrut() {
    
}

// Simulation loop
void simLoop (int pause) {
    count++;

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]
    dJointGroupEmpty (contactgroup);

    dBodySetForce(capsule.body, 0, 0, 0);
    dBodySetForce(capsule2.body, 0, 0, 0);

    // ----- FORCES -----

    dVector3 capsule_one_top, capsule_one_bottom;
    dVector3 capsule_two_top, capsule_two_bottom;
    dVector3 capsule_three_top, capsule_three_bottom;

    dBodyGetRelPointPos(capsule.body, 0, 0, capsule.length/2, capsule_one_top);
    dBodyGetRelPointPos(capsule.body, 0, 0, -capsule.length/2, capsule_one_bottom);

    dBodyGetRelPointPos(capsule2.body, 0, 0, capsule2.length/2, capsule_two_top);
    dBodyGetRelPointPos(capsule2.body, 0, 0, -capsule2.length/2, capsule_two_bottom);

    dBodyGetRelPointPos(capsule3.body, 0, 0, capsule3.length/2, capsule_three_top);
    dBodyGetRelPointPos(capsule3.body, 0, 0, -capsule3.length/2, capsule_three_bottom);

    // 1t - 2t
    addForce(capsule, capsule2, capsule_one_top, capsule_two_top, 1, 1);
    // 1t - 3t
    addForce(capsule, capsule3, capsule_one_top, capsule_three_top, 1, 1);
    // 1t - 3b
    addForce(capsule, capsule3, capsule_one_top, capsule_three_bottom, 1, -1);
    // 1b - 2t
    addForce(capsule, capsule2, capsule_one_bottom, capsule_two_top, -1, 1);
    // 1b - 2b
    addForce(capsule, capsule2, capsule_one_bottom, capsule_two_bottom, -1, -1);
    // 1b - 3b
    addForce(capsule, capsule3, capsule_one_bottom, capsule_three_bottom, -1, -1);
    // 2t - 3t
    addForce(capsule2, capsule3, capsule_two_top, capsule_three_top, 1, 1);
    // 2b - 3b
    addForce(capsule2, capsule3, capsule_two_bottom, capsule_three_bottom, -1, -1);
    // 2b - 3t
    addForce(capsule2, capsule3, capsule_two_bottom, capsule_three_top, -1, 1);

    // ----- DRAW -----

    // Springs
    dsDrawLine(capsule_one_top, capsule_two_top);
    dsDrawLine(capsule_one_top, capsule_three_top);
    dsDrawLine(capsule_one_top, capsule_three_bottom);
    dsDrawLine(capsule_one_bottom, capsule_two_top);
    dsDrawLine(capsule_one_bottom, capsule_two_bottom);
    dsDrawLine(capsule_one_bottom, capsule_three_bottom);
    dsDrawLine(capsule_two_top, capsule_three_top);
    dsDrawLine(capsule_two_bottom, capsule_three_bottom);
    dsDrawLine(capsule_two_bottom, capsule_three_top);

    // Struts (Capsule 1 - Blue; 2 - Green; 3 - Red)
    drawStrut(capsule);
    drawStrut(capsule2);
    drawStrut(capsule3);
}

// Start function void start()
void start() {
    // Set a camera
    static float xyz[3] = {2, 0, 1};     // View position (x, y, z [m])
    static float hpr[3] = {180, 0, 0};    // View direction head, pitch, roll[]

    dsSetViewpoint (xyz,hpr);// Set a view point
}

// main function
int main (int argc, char **argv) {

    // for drawstuff
    dsFunctions fn; // drawstuff structure
    fn.version = DS_VERSION;    // the version of the drawstuff
    fn.start = &start;            // start function
    fn.step = &simLoop;               // step function
    fn.command = NULL;     // no command function for keyboard
    fn.stop    = NULL;         // no stop function
    fn.path_to_textures = "/usr/local/include/drawstuff/textures"; //path to the texture

    dInitODE(); // Initialize ODE
    world = dWorldCreate(); // Create a dynamic world

    dWorldSetGravity (world,0,0,-0.2);
    dWorldSetDamping (world, 0.05, 0.05);

    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // --- Capsule 1: Blue ---

    // Create: Body
    capsule.body = dBodyCreate(world);

    // Set: (x, y, z); Angle
    capsule.x = 0;
    capsule.y = 0;
    capsule.z = 1;
    //dQFromAxisAndAngle(q, 1, 0, 0, PI/4);
    //dQtoR(q, capsule.rot);
    dRFromEulerAngles(capsule.rot, 0, 0, 0);

    // Set: Radius; Length
    capsule.radius = 0.02;
    capsule.length = 1;

    // Create: Geom
    capsule.geom = dCreateCapsule(space, capsule.radius, capsule.length);

    // Set: Mass
    capsule.mass = 0.1;

    // Set: color
    dReal color[3] = {0.0, 0.0, 1.0};
    capsule.color = color;

    // Calculate and Initalize Mass object
    dMassSetZero(&(capsule.massObj));
    dMassSetCapsule(&(capsule.massObj), DENSITY, 3, capsule.radius,
        capsule.length);
    dBodySetMass(capsule.body, &(capsule.massObj));

    // Set Configuration
    dBodySetPosition(capsule.body, capsule.x, capsule.y, capsule.z);
    dBodySetRotation(capsule.body, capsule.rot);

    dGeomSetBody(capsule.geom, capsule.body);

    // --- Capsule 2: Green ---

    // Create: Body
    capsule2.body = dBodyCreate(world);

    // Set: (x, y, z); Angle
    capsule2.x = 0;
    capsule2.y = 0.5;
    capsule2.z = 1;
    //dQFromAxisAndAngle(q, 1, 0, 0, PI/4);
    //dQtoR(q, capsule.rot);
    dRFromEulerAngles(capsule2.rot, 0, 0, 0);

    // Set: Radius; Length
    capsule2.radius = 0.02;
    capsule2.length = 1;

    // Create: Geom
    capsule2.geom = dCreateCapsule(space, capsule.radius, capsule.length);

    // Set: Mass
    capsule2.mass = 0.1;

    // Set: color
    dReal color2[3] = {0.0, 1.0, 0.0};
    capsule2.color = color2;

    // Calculate and Initalize Mass object
    dMassSetZero(&(capsule2.massObj));
    dMassSetCapsule(&(capsule2.massObj), DENSITY, 3, capsule2.radius,
        capsule2.length);
    dBodySetMass(capsule2.body, &(capsule2.massObj));

    // Set Configuration
    dBodySetPosition(capsule2.body, capsule2.x, capsule2.y, capsule2.z);
    dBodySetRotation(capsule2.body, capsule2.rot);

    dGeomSetBody(capsule2.geom, capsule2.body);

    // --- Capsule 3: Red ---

    // Create: Body
    capsule3.body = dBodyCreate(world);

    // Set: (x, y, z); Angle
    capsule3.x = 0.6;
    capsule3.y = 0.1;
    capsule3.z = 1;
    //dQFromAxisAndAngle(q, 1, 0, 0, PI/4);
    //dQtoR(q, capsule.rot);
    dRFromEulerAngles(capsule2.rot, PI/4, 0, 0);

    // Set: Radius; Length
    capsule3.radius = 0.02;
    capsule3.length = 1;

    // Create: Geom
    capsule3.geom = dCreateCapsule(space, capsule3.radius, capsule3.length);

    // Set: Mass
    capsule3.mass = 0.1;

    // Set: color
    dReal color3[3] = {1.0, 0.0, 0.0};
    capsule3.color = color3;

    // Calculate and Initalize Mass object
    dMassSetZero(&(capsule3.massObj));
    dMassSetCapsule(&(capsule3.massObj), DENSITY, 3, capsule3.radius,
        capsule3.length);
    dBodySetMass(capsule3.body, &(capsule3.massObj));

    // Set Configuration
    dBodySetPosition(capsule3.body, capsule3.x, capsule3.y, capsule3.z);
    dBodySetRotation(capsule3.body, capsule3.rot);

    dGeomSetBody(capsule3.geom, capsule3.body);

    // ----------------------------------------

    // Simulation loop
    // argc, argv are argument of main function.
    // fn is a structure of drawstuff
    dsSimulationLoop(argc, argv, 1024, 800, &fn);

    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();

    return 0;
}
