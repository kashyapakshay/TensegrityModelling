/*
tensegrity.cpp: 3-Strut Tensegrity Simulation.

*** I used your favorite C/C++ braces style, John! ***
*/

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
#define FORCE_K (1.0)
#define PI (3.14159265)

struct Strut {
    dBodyID body; dGeomID geom;

    dMass massObj;

    dReal mass, length, radius;
    dReal x, y, z;
    dMatrix3 rot;

    dReal *color;
};


// WORLD
dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;
int flag = 0;

const float gravity = 0;

int count = 0;
float origin[] = {0, 0, 0};

// OBJECTS
struct Strut capsule, capsule2, capsule3, capsule4, capsule5, capsule6;

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

float getSpringForce(float r, float x, float k) { return k * (x - r); }

static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact;
    contact.surface.mode = dContactMu2;
    // friction parameter
    contact.surface.mu = dInfinity;
    if (int numc = dCollide (o1, o2, 1, &contact.geom,sizeof(dContact))) {
        dJointID c = dJointCreateContact (world,contactgroup,&contact);
        dJointAttach (c,b1,b2);
    }
}

void addForce(Strut c1, Strut c2, dVector3 cp1, dVector3 cp2, int d1, int d2) {
    float pdist, vdist[3];
    float force;

    pdist = getDist(cp1, cp2);
    force = getSpringForce(0.2, pdist, 0.001);
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

Strut createStrut(dSpaceID space, dWorldID world, dVector3 coords, dVector3 angles, dReal mass,
    dReal length, dReal radius, dVector3 color) {

    Strut strut;

    // Create: Body
    strut.body = dBodyCreate(world);

    // Set: (x, y, z); Angle
    strut.x = coords[0];
    strut.y = coords[1];
    strut.z = coords[2];

    dRFromEulerAngles(strut.rot, angles[0], angles[1], angles[2]);

    // Set: Mass
    strut.mass = mass;

    // Set: Radius; Length
    strut.length = length;
    strut.radius = radius;

    // Create: Geom
    strut.geom = dCreateCapsule(space, strut.radius, strut.length);

    // Set: color
    strut.color = color;

    // Calculate and Initalize Mass object
    dMassSetZero(&(strut.massObj));
    dMassSetCapsule(&(strut.massObj), DENSITY, 3, strut.radius, strut.length);
    dBodySetMass(strut.body, &(strut.massObj));

    // Set Configuration
    dBodySetPosition(strut.body, strut.x, strut.y, strut.z);
    dBodySetRotation(strut.body, strut.rot);

    dGeomSetBody(strut.geom, strut.body);

    return strut;
}

// Simulation loop
void simLoop (int pause) {
    count++;

    dSpaceCollide (space, 0, &nearCallback);
    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]
    dJointGroupEmpty(contactgroup);

    dBodySetForce(capsule.body, 0, 0, 0);
    dBodySetForce(capsule2.body, 0, 0, 0);
    dBodySetForce(capsule3.body, 0, 0, 0);
    dBodySetForce(capsule4.body, 0, 0, 0);
    dBodySetForce(capsule5.body, 0, 0, 0);
    dBodySetForce(capsule6.body, 0, 0, 0);

    // dBodySetTorque(capsule.body, 0, 0, 0);
    // dBodySetTorque(capsule2.body, 0, 0, 0);
    // dBodySetTorque(capsule3.body, 0, 0, 0);
    // dBodySetTorque(capsule4.body, 0, 0, 0);
    // dBodySetTorque(capsule5.body, 0, 0, 0);
    // dBodySetTorque(capsule6.body, 0, 0, 0);

    // dBodyAddTorque(capsule.body, 0, 0, 0.00002);
    // dBodyAddTorque(capsule2.body, 0, 0, 0.00002);
    // dBodyAddTorque(capsule3.body, 0, 0, 0.00001);

    // Simulate motor vibration
    if (flag == 0) {
        dBodyAddForceAtRelPos(capsule.body,
            0.0005, 0, 0,
            0, 0, capsule.length/2);

        dBodyAddForceAtRelPos(capsule.body,
            -0.0005, 0, 0,
            0, 0, -capsule.length/2);

        dBodyAddForceAtRelPos(capsule4.body,
            0.0005, 0, 0,
            0, 0, capsule4.length/2);

        dBodyAddForceAtRelPos(capsule4.body,
            -0.0005, 0, 0,
            0, 0, -capsule4.length/2);

        flag = 1;
    } else {
        dBodyAddForceAtRelPos(capsule.body,
            -0.0005, 0, 0,
            0, 0, capsule.length/2);

        dBodyAddForceAtRelPos(capsule.body,
            0.0005, 0, 0,
            0, 0, -capsule.length/2);

        dBodyAddForceAtRelPos(capsule4.body,
            -0.0005, 0, 0,
            0, 0, capsule4.length/2);

        dBodyAddForceAtRelPos(capsule4.body,
            0.0005, 0, 0,
            0, 0, -capsule4.length/2);

        flag = 0;
    }

    // ----- EDGES -----
    dVector3 capsule_one_top, capsule_one_bottom;
    dVector3 capsule_two_top, capsule_two_bottom;
    dVector3 capsule_three_top, capsule_three_bottom;
    dVector3 capsule_four_top, capsule_four_bottom;
    dVector3 capsule_five_top, capsule_five_bottom;
    dVector3 capsule_six_top, capsule_six_bottom;

    dBodyGetRelPointPos(capsule.body, 0, 0, capsule.length/2, capsule_one_top);
    dBodyGetRelPointPos(capsule.body, 0, 0, -capsule.length/2, capsule_one_bottom);

    dBodyGetRelPointPos(capsule2.body, 0, 0, capsule2.length/2, capsule_two_top);
    dBodyGetRelPointPos(capsule2.body, 0, 0, -capsule2.length/2, capsule_two_bottom);

    dBodyGetRelPointPos(capsule3.body, 0, 0, capsule3.length/2, capsule_three_top);
    dBodyGetRelPointPos(capsule3.body, 0, 0, -capsule3.length/2, capsule_three_bottom);

    dBodyGetRelPointPos(capsule4.body, 0, 0, capsule4.length/2, capsule_four_top);
    dBodyGetRelPointPos(capsule4.body, 0, 0, -capsule4.length/2, capsule_four_bottom);

    dBodyGetRelPointPos(capsule5.body, 0, 0, capsule5.length/2, capsule_five_top);
    dBodyGetRelPointPos(capsule5.body, 0, 0, -capsule5.length/2, capsule_five_bottom);

    dBodyGetRelPointPos(capsule6.body, 0, 0, capsule6.length/2, capsule_six_top);
    dBodyGetRelPointPos(capsule6.body, 0, 0, -capsule6.length/2, capsule_six_bottom);

    // ----- APPLY EDGE FORCES (SIMULATE SPRINGS) -----
    addForce(capsule, capsule3, capsule_one_top, capsule_three_top, 1, 1);
    addForce(capsule2, capsule3, capsule_two_top, capsule_three_top, 1, 1);
    addForce(capsule, capsule3, capsule_one_top, capsule_three_bottom, 1, -1);
    addForce(capsule2, capsule3, capsule_two_top, capsule_three_bottom, 1, -1);

    addForce(capsule, capsule5, capsule_one_top, capsule_five_top, 1, 1);
    addForce(capsule3, capsule5, capsule_three_top, capsule_five_top, 1, 1);
    addForce(capsule, capsule5, capsule_one_bottom, capsule_five_top, -1, 1);
    addForce(capsule4, capsule5, capsule_four_top, capsule_five_top, 1, 1);

    addForce(capsule2, capsule5, capsule_two_top, capsule_five_bottom, 1, -1);
    addForce(capsule4, capsule5, capsule_four_top, capsule_five_bottom, 1, -1);
    addForce(capsule2, capsule5, capsule_two_bottom, capsule_five_bottom, -1, -1);
    addForce(capsule3, capsule5, capsule_three_top, capsule_five_bottom, 1, -1);

    addForce(capsule, capsule4, capsule_one_bottom, capsule_four_top, -1, 1);
    addForce(capsule2, capsule4, capsule_two_bottom, capsule_four_top, -1, 1);
    addForce(capsule, capsule4, capsule_one_bottom, capsule_four_bottom, -1, -1);
    addForce(capsule2, capsule4, capsule_two_bottom, capsule_four_bottom, -1, -1);

    addForce(capsule, capsule6, capsule_one_top, capsule_six_top, 1, 1);
    addForce(capsule, capsule6, capsule_one_bottom, capsule_six_top, -1, 1);
    addForce(capsule2, capsule6, capsule_two_top, capsule_six_bottom, 1, -1);
    addForce(capsule2, capsule6, capsule_two_bottom, capsule_six_bottom, -1, -1);

    addForce(capsule3, capsule6, capsule_three_bottom, capsule_six_top, -1, 1);
    addForce(capsule4, capsule6, capsule_four_bottom, capsule_six_top, -1, 1);
    addForce(capsule4, capsule6, capsule_four_bottom, capsule_six_bottom, -1, -1);
    addForce(capsule3, capsule6, capsule_three_bottom, capsule_six_bottom, -1, -1);

    // ----- DRAW -----
    // Springs
    dsDrawLine(capsule_one_top, capsule_three_top);
    dsDrawLine(capsule_two_top, capsule_three_top);
    dsDrawLine(capsule_one_top, capsule_three_bottom);
    dsDrawLine(capsule_two_top, capsule_three_bottom);

    dsDrawLine(capsule_one_top, capsule_five_top);
    dsDrawLine(capsule_three_top, capsule_five_top);
    dsDrawLine(capsule_one_bottom, capsule_five_top);
    dsDrawLine(capsule_four_top, capsule_five_top);

    dsDrawLine(capsule_two_top, capsule_five_bottom);
    dsDrawLine(capsule_four_top, capsule_five_bottom);
    dsDrawLine(capsule_two_bottom, capsule_five_bottom);
    dsDrawLine(capsule_three_top, capsule_five_bottom);

    dsDrawLine(capsule_one_bottom, capsule_four_top);
    dsDrawLine(capsule_two_bottom, capsule_four_top);
    dsDrawLine(capsule_one_bottom, capsule_four_bottom);
    dsDrawLine(capsule_two_bottom, capsule_four_bottom);

    dsDrawLine(capsule_one_top, capsule_six_top);
    dsDrawLine(capsule_one_bottom, capsule_six_top);
    dsDrawLine(capsule_two_top, capsule_six_bottom);
    dsDrawLine(capsule_two_bottom, capsule_six_bottom);

    dsDrawLine(capsule_three_bottom, capsule_six_top);
    dsDrawLine(capsule_four_bottom, capsule_six_top);
    dsDrawLine(capsule_four_bottom, capsule_six_bottom);
    dsDrawLine(capsule_three_bottom, capsule_six_bottom);

    // Struts (Capsule 1 - Blue; 2 - Green; 3 - Red)
    drawStrut(capsule);
    drawStrut(capsule2);
    drawStrut(capsule3);
    drawStrut(capsule4);
    drawStrut(capsule5);
    drawStrut(capsule6);
}

// Start function void start()
void start() {
    // Set a camera
    static float xyz[3] = {2.0, 0, 1};     // View position (x, y, z [m])
    static float hpr[3] = {180, 0, 0};    // View direction head, pitch, roll[]

    dsSetViewpoint (xyz, hpr);// Set a view point
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

    dWorldSetGravity (world, 0, 0, -0.05);
    dWorldSetDamping (world, 0.05, 0.05);

    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // ----- CREATE STRUTS -----
    // Global Strut Props
    dReal mass = 0.1, length = 1.0, radius = 0.02;

    // --- Capsule 1/2: Blue ---
    dVector3 coords1 = {0.0, 0.2, 1.0};
    dVector3 angles1 = {0.0, 0.0, 0.0};
    dVector3 color1 = {0.0, 0.0, 1.0};
    capsule = createStrut(space, world, coords1, angles1, mass, length, radius, color1);

    dVector3 coords2 = {0.0, -0.2, 1.0};
    capsule2 = createStrut(space, world, coords2, angles1, mass, length, radius, color1);

    // --- Capsule 3/4: Green ---
    dVector3 coords3 = {0.0, 0.0, 1.2};
    dVector3 angles3 = {0.0, -PI/2, 0.0};
    dVector3 color3 = {0.0, 1.0, 0.0};
    capsule3 = createStrut(space, world, coords3, angles3, mass, length, radius, color3);

    dVector3 coords4 = {0.0, 0.0, 0.8};
    capsule4 = createStrut(space, world, coords4, angles3, mass, length, radius, color3);

    // --- Capsule 5/6: Red ---
    dVector3 coords5 = {0.2, 0.0, 1.0};
    dVector3 angles5 = {PI/2, 0.0, 0.0};
    dVector3 color5 = {1.0, 0.0, 0.0};
    capsule5 = createStrut(space, world, coords5, angles5, mass, length, radius, color5);

    dVector3 coords6 = {-0.2, 0.0, 1.0};
    capsule6 = createStrut(space, world, coords6, angles5, mass, length, radius, color5);

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
