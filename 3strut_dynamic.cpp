/*
tensegrity.cpp: 3-Strut Tensegrity Simulation.

*** I used your favorite C/C++ braces style, John! ***
*/

#include <math.h>
#include <vector>

#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

#include "tensegrity/components/dynamic/Strut.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

#define DENSITY (0.5)
#define FORCE_K (0.005)
#define PI (3.14159265)

// WORLD
dWorldID world;
dSpaceID space;
dGeomID  ground;
dJointGroupID contactgroup;

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

// void drawStrut(Strut strut) {
//     dReal *color = strut.color;
//     dsSetColor(color[0], color[1], color[2]);
//
//     dsDrawCapsule(dBodyGetPosition(strut.body),
//         dBodyGetRotation(strut.body), strut.length, strut.radius);
// }

// Simulation loop
void simLoop (int pause) {
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]
    dJointGroupEmpty (contactgroup);

    // dBodySetForce(capsule.body, 0, 0, 0);
    // dBodySetForce(capsule2.body, 0, 0, 0);
    //
    //
    // // ----- APPLY EDGE FORCES (SIMULATE SPRINGS) -----
    // addForce(capsule, capsule2, capsule_one_top, capsule_two_top, 1, 1);
    //
    // // Struts (Capsule 1 - Blue; 2 - Green; 3 - Red)
    // drawStrut(capsule);
    // drawStrut(capsule2);
    // drawStrut(capsule3);
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
    fn.path_to_textures = "/opt/ode-0.13/drawstuff/textures"; //path to the texture

    dInitODE(); // Initialize ODE
    world = dWorldCreate(); // Create a dynamic world

    dWorldSetGravity (world, 0, 0, -0.2);
    dWorldSetDamping (world, 0.05, 0.05);

    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // ----------------------------------------

    Strut strut(world, space);

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
