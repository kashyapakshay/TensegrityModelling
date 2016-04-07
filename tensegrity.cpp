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
int count = 0;
float origin[] = {0, 0, 0};

// OBJECTS
struct Strut capsule, capsule2;

float getVector(float p1[3], float p2[3]) {
    float vector[] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};

    return *vector;
}

float getDist(float p1[3], float p2[3]) {
    float dist = sqrt(pow(p1[0] - p2[0], 2) +
                    pow(p1[1] - p2[1], 2) +
                    pow(p1[2] - p2[2], 2));

    return dist;
}

float getHorizontal(float l, float theta) {return l * cos(theta);}
float getVertical(float l, float theta)   {return l * sin(theta);}

float getForce(float x, float k) {return k*x;}

// Simulation loop
void simLoop (int pause)
{
    count++;

    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]

    //const dReal *rot = dBodyGetRotation(capsule.body);
    //float angle = atan2(rot[9], rot[10]);

    dVector3 vect;
    dBodyGetRelPointPos(capsule.body, 0, 0, capsule.length/2, vect);
    dVector3 vect2;
    dBodyGetRelPointPos(capsule2.body, 0, 0, capsule2.length/2, vect2);

    float p2[] = {vect2[0], vect2[1], vect2[2]};
    float p1[] = {vect[0], vect[1], vect[2]};

    dsDrawLine(p2, p1);

    printf("CAPSULE [1]: (%f, %f, %f)\n", p1[0], p1[1], p1[2]);
    printf("CAPSULE [2]: (%f, %f, %f)\n", p2[0], p2[1], p2[2]);
    printf("\n");

    // --- Forces ---

    // ----- DRAW -----

    // Capsule 1 - Blue

    // Set color (red, green, blue) value is from 0 to 1.0
    dReal *color = capsule.color;
    dsSetColor(color[0], color[1], color[2]);

    dsDrawCapsule(dBodyGetPosition(capsule.body),
        dBodyGetRotation(capsule.body), capsule.length, capsule.radius);


    // Capsule 2 - Green

    // Set color (red, green, blue) value is from 0 to 1.0
    dReal *color2 = capsule2.color;
    dsSetColor(color2[0], color2[1], color2[2]);

    dsDrawCapsule(dBodyGetPosition(capsule2.body),
        dBodyGetRotation(capsule2.body), capsule2.length, capsule2.radius);
}

// Start function void start()
void start()
{
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
    fn.path_to_textures = "../../drawstuff/textures"; //path to the texture

    dInitODE(); // Initialize ODE
    world = dWorldCreate(); // Create a dynamic world
    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);

    dWorldSetGravity(world, 0, 0, gravity);// Set gravity (x, y, z)

    // --- Capsule 1: Blue ---

    // Create: Body
    capsule.body = dBodyCreate(world);

    // Set: (x, y, z); Angle
    capsule.x = 0;
    capsule.y = 0;
    capsule.z = 1;
    //dQFromAxisAndAngle(q, 1, 0, 0, PI/4);
    //dQtoR(q, capsule.rot);
    dRFromEulerAngles(capsule.rot, PI/4, 0, 0);

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
    capsule2.y = 0;
    capsule2.z = 1;
    //dQFromAxisAndAngle(q, 1, 0, 0, PI/4);
    //dQtoR(q, capsule.rot);
    dRFromEulerAngles(capsule2.rot, PI/6, 0, 0);

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

    // ----------------------------------------

    // Simulation loop
    // argc, argv are argument of main function.
    // fn is a structure of drawstuff
    dsSimulationLoop(argc, argv, 1024, 800, &fn);

    dWorldDestroy(world);// Destroy the world
    dCloseODE();                                 // Close ODE

    return 0;
}
