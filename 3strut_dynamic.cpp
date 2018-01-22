/*
tensegrity.cpp: 3-Strut Tensegrity Simulation.

*** I used your favorite C/C++ braces style, John! ***
*/

#include <math.h>
#include <vector>

#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

#include "tensegrity/components/dynamic/Strut.h"
#include "tensegrity/components/static/Spring.h"

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

Strut *strut_ptr;
Spring *spring_ptr;

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

void drawStrut(Strut *strut) {
    d_vector color = strut->get_color();
    dsSetColor(color[0], color[1], color[2]);

    // strut->draw();
    dsDrawCapsule(dBodyGetPosition(strut->get_body()),
        dBodyGetRotation(strut->get_body()), strut->get_length(), strut->get_radius());
}

void drawSpring(Spring *spring) {
	dVector3 cap_one_point, cap_two_point;
	Strut *strut_1, *strut_2;

	strut_1 = spring->get_strut_one();
	strut_2 = spring->get_strut_two();

	dBodyGetRelPointPos(strut_1->get_body(), 0, 0, strut_1->get_length() / 2, cap_one_point);
	dBodyGetRelPointPos(strut_2->get_body(), 0, 0, strut_2->get_length() / 2, cap_two_point);

	dsDrawLine(cap_one_point, cap_two_point);
}

void handle_one_force(Spring* spr_ptr) {
	d_vector force_vec = spr_ptr->compute_spring_force_vector();
	int dir_1 = spr_ptr->get_edge_one(), dir_2 = spr_ptr->get_edge_two();

	dBodyAddForceAtRelPos(spr_ptr->get_strut_one()->get_body(),
        dir_1 * force_vec[0],
        dir_1 * force_vec[1],
        dir_1 * force_vec[2],
        0, 0, dir_1 * spr_ptr->get_strut_one()->get_length()/2);

    dBodyAddForceAtRelPos(spr_ptr->get_strut_two()->get_body(),
        dir_2 * force_vec[0],
        dir_2 * force_vec[1],
        dir_2 * force_vec[2],
        0, 0, dir_2 * spr_ptr->get_strut_one()->get_length()/2);
}

// void handle_springs() {
// 	Spring *tmp = spring_ptr;
// 	for(int i = 0; i < 3; i++) {
// 		// handle_one_force(tmp);
// 		drawSpring(tmp);
// 		tmp++;
// 	}
// }

// Simulation loop
void simLoop (int pause) {
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep(world, 0.05); // Step a simulation world, time step is 0.05 [s]
    dJointGroupEmpty (contactgroup);

    // dBodySetForce(capsule.body, 0, 0, 0);

    // ----- APPLY EDGE FORCES (SIMULATE SPRINGS) -----
    // addForce(capsule, capsule2, capsule_one_top, capsule_two_top, 1, 1);

    Strut *tmp_str = strut_ptr;
    for(int i = 0; i < 3; i++)
        drawStrut(tmp_str++);

	Spring *tmp_spr = spring_ptr;
	for(int j = 0; j < 3; j++)
		drawSpring(tmp_spr++);
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
    // fn.path_to_textures = "/usr/local/include/drawstuff/textures"; //path to the texture

    dInitODE(); // Initialize ODE
    world = dWorldCreate(); // Create a dynamic world

    dWorldSetGravity (world, 0, 0, -0.2);
    dWorldSetDamping (world, 0.05, 0.05);

    space = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // ----------------------------------------

    strut_ptr = (Strut *) malloc(3 * sizeof(Strut));
	spring_ptr = (Spring *) malloc(3 * sizeof(Spring));

    // ----------------------------------------

    Strut strut_1(world, space), strut_2(world, space), strut_3(world, space);
    strut_1.set_color({1.0, 0, 0});
    strut_2.set_color({0, 1.0, 0});
    strut_3.set_color({0, 0, 1.0});

    Spring
		spring_1(&strut_1, 1, &strut_2, -1),
		spring_2(&strut_1, 1, &strut_3, -1),
		spring_3(&strut_2, -1, &strut_3, 1);

    strut_ptr = &strut_1;
    strut_ptr++;
    strut_ptr = &strut_2;
    strut_ptr++;
    strut_ptr = &strut_3;
    strut_ptr = strut_ptr - 2;

	spring_ptr = &spring_1;
	spring_ptr++;
	spring_ptr = &spring_2;
	spring_ptr++;
	spring_ptr = &spring_3;
	spring_ptr = spring_ptr - 2;

    // ----------------------------------------

    // Simulation loop
    // argc, argv are argument of main function.
    // fn is a structure of drawstuff
    dsSimulationLoop(argc, argv, 1024, 800, &fn);

    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();

    // free(strut_ptr);

    return 0;
}
