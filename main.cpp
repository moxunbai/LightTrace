#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include "Pinholecamera.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    Scene scene(784, 784);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);
    MeshTriangle left("../models/cornellbox/left.obj", red);
    MeshTriangle right("../models/cornellbox/right.obj", green);
    MeshTriangle light_("../models/cornellbox/light.obj", light);

    Material* sp1 = new Material(MICROFACET, Vector3f(0.0f));
	sp1->Kd  = Vector3f(0.725f, 0.71f, 0.68f);
    sp1->ior = 3.2f;

    Material* sp2 = new Material(MICROFACET, Vector3f(0.0f));
	sp2->Kd  = Vector3f(0.725f, 0.71f, 0.68f);
    sp2->ior = 2.1f;
	
    Sphere leftBall(Vector3f(274,185,251),85,sp1);
	Sphere rightBall(Vector3f(261,122,105),82,sp2);

    Sphere ball1(Vector3f(300,100,300),100,sp1);

    // MeshTriangle glass1("../models/cornellbox/glass1.obj", sp1);
	
    scene.Add(&floor);
    //scene.Add(&shortbox);
    //scene.Add(&tallbox);
	scene.Add(&leftBall);
	// scene.Add(&rightBall);
    // scene.Add(&glass1);
    // scene.Add(&ball1);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    Vector3f eye_pos(278, 273, -800);
    Vector3f dst(0, 0, 1);
    Vector3f up(0, 1, 0);
    PinholeCamera pinholeCamera(1.0f , eye_pos , dst , up ,M_PI/2.0f , 0.5f);
    scene.setCamera(&pinholeCamera);
    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}