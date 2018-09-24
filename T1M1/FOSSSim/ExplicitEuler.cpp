#include "ExplicitEuler.h"
#include <iostream>

ExplicitEuler::ExplicitEuler()
: SceneStepper()
{}

ExplicitEuler::~ExplicitEuler()
{}

bool ExplicitEuler::stepScene( TwoDScene& scene, scalar dt )
{
    // A vector containing all of the system's position DoFs. x0, y0, x1, y1, ...
    VectorXs& x = scene.getX();
    // A vector containing all of the system's velocity DoFs. v0, v0, v1, v1, ...
    VectorXs& v = scene.getV();
    // A vector containing the masses associated to each DoF. m0, m0, m1, m1, ...
    const VectorXs& m = scene.getM();
    
    // Get force from scene
    VectorXs F = VectorXs::Zero(x.size());
    scene.accumulateGradU(F);
    // Force is negative gradient
    F = -F;
    
    // Fixed Degrees of Freedom
    for(int i = 0; i < scene.getNumParticles(); ++i) {
        if(scene.isFixed(i))
            F.segment<2>(2 * i).setZero();
    }
    //std::cout << "Force: " << F << std::endl;
    
    // Update position
    x += v * dt;
    // Get acceleration and update velocity
    VectorXs a = F.array() / m.array();
    v += a * dt;
    
    return true;
}

std::string ExplicitEuler::getName() const
{
    return "Explicit Euler";
}
