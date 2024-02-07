#pragma once
#include <string>
#include "phase_step.h"
#include "path_step.h"

class uas {
public:
    float* state;
    float epsilon, fovNarrow, fovWide, theta, thetaJoint, jointTime;
    bool jointComplete;
    std::string status;
    pathStep* pathRoot;
    pathStep* pathNow;
    phaseStep* phaseRoot;
    phaseStep* phaseNow;
    unsigned short int p;

    uas();
};