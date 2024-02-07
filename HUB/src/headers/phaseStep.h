#pragma once
#include <string>

struct phaseStep {
    float time;
    std::string phase;
    phaseStep* next;
};