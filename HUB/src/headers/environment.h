#pragma once

class environment {
public:
    float time, timeStep;
    float bounds[6];

    environment();
    environment(float boundsIn[6], float timeStepIn);
};