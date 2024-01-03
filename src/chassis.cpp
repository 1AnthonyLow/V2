#include "main.h"
#include "robot.h"
#include "util.h"
#include <math.h>
#include <vector>

void move(float lpwr, float rpwr) {
    lf.move(lpwr);
    lm.move(lpwr);
    lb.move(lpwr);
    rf.move(lpwr);
    rm.move(lpwr);
    rb.move(lpwr);
}

void moveTo(float target, int duration, int exitTime, int powerCap) {
    const float kP = 5;
    const float kI = 0;
    const float kD = 0;
    const float kJ = 0;

    float error = 0;
    float integral = 0;
    float derivative;
    float output;
    float correction;
    float prevError;
    //float initial = imu.get_heading();
    float heading;
    float headingError;
    int x = 0;

    reset();

    Timer timer;

    while(true) {
        timer.startTime();

        /*heading = imu.get_heading();
        if (heading - initial > 180) headingError = heading - initial - 360; // Heading correction
        else headingError = heading - initial;
        correction = headingError*kJ; */

        prevError = error;

        error = target - lf.get_position(); //P

        integral = integral + error; // I
        if (fabs(error) < 1) integral = 0;
        else if (integral > 50) integral = 50;

        derivative = error - prevError; // D

        output = error*kP+integral*kI+derivative*kD; //PID
        if (output > powerCap) output = powerCap;

        //if (error < 1) x++;
        //if (x > exitTime) break;

        move(output - correction, output + correction);

        //if (timer.getTime() > duration) break;
    }
}

void turnTo(float target, int duration, int exitTime, int powerCap) {
    float kP;
    float kI;
    float kD;
    float error = 0;
    float integral = 0;
    float derivative;
    float output;
    float prevError;
    float initial = imu.get_heading();
    float heading;
    int x = 0;
    float delta = fabs(target - initial);
    std::vector<float> constants;

    if      (delta <=  30) constants = {0,0,0};  
    else if (delta <=  60) constants = {0,0,0}; 
    else if (delta <=  90) constants = {0,0,0}; 
    else if (delta <= 120) constants = {0,0,0}; 
    else if (delta <= 150) constants = {0,0,0}; 
    else if (delta <= 180) constants = {0,0,0}; 

    reset();

    Timer timer;

    while(true) {
        timer.startTime();

        if (heading - initial > 180) error = heading - initial - 360; // P
        else error = heading - initial;

        integral = integral + error; // I
        if (fabs(error) < 1) integral = 0;
        else if (integral > 50) integral = 50;

        derivative = error - prevError; // D

        output = error*kP+integral*kI+derivative*kD; //PID
        if (output > powerCap) output = powerCap;

        if (error < 1) x++;
        if (x > exitTime) break;

        move(output, -output);

        if (timer.getTime() > duration) break;
    }
}

void arc(float radius, float theta, std::string dir) {
    float curr;
    float error;
    float ratio;
    float inner = 0;
    float outer = 0;
    float ito = 11.05;
    float arclen = radius*M_PI*theta/180;
    float target = arclen;
    float offradius = radius - ito;

    std::string left = "left";
    std::string right = "right";

    reset();
    
    while(true) {
        if(dir.find(left) != std::string::npos) {
            curr = rf.get_position();
            move(inner, outer);
        }
        else if(dir.find(right) != std::string::npos) {
            curr = lf.get_position();
            move(outer, inner);
        }
        error = target - curr;
        ratio = offradius/radius; // The ratio of radii is equivalent to the ratio of speeds to maintain the arc
        outer = error;
        inner = ratio*outer;
        
    }
}

