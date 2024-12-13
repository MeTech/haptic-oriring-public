/**********************************************************************************************
* Pyramid Force Resistive Sensor - Designed by Sunju
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/

#include "Arduino.h"
#include "pyramidFSR.h"

/* Constructor*/
PFSR::PFSR(int pins[4]){
    // set the fsr pins
    for(int i=0; i<4; i++){
        this->sectionPINs[i] = pins[i];
    }
}

void PFSR::setupPINs(){  // needs to be called after the creation of the objects
    for(int i=0; i<4; i++){
        pinMode(this->sectionPINs[i], INPUT_PULLDOWN);
    }
}

void PFSR::readNormalForce(){
    double section_poly_coeff[4][4]= {
        {0.000005664,   -0.003954,  0.9425,     -67.69},
        {0.0000004643,  -0.0004549, 0.1498,     -12.9153},
        {0.000000507,   -0.0004168, 0.1201,     -9.683},
        {0.0000007124,  -0.0004253, 0.08852,    -5.84165}
    };

    for(int i=0; i<4; i++){
        
        //this->sectionValues[i] = lowpassFilter(this->sectionValues[i], analogRead(this->sectionPINs[i]), 0.90);
        this->sectionValues[i] = analogRead(this->sectionPINs[i]);
        double sV = double(this->sectionValues[i]);
        this->sectionForces[i] = section_poly_coeff[i][0]*pow(sV,3) + section_poly_coeff[i][1]*pow(sV,2) + section_poly_coeff[i][2]*sV + section_poly_coeff[i][3];
    }

    this->normalForce = this->sectionForces[0] + this->sectionForces[1] + this->sectionForces[2] + this->sectionForces[3];
}

void initialBalance(){ 
}

void PFSR::readShearXForce(){
    if(this->normalForce>0)
        this->shearXForce = (this->sectionForces[0]+this->sectionForces[1]-this->sectionForces[3]-this->sectionForces[2])/this->normalForce;
    else
        this->shearXForce = 0;

}

void PFSR::readShearYForce(){
    if(this->normalForce>0)
        this->shearYForce = (this->sectionForces[0]-this->sectionForces[1]+this->sectionForces[3]-this->sectionForces[2]);
    else
        this->shearYForce = 0;
}

int PFSR::shearDirection(){
    // it should scan over a time

    //check over a period 
}
double PFSR::lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}
