/**********************************************************************************************
* Pyramid Force Resistive Sensor - Designed by Sunju Kang
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/
#ifndef pyramidFsR
#define pyramidFsR

class PFSR
{
    private:
    public:
    // Pins for pressure sensors
    int sectionPINs[4];
    int sectionValues[4];
    double sectionForces[4];
    double normalForce, shearXForce, shearYForce;
    // double section_poly_coeff[4][4]; // couldn't figure out how to pass

    // Constructor  for 3 DoF
    PFSR(int pins[4]);
    
    // assign the input and output pins and sets the frequencies
    void setupPINs();
    void readNormalForce();
    void readShearXForce();
    void readShearYForce();
    void initialBalance();
    int shearDirection();
    double lowpassFilter(double, double, double);
};

#endif
