#include "CurrentSense.h"
#include "../../communication/SimpleFOCDebug.h"


// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided
float CurrentSense::getDCCurrent(float motor_electrical_angle){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();
    
    // calculate clarke transform
    ABCurrent_s ABcurrent = getABCurrents(current);

    // current sign - if motor angle not provided the magnitude is always positive
    float sign = 1;

    // if motor angle provided function returns signed value of the current
    // determine the sign of the current
    // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
    if(motor_electrical_angle) {
        float ct;
        float st;
        _sincos(motor_electrical_angle, &st, &ct);
        sign = (ABcurrent.beta*ct - ABcurrent.alpha*st) > 0 ? 1 : -1;  
    }
    // return current magnitude
    return sign*_sqrt(ABcurrent.alpha*ABcurrent.alpha + ABcurrent.beta*ABcurrent.beta);
}

// function used with the foc algorithm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents and getABCurrents internally
DQCurrent_s CurrentSense::getFOCCurrents(float angle_el){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();

    // calculate clarke transform
    ABCurrent_s ABcurrent = getABCurrents(current);
    
    // calculate park transform
    DQCurrent_s return_current = getDQCurrents(ABcurrent,angle_el);

    return return_current;
}

// function used with the foc algorithm
//   calculating Alpha Beta currents from phase currents
//   - function calculating Clarke transform of the phase currents
ABCurrent_s CurrentSense::getABCurrents(PhaseCurrent_s current){

    // check if driver is an instance of StepperDriver
    // if so there is no need to Clarke transform
    if (driver_type == DriverType::Stepper){
        ABCurrent_s return_ABcurrent;
        return_ABcurrent.alpha = current.a;
        return_ABcurrent.beta = current.b;
        return return_ABcurrent;
    }

    // otherwise it's a BLDC motor and 
    // calculate clarke transform
    float i_alpha, i_beta;
    if(!current.c){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else if(!current.a){
        // if only two measured currents
        float a = -current.c - current.b;
        i_alpha = a;  
        i_beta = _1_SQRT3 * a + _2_SQRT3 * current.b;
    }else if(!current.b){
        // if only two measured currents
        float b = -current.a - current.c;
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * b;
    } else {
        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    ABCurrent_s return_ABcurrent;
    return_ABcurrent.alpha = i_alpha;
    return_ABcurrent.beta = i_beta;
    return return_ABcurrent;
}

// function used with the foc algorithm
//   calculating D and Q currents from Alpha Beta currents and electrical angle
//   - function calculating Clarke transform of the phase currents
DQCurrent_s CurrentSense::getDQCurrents(ABCurrent_s current, float angle_el){
 // calculate park transform
    float ct;
    float st;
    _sincos(angle_el, &st, &ct);
    DQCurrent_s return_current;
    return_current.d = current.alpha * ct + current.beta * st;
    return_current.q = current.beta * ct - current.alpha * st;
    return return_current;
}

/**
	Driver linking to the current sense
*/
void CurrentSense::linkDriver(FOCDriver* _driver) {
    driver = _driver;
    // save the driver type for easier access
    driver_type = driver->type();
}


void CurrentSense::enable(){
    // nothing is done here, but you can override this function
};

void CurrentSense::disable(){
    // nothing is done here, but you can override this function
};


// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
// IMPORTANT, this function can be overriden in the child class
int CurrentSense::driverAlign(float voltage, bool modulation_centered){
        
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if (!initialized) return 0;

    // check if stepper or BLDC 
    if(driver_type == DriverType::Stepper)
        return alignStepperDriver(voltage, (StepperDriver*)driver, modulation_centered);
    else
        return alignBLDCDriver(voltage, (BLDCDriver*)driver, modulation_centered);
}



// Helper function to read and average phase currents
PhaseCurrent_s CurrentSense::readAverageCurrents(int N) {
    PhaseCurrent_s c = getPhaseCurrents();
    for (int i = 0; i < N; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a * 0.6f + 0.4f * c1.a;
        c.b = c.b * 0.6f + 0.4f * c1.b;
        c.c = c.c * 0.6f + 0.4f * c1.c;
        _delay(3);
    }
    return c;
};


// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int CurrentSense::alignStepperDriver(float voltage, StepperDriver* stepper_driver, bool modulation_centered){
    
    _UNUSED(modulation_centered);

    bool phases_switched = 0;
    bool phases_inverted = 0;

    if(!_isset(pinA) || !_isset(pinB)){
        SIMPLEFOC_DEBUG("CS: Pins A & B not specified!");
        return 0;
    }

    // set phase A active and phases B down
    // ramp 300ms
    for(int i=0; i < 100; i++){
        stepper_driver->setPwm(voltage/100.0*((float)i), 0);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c = readAverageCurrents();
    // disable the phases
    stepper_driver->setPwm(0, 0);        
    if (fabs(c.a) < 0.1f && fabs(c.b) < 0.1f ){
        SIMPLEFOC_DEBUG("CS: Err too low current!");
        return 0; // measurement current too low
    }
    // align phase A
    // 1) only one phase can be measured so we first measure which ADC pin corresponds 
    // to the phase A by comparing the magnitude
    if (fabs(c.a) < fabs(c.b)){
        SIMPLEFOC_DEBUG("CS: Switch A-B");
        // switch phase A and B
        _swap(pinA, pinB);
        _swap(offset_ia, offset_ib);
        _swap(gain_a, gain_b);
        phases_switched = true; // signal that pins have been switched
    }
    // 2) check if measured current a is positive and invert if not
    if (c.a < 0){
        SIMPLEFOC_DEBUG("CS: Inv A");
        gain_a *= -1;
        phases_inverted = true; // signal that pins have been inverted
    }

    // at this point the driver's phase A is aligned with the ADC pinA
    // and the pin B should be the phase B

    // set phase B active and phases A down
    // ramp 300ms
    for(int i=0; i < 100; i++){
        stepper_driver->setPwm(0, voltage/100.0*((float)i));
        _delay(3);
    }
    _delay(500);
    c = readAverageCurrents();
    stepper_driver->setPwm(0, 0);

    // phase B should be aligned
    // 1) we just need to verify that it has been measured
    if (fabs(c.b) < 0.1f ){
        SIMPLEFOC_DEBUG("CS: Err too low current on B!");
        return 0; // measurement current too low
    }
    // 2) check if measured current a is positive and invert if not
    if (c.b < 0){
        SIMPLEFOC_DEBUG("CS: Inv B");
        gain_b *= -1;
        phases_inverted = true; // signal that pins have been inverted
    }

    // construct the return flag
    // if success and nothing changed return 1 
    // if the phases have been switched return 2
    // if the gains have been inverted return 3
    // if both return 4
    uint8_t exit_flag = 1;
    if(phases_switched) exit_flag += 1;
    if(phases_inverted) exit_flag += 2;
    return exit_flag;
}


