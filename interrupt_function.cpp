//***************************************************
//Function for reading the photointerrupter states
//***************************************************

//One input at least will need to be an InterruptIn for rps function
DigitalIn I1, I2, I3

inline int8_t readRotorState()
{
    return (I1 + I2<<1 + I3<<2);
}

//***************************************************
//Function for calculating the rotational velocity of the motor
//***************************************************

InterruptIn PIN;
Timer speedTimer;

PIN.attach(&rps);

void rps()
{
	speedTimer.stop();
	revtimer = speedTimer.read_ms();
	speedTimer.reset();
	speedTimer.start();

	measuredSpeed = 1000/(revtimer);
}

//***************************************************
//Thread for rotating at a fixed speed
//***************************************************

Thread fixedSpeedThread;

fixedSpeedThread.start(fixedSpeed);

void fixedSpeed()
{
	while(1){
		intState = readRotorState();
		motorOut(intState);
		Thread::wait(fixedSpeedWait);
	}
}

//***************************************************
//Thread for calculating new speed values in PID loop
//***************************************************

Thread PIDThread;

PIDThread.start(VPID);

//PID controller configuration
float PIDrate = 0.2;
float Kc = 0.2;
float Ti = 0.8;
float Td = 0.0;

//Speed demanded by user
float speedTarget = 0;

//PID controller output
float speedOutput = 0;

//Wait value for fixed speed operation
float fixedSpeedWait = 0;

//Initialise PID controller
PID speedController(Kc, Ti, Td, PIDrate);

//Tuning calls for PID controller
speedController.setTunings(Kc, Ti, Td);
speedController.setInterval(PIDrate);
speedController.setSetPoint(speedTarget);

void VPID()
{
    while(1) {
        speedController.setProcessValue(measuredSpeed);
        speedOutput = speedController.compute();
        fixedSpeedWait = (1/(speedOutput*6));
        Thread::wait(PIDrate);
    }
}