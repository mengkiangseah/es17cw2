//***************************************************
//Pin assignment declarations
//***************************************************

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//***************************************************
//Function for reading the photointerrupter states
//***************************************************

//One input at least will need to be an InterruptIn for rps function
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

inline int8_t readRotorState()
{
    return (I1 + I2<<1 + I3<<2);
}

//***************************************************
//Universal function to stop the motor (non-braking)
//***************************************************

//DEPENDS ON HOW WE IMPLEMENT THE MOTOR CONTROL PINS
inline void motorStop()
{

}

//***************************************************
//Function for calculating the rotational velocity of the motor
//***************************************************

//DEPENDS ON HOW WE IMPLEMENT THE MOTOR CONTROL PINS
void motorOut(int8_t driveState)
{
	//Switch off all transistors to prevent shoot-through

	//Assign new values to motor output
}

//***************************************************
//Function for calculating the rotational velocity of the motor
//***************************************************

InterruptIn PIN;
Timer speedTimer;
float revTimer = 0;


PIN.rise(&rps);

//Don't forget to start the timer in the main loop before
//attaching the interrupt

void rps()
{
	speedTimer.stop();
	revTimer = speedTimer.read_ms();
	speedTimer.reset();
	speedTimer.start();

	//1000ms over the timer to calculate the speed
	measuredSpeed = 1000/(revTimer);
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
speedController.setMode(1); //Set controller to automatic mode

//Set priority of thread
PIDthread.set_priority(osPriorityAboveNormal);

void VPID()
{
    while(1) {
        speedController.setProcessValue(measuredSpeed);
        speedOutput = speedController.compute();
        fixedSpeedWait = (1/(speedOutput*6));
        Thread::wait(PIDrate);
    }
}