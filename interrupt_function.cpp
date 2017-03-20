//***************************************************
//Pin assignment declarations
//***************************************************


//NOT ACCOMMODATING THAT ONE (D9) CANNOT BE PWM OUT

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7  //QUADRATURE_PIN
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

//One input at least will need to be an InterruptIn for rps functions
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

BusOut motorLow(L1Lpin, L2Lpin, L3Lpin);
BusOut motorHigh(L1Hpin, L2Hpin, L3Hpin);

//DEPENDS ON HOW WE IMPLEMENT THE MOTOR CONTROL PINS
inline void motorStop()
{
	motorLow = 0x0;
	motorHigh = 0x7;
}

//***************************************************
//Function for calculating the rotational velocity of the motor
//***************************************************

int8_t CWLow[6] = {0x2, 0x2, 0x1, 0x1, 0x4, 0x4};
<<<<<<< HEAD
int8_t CWHigh[6] = {0x6, 0x3, 0x3, 0x5, 0x5, 0x6};

int8_t ACWLow[6] = {0x4, 0x1, 0x1, 0x2, 0x2, 0x4};
int8_t ACWHigh[6] = {0x5, 0x5, 0x3, 0x3, 0x6, 0x6};

bool spinCW = True;
=======
int8_t CWHigh[6] = {0x2, 0x2, 0x1, 0x1, 0x4, 0x4};

int8_t ACWLow[6] = {0x4, 0x1, 0x1, 0x2, 0x2, 0x4};
int8_t ACWHigh[6] = {0x5, 0x5, 0x3, 0x3, 0x6, 0x6};
>>>>>>> b910c3d07d66fc2800669c051b13b6fceab7410d

//DEPENDS ON HOW WE IMPLEMENT THE MOTOR CONTROL PINS
void motorOut(int8_t driveState)
{
	//Switch off all transistors to prevent shoot-through
	motorLow = 0x0;
	motorHigh = 0x7;

	//Assign new values to motor output
	if(spinCW){
		motorLow = CWLow[driveState];
		motorHigh = CWHigh[driveState];
	} else {
		motorLow = ACWLow[driveState];
		motorHigh = ACWHigh[driveState];
	}
}

//***************************************************
//Function for calculating the rotational velocity of the motor
//***************************************************

InterruptIn PIN;
Timer speedTimer;
float revTimer = 0;
float measuredSpeed = 0;


PIN.rise(&rps);

float position = 0; // variable to store actual rotational position of the motor. Photointerrupt is 0deg

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
	position = 0;
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

Thread speedPIDThread;

speedPIDThread.start(VPID);

//PID controller configuration
float speedPIDrate = 0.2;
float speedKc = 0.2;
float speedTi = 0.8;
float speedTd = 0.0;

//Speed demanded by user
float speedTarget = 0;

//PID controller output
float speedOutput = 0;

//Wait value for fixed speed operation
float fixedSpeedWait = 0;

//Initialise PID controller
PID speedController(speedKc, speedTi, speedTd, speedPIDrate);

//Tuning calls for PID controller
speedController.setTunings(speedKc, speedTi, speedTd);
speedController.setInterval(speedPIDrate);
speedController.setSetPoint(speedTarget);
speedController.setMode(1); //Set controller to automatic mode

//Set priority of thread
speedPIDthread.set_priority(osPriorityAboveNormal);

void VPID()
{
	while(1) {
		speedController.setProcessValue(measuredSpeed);
		speedOutput = speedController.compute();
		fixedSpeedWait = (1/(speedOutput*6));
		Thread::wait(speedPIDrate);
	}
}

//***************************************************
//Thread for PWM-ing between two states, for moving very slowly
//***************************************************

int8_t PwmStateAHigh = 0;
int8_t PwmStateALow = 0;
int8_t PwmStateBHigh = 0;
int8_t PwmStateBLow = 0;

Thread StatePwmThread;

StatePwmThread.start(StatePwm);

float ratio = 0.5;
float PwmPeriod_ms = 10.0;

void StatePwm(){
	while(1){
		motorHigh = PwmStateAHigh;
		motorLow = PwmStateALow;
		Thread::wait(ratio*PwmPeriod_ms);
		motorHigh = PwmStateBHigh;
		motorLow = PwmStateBLow;
		Thread::wait((1-ratio)*PwmPeriod_ms);
	}
}

//***************************************************
//Thread for calculating PID for distance
//***************************************************
Thread distancePIDThread;

distancePIDThread.start(RPID);

//PID controller configuration
float distancePIDrate = 0.2;
float distanceKc = 0.2;
float distanceTi = 0.8;
float distanceTd = 0.0;

//Speed demanded by user
float distanceTarget = 0.0;

int8_t countRevs = 0;

//PID controller output
// float distanceOutput = 0.0;
float distanceLimit = 5.0;

//Wait value for fixed speed operation
// float fixeddistanceWait = 0;

//Initialise PID controller
PID distanceController(distanceKc, distanceTi, distanceTd, distancePIDrate);

//Tuning calls for PID controller
distanceController.setTunings(distanceKc, distanceTi, distanceTd);
distanceController.setInterval(distancePIDrate);
distanceController.setSetPoint(distanceTarget - distanceLimit);
distanceController.setMode(1); //Set controller to automatic mode

//Set priority of thread
speedPIDthread.set_priority(osPriorityAboveNormal);

void RPID()
{
	while(1) {
		if(countRevs<(distanceTarget - distanceLimit)){
			distanceController.setProcessValue(countRevs);
			speedOutput = speedController.compute();
			fixedSpeedWait = (1/(speedOutput*6));
		} else {
			//switch to quadrature encoder mode
		}
		Thread::wait(distancePIDrate);
	}
}

//***************************************************
//Function for calculating the rotational velocity of the motor AND counting rotations
//***************************************************

InterruptIn PIN;
Timer speedTimer;
float revTimer = 0;
float measuredSpeed = 0;
int countRevs = 0; //Already declared in RPID function

PIN.rise(&rps_counter);

//Don't forget to start the timer in the main loop before
//attaching the interrupt

void rps_counter()
{
	speedTimer.stop();
	revTimer = speedTimer.read_ms();
	speedTimer.reset();
	speedTimer.start();

	//1000ms over the timer to calculate the speed
	measuredSpeed = 1000/(revTimer);

	countRevs++;
	position = 0;

}

//***************************************************
//Quadrature mode interrupts and controllers
//***************************************************

InterruptIn QUADRATURE_PIN(CHA);

// WARNING	only attach at low speeds, otherwise it will trigger too often
QUADRATURE_PIN.rise(&quadrature_counter);

//Don't forget to start the timer in the main loop before
//attaching the interrupt

void quadrature_counter()
{

	position = position + 3.07;	

}
