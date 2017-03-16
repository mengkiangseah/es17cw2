//Function for calculating the rotational velocity of the motor

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

//Thread for rotating at a fixed speed

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