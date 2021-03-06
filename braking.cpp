#include "mbed.h"
#include "rtos.h"
#include "PID.h"

// Photointerrupter input pins
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

//Define sized for command arrays
#define ARRAYSIZE 49

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

// Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
InterruptIn I3(I3pin);

//NOTE, BusOut declares things in reverse (ie, 0, 1, 2, 3) compared to binary represenation
// BusOut motorLow(L1Lpin, L2Lpin, L3Lpin);
BusOut motorHigh(L1Hpin, L2Hpin, L3Hpin);
// PWM out for singing.
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);
// Diagnostics
DigitalOut clk(LED1);

// For connection
Serial pc(SERIAL_TX, SERIAL_RX);

// Some globals
volatile float desiredSpeedValue = 0.0f;
volatile float desiredRevolutions = 0.0f;
volatile float measuredSpeed = 0.0f;
volatile bool spinCW = true;
volatile bool isSinging = false;
//Wait value for fixed speed operation
volatile float fixedSpeedWait = 0;

// Diagnostics
volatile int8_t brakeRevCount = 0;

//PID controller configuration
volatile float speedPIDrate = 0.5;
volatile float speedKc = 0.2;
volatile float speedTi = 1.0;
volatile float speedTd = 0.01;

// For revolutions
volatile int16_t wholeRevolutions = 0;
volatile float partThereof = 0.0f

//PID controller output
volatile float speedOutput = 0;

PID speedController(speedKc, speedTi, speedTd, speedPIDrate);

Timer speedTimer;
volatile int revTimer = 0;

// Drive states
const int8_t CWHigh[7] = {0x0, 0x6, 0x3, 0x6, 0x5, 0x5, 0x3};
const int8_t ACWHigh[7] = {0x0, 0x3, 0x5, 0x3, 0x6, 0x6, 0x5};

const float CWL1L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0};
const float CWL2L[7] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0};
const float CWL3L[7] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};

const float ACWL1L[7] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0};
const float ACWL2L[7] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
const float ACWL3L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0};

// List of threads.
Thread* brakingThread;
Thread* fixedSpeedThread;


// Interrupt function.
void rps()
{
    // Disable during this function.
    I3.disable_irq();
    // Calculation of speed.
    speedTimer.stop();
    revTimer = speedTimer.read_ms();
    speedTimer.reset();
    speedTimer.start();
    //1000ms over the timer to calculate the speed, moving average with previous one.
    // if(revTimer != 0)
    //     measuredSpeed = 0.5*measuredSpeed + 500/(revTimer);
//        measuredSpeed = 1000/(revTimer);
    // Carry on.
    I3.enable_irq();
    brakeRevCount++;
}

// Advances states at specified rate.
void fixedSpeed()
{
    while(1) {
        motorHigh = 0x7;
        L1L = 0;
        L2L = 0;
        L3L = 0;

        motorHigh = CWHigh[(I1 + I2*2 + I3*4)];
        L1L = CWL1L[(I1 + I2*2 + I3*4)];
        L2L = CWL2L[(I1 + I2*2 + I3*4)];
        L3L = CWL3L[(I1 + I2*2 + I3*4)];
        
        Thread::wait(fixedSpeedWait);
    }
}




// Every new command.
void resetThreads()
{
    // Only needed if a thread is running, so if not inactive or idle.
    // Terminate, then join.
    pc.printf("Resetting threads...\r\n");

    if(fixedSpeedThread->get_state()!=0 && fixedSpeedThread->get_state()!=10) {
        fixedSpeedThread->terminate();
        fixedSpeedThread->join();
    }

}

//Main function
int main()
{
    pc.printf("Startup!\n\r");
    motorHigh = 0x7;
    L1L = 0;
    L2L = 0;
    L3L = 0;

    // Input buffer
    char command[ARRAYSIZE] = {0};
    char ch;
    // For tracking input and output.
    int index = 0;
    int indexV = 0;
    bool found = false;

    // New threads.
    fixedSpeedThread = new Thread(osPriorityNormal, 512);
    brakingThread = new Thread(osPriorityNormal, 512)

    // Interrrupt
    I3.mode(PullNone);
    I3.rise(&rps);
    I3.disable_irq();

    while(1) {
        // If there's a character to read from the serial port
        if (pc.readable()) {

            // Brake on.
            motorHigh = CWHigh[1];
            L1L = CWL1L[1];
            L2L = CWL2L[1];
            L3L = CWL3L[1];
            wait(1.0);
            // Off
            motorHigh = 0x7;
            L1L = 0;
            L2L = 0;
            L3L = 0;

            // Remove interrupts
            I3.disable_irq();

            // Remove threads
            resetThreads();

            // Clear counters
            index = 0;

            // Read each value from the serial port until Enter key is pressed
            do {
                // Read character
                ch = pc.getc();
                // Print character to serial for visual feedback
                pc.putc(ch);
                // Add character to input array
                command[index++] = ch;  // put it into the value array and increment the index
                // d10 and d13 used for detecting Enter key on Windows/Unix/Mac
            } while(ch != 10 && ch != 13);

            // 10 and 13 are basically EOL symbols
            // index points to after EOL, so decrement to point to EOL
            index--;

            // Detect location of V, if at all.
            indexV = 0;
            found = false;

            while(indexV <= index && !found) {
                if(command[indexV] == 'V') {
                    found = true;
                }
                indexV++;
            }
            // IndexV ends one more than index of V if V exists
            // OR ends at index+1, which in this case is after EOL. Decrement.
            indexV--;
            // IndexV is now V, or EOL.

            // Start new line on terminal for printing data
            pc.putc('\n');
            pc.putc('\r');

            //Analyse the input string
            switch (command[0]) {

                // Only V
                case 'V':
                    // index is EOL, index - 1 is last char
                    desiredSpeedValue = charsToFloat(command, 1, index - 1);
                    speedController.setSetPoint(desiredSpeedValue);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    fixedSpeedWait = 0.001;
                    pc.printf("Wait: %2.3f\r\n", fixedSpeedWait);
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    fixedSpeedThread->start(&fixedSpeed);
                    pc.printf("Starting PID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    break;

                // Is R first.
                case 'R':
                    // V also exists, because indexV before index (EOL)
                    if (indexV < index) {
                        desiredRevolutions = charsToFloat(command, 1, indexV - 1);
                        desiredSpeedValue = charsToFloat(command, indexV + 1, index - 1);
                        // Run thread.
                        speedController.setSetPoint(desiredSpeedValue);
                        // Set values
                        wholeRevolutions = floor(desiredRevolutions) - 1;
                        partThereof = (float)desiredRevolutions - (float)wholeRevolutions;
                    }
                    // Only R
                    else {
                        desiredRevolutions = charsToFloat(command, 1, index - 1);
                        // Run thread.
                    }
                    break;

                // Needs to sing
                case 'T':
                    numberNotes = charToNotes(command, 1, index - 1);
                    // Run normal speed thread
                    for(notePointer = 0; notePointer < numberNotes; notePointer++){
                        pc.printf("Note: %d, Time: %d.\r\n", noteArray[notePointer], timeArray[notePointer]);
                    }
                    fixedSpeedWait = 8.0f;
                    fixedSpeedThread->start(&fixedSpeed);
                    notePointer = 0;
                    // Run singing thread
                    pc.printf("Singing beginning.\r\n");
                    isSinging = true;
                    playNotesThread->start(&playNotes);
                    break;

                //Calibration speedKc, speedTi, speedTd
                case 'K':
                    speedKc = charsToFloat(command, 1, index - 1);
                    speedController.setTunings(speedKc, speedTi, speedTd);
                    speedController.setSetPoint(desiredSpeedValue);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    fixedSpeedWait = 0.001;
                    pc.printf("Wait: %2.3f\r\n", fixedSpeedWait);
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    fixedSpeedThread->start(&fixedSpeed);
                    pc.printf("Starting PID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    break;
                case 'I':
                    speedTi = charsToFloat(command, 1, index - 1);
                    speedController.setTunings(speedKc, speedTi, speedTd);
                    speedController.setSetPoint(desiredSpeedValue);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    fixedSpeedWait = 0.001;
                    pc.printf("Wait: %2.3f\r\n", fixedSpeedWait);
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    fixedSpeedThread->start(&fixedSpeed);
                    pc.printf("Starting PID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    break;
                case 'D':
                    speedTd = charsToFloat(command, 1, index - 1);
                    speedController.setTunings(speedKc, speedTi, speedTd);
                    speedController.setSetPoint(desiredSpeedValue);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    fixedSpeedWait = 0.001;
                    pc.printf("Wait: %2.3f\r\n", fixedSpeedWait);
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    fixedSpeedThread->start(&fixedSpeed);
                    pc.printf("Starting PID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    break;

                // Braking function
                case 'B':
                    I3.mode(PullNone);
                    I3.rise(&brakeCount);
                    I3.enable_irq();
                    brakeRevCount = 0;
                    motorHigh = CWHigh[1];
                    L1L = CWL1L[1];
                    L2L = CWL2L[1];
                    L3L = CWL3L[1];
                    break;
                case 'C':
                    pc.printf("Brake Count: %d\n\r", brakeRevCount);
                    break;

                // If something weird comes along.
                default:
                    // Commands to kill all threads
                    // Reset all values to zero
                    pc.printf("Error in received data.\n\r");
                    break;
            }

            pc.printf("desiredSpeed: %3.2f, ", desiredSpeedValue);
            pc.printf(" desiredRevolutions: %3.2f\n\r", desiredRevolutions);

            // Clear buffer
            for(index = 0; index < 49; index++) {
                command[index] = 0;
            }
        }
        wait(0.01);
    }

}
