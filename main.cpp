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

// Maximum usable speed
#define MAXIMUMSPEED 60.0
#define MAXIMUMBRAKE 30

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

// PWM outs for transistors
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2H(L2Hpin);
PwmOut L3H(L3Hpin);
// Diagnostics
DigitalOut clk(LED1);

// For connection
Serial pc(SERIAL_TX, SERIAL_RX);

// Some globals for the inputs and outputs.
volatile float desiredSpeedValue = 0.0f;
volatile float desiredRevolutions = 0.0f;
volatile bool spinCW = true;
// Speed Calculation
volatile float measuredSpeed = 0.0f;
// operation mode flags
volatile bool isSinging = false;
volatile bool isSpdCtrl = false;
volatile bool isPosCtrl = false;
//Wait value for fixed speed operation
volatile float fixedSpeedWait = 0;
// If revolutions are not counted
volatile bool noRevs = true;

// For singing
volatile int8_t numberNotes = 0;
volatile int8_t notePointer = 0;
// // Diagnostics
// volatile int8_t brakeRevCount = 0;

//PID controller configurations
volatile float speedKc = 1.2;
volatile float speedTi = 0.6;
volatile float speedTd = 0.05;
volatile float controlKc = 1.0;
volatile float controlTi = 0.0;
volatile float controlTd = 1.0;

// For revolutions
volatile float limitRevolutions = 0.0;

//PID controller outputs
volatile float speedPwm = 1;
volatile float positionPwm = 1;
volatile float currPwm = 1;

PID speedController(speedKc, speedTi, speedTd, 0.02);
PID positionController(controlKc, controlTi, controlTd, 0.02);

// Timing object, variable to store milliseconds, and revCounter
Timer speedTimer;
volatile int revTimer = 0;
volatile int revCounter = 0;

// // Drive states
// const int8_t CWHigh[7] = {0x0, 0x6, 0x3, 0x6, 0x5, 0x5, 0x3};
const int8_t ACWHigh[7] = {0x0, 0x3, 0x5, 0x3, 0x6, 0x6, 0x5};
const float ACWL1H[7] = {0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0};
const float ACWL2H[7] = {0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0};
const float ACWL3H[7] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};
//
// const float CWL1L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0};
// const float CWL2L[7] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0};
// const float CWL3L[7] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
//
const float ACWL1L[7] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0};
const float ACWL2L[7] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
const float ACWL3L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0};

// Drive states
const int8_t CWHigh[7] = {0x0, 0x5, 0x6, 0x6, 0x3, 0x5, 0x3};
const float CWL1H[7] = {0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0};
const float CWL2H[7] = {0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0};
const float CWL3H[7] = {0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.0};

// const int8_t ACWHigh[7] = {0x0, 0x3, 0x5, 0x5, 0x6, 0x3, 0x6};

const float CWL1L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0};
const float CWL2L[7] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
const float CWL3L[7] = {0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0};

// const float ACWL1L[7] = {0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0};
// const float ACWL2L[7] = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0};
// const float ACWL3L[7] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  


// To store notes
volatile int8_t noteArray[8] = {0};
volatile int16_t timeArray[8] = {0};

// Mapping note to frequency
const float frequencyPeriodTable[14] = {253.0963, 238.891, 225.4831, 212.8277, 200.8826, 189.6079, 178.966, 168.9215, 159.4406, 150.4919, 142.0455, 134.0731, 126.5481, 119.4455};

// new vars for eds driving
volatile int lead = 2;
volatile int8_t orgState;
volatile int8_t nextState;

// List of threads.
// Thread* playNotesThread;
Thread* speedPIDThread;
Thread* motorThread;
Thread* revolutionPIDThread;

// For counting revolutions during braking.
// void brakeCount(){
//     brakeRevCount++;
// }

// Function running in playNotesThread,
void playNotes(){
    while(1){
        int currentPeriod = frequencyPeriodTable[noteArray[notePointer]];
        int currentTime = timeArray[notePointer];
        L1L.period_us(currentPeriod);
        L2L.period_us(currentPeriod);
        L3L.period_us(currentPeriod);
        notePointer = (notePointer + 1) % numberNotes;
        Thread::wait(currentTime);
    }
}

// Function in PID thread.
void VPID()
{
    while(1) {
        clk = !clk;
        //1000ms over the timer to calculate the speed, moving average with previous one.
        if(revTimer != 0)
            measuredSpeed = 0.5*measuredSpeed + 500.0/float(revTimer);
        speedController.setSetPoint(desiredSpeedValue);
        speedController.setProcessValue(measuredSpeed);
        speedPwm = speedController.compute();
        pc.printf("%1.3f", speedPwm);
        Thread::wait(20);
    }
}

// Function in PID thread.
void PPID()
{
    while(1) {
        clk = !clk;
        positionController.setSetPoint(desiredRevolutions);
        positionController.setProcessValue(revCounter);
        positionPwm = speedController.compute();
        pc.printf("%1.3f", positionPwm);

        Thread::wait(20);
    }
}

// Interrupt function.
void rps()
{
    // Calculation of speed.
    speedTimer.stop();
    revTimer = speedTimer.read_ms();
    speedTimer.reset();
    speedTimer.start();
    // Increase counter.
    revCounter++;
}

// Advances states at specified rate.
void fixedSpeedRevolutions()
{
    int8_t rotState = 0;
    int8_t rotStateOld = 0;
    while(1) {
        // read current state of rotor
        rotState = I1 + I2*2 + I3*4;
        // if state has changed, set the state to the next state
        if(rotState != rotStateOld){
            rotStateOld = rotState;

            lead = -2;

            if(spinCW)
                lead = 1;

            nextState = ((rotState-orgState+lead+6)%6);
            int8_t driveOut = driveTable[nextState & 0x07];

            currPwm = 1;
            if (isPosCtrl)
                currPwm = positionPwm;
            else if (isSpdCtrl){
                currPwm =speedPwm;

                if (revCounter > limitRevolutions){
                    isSpdCtrl = false;
                    isPosCtrl = true;
                }
            }

            // turn off first
            if (~driveOut & 0x01) L1L = 0;
            if (~driveOut & 0x02) L1H = 1;
            if (~driveOut & 0x04) L2L = 0;
            if (~driveOut & 0x08) L2H = 1;
            if (~driveOut & 0x10) L3L = 0;
            if (~driveOut & 0x20) L3H = 1;
            
            //Then turn on
            if (driveOut & 0x01) L1L.write(currPwm);
            if (driveOut & 0x02) L1H.write(0);
            if (driveOut & 0x04) L2L.write(currPwm);
            if (driveOut & 0x08) L2H.write(0);
            if (driveOut & 0x10) L3L.write(currPwm);
            if (driveOut & 0x20) L3H.write(0);

            
            if(isSinging){
                L1L = L1L/2.0;
                L2L = L2L/2.0;
                L3L = L3L/2.0;
            }
        }

        Thread::wait(20);
    }
}

// Converts char array from start to end into float
// Returns number of notes
int8_t charToNotes(char* commandBuffer, int8_t start, int8_t end)
{
    // Start of first note.
    int8_t current_ptr = start;
    int8_t note_ptr = 0;

    // Clear buffers;
    int i = 0;
    for (i = 0; i < 8; i++) {
        noteArray[i] = 0;
        timeArray[i] = 0;
    }

    // Current pointer should never exceed the end of the buffer.
    while(current_ptr < end) {
        // Each command is either 2 or 3 characters long.
        // If the 2nd charater is # or ^, it is three characters long.
        if(commandBuffer[current_ptr+1] == '#' || commandBuffer[current_ptr+1] == '^') {
            // The 3rd char is the length.
            timeArray[note_ptr] = (commandBuffer[current_ptr+2] - '0')*1000;
            // Match 1st char to the note.
            switch (commandBuffer[current_ptr]) {
                case 'C':
                    noteArray[note_ptr] = 1;
                    break;
                case 'D':
                    noteArray[note_ptr] = 3;
                    break;
                case 'E':
                    noteArray[note_ptr] = 5;
                    break;
                case 'F':
                    noteArray[note_ptr] = 6;
                    break;
                case 'G':
                    noteArray[note_ptr] = 8;
                    break;
                case 'A':
                    noteArray[note_ptr] = 10;
                    break;
                case 'B':
                    noteArray[note_ptr] = 12;
                    break;
                default:
                    noteArray[note_ptr] = 1;
                    break;
            }
            // For the sharp or flat, increment or decrement.
            switch (commandBuffer[current_ptr+1]) {
                case '#':
                    noteArray[note_ptr]++;
                    break;
                case '^':
                    noteArray[note_ptr]--;
                    break;
                default:
                    break;
            }
            // Move on to next pointer in array
            note_ptr++;
            // Advance 3 in buffer.
            current_ptr = current_ptr + 3;

        }

        // Otherwise, command is 2 characters long.
        else {
            // 2nd char is time.
            timeArray[note_ptr] = (commandBuffer[current_ptr+1] - '0')*1000;
            // 1st char is note.
            switch (commandBuffer[current_ptr]) {
                case 'C':
                    noteArray[note_ptr] = 1;
                    break;
                case 'D':
                    noteArray[note_ptr] = 3;
                    break;
                case 'E':
                    noteArray[note_ptr] = 5;
                    break;
                case 'F':
                    noteArray[note_ptr] = 6;
                    break;
                case 'G':
                    noteArray[note_ptr] = 8;
                    break;
                case 'A':
                    noteArray[note_ptr] = 10;
                    break;
                case 'B':
                    noteArray[note_ptr] = 12;
                    break;
                default:
                    noteArray[note_ptr] = 1;
                    break;
            }
            // Increment note pointer, advance buffer pointer by 2.
            note_ptr++;
            current_ptr = current_ptr + 2;
        }
    }
    // Points to after last note, aka equals number of notes.
    return note_ptr;
}

//Converts char array from start to end into float
float charsToFloat(char* commandBuffer, int8_t start, int8_t end)
{
    int8_t isPositive = 1;

    float partDecimal = 0.0;
    float partWhole = 0.0;

    // By default, indexDecimal after the last char
    int8_t indexDecimal = end + 1;

    // If first char is negative, set isPositive flag, and remove the - from
    // consideration.
    if (commandBuffer[start]  == '-') {
        isPositive = -1;
        start++;
    }

    // Decimal point either second last, or third last char.
    if (commandBuffer[end - 1] == '.') {
        indexDecimal = end - 1;
        partDecimal = float(commandBuffer[end] - '0')/10.0f;
    }

    else if (commandBuffer[end - 2] == '.') {
        indexDecimal = end - 2;
        partDecimal = float(commandBuffer[end - 1] - '0')/10.0f + float(commandBuffer[end] - '0')/100.0f;
    }

    // In any case, indexDecimal points to after the ones digit
    // If this case, then only ones.
    if (start == indexDecimal - 1) {
        partWhole = float(commandBuffer[start] - '0');
        // Tens and ones.
    } else if (start == indexDecimal - 2) {
        partWhole = (float(commandBuffer[start] - '0') * 10.0f) + float(commandBuffer[start + 1] - '0');
        // Hundreds, tens, and ones.
    } else {
        partWhole = (float(commandBuffer[start] - '0') * 100.0f) + (float(commandBuffer[start + 1] - '0') * 10.0f) + float(commandBuffer[start + 2] - '0');
    }

    return (partWhole + partDecimal) * isPositive;

}

// Every new command.
void resetThreads()
{
    // Only needed if a thread is running, so if not inactive or idle.
    // Terminate, then join.
    pc.printf("Resetting threads...\r\n");

    // if(playNotesThread->get_state()!=0 && playNotesThread->get_state()!=10) {
    //     playNotesThread->terminate();
    //     playNotesThread->join();
    // }

    if(speedPIDThread->get_state()!=0 && speedPIDThread->get_state()!=10) {
        speedPIDThread->terminate();
        speedPIDThread->join();
    }

    if(revolutionPIDThread->get_state()!=0 && revolutionPIDThread->get_state()!=10) {
        revolutionPIDThread->terminate();
        revolutionPIDThread->join();
    }

    if(motorThread->get_state()!=0 && motorThread->get_state()!=10) {
        motorThread->terminate();
        motorThread->join();
    }

}

//Main function
int main()
{
    pc.printf("Startup!\n\r");
    L1H = 1;
    L2H = 1;
    L3H = 1;    
    L1L = 0;
    L2L = 0;
    L3L = 0;

    orgState = (I1 + I2*2 + I3*4);

    pc.printf("Rotor origin: %x\n\r",orgState);

    // Input buffer
    char command[ARRAYSIZE] = {0};
    char ch;
    // For tracking input and output.
    int index = 0;
    int indexV = 0;
    bool found = false;

    // New threads.
    speedPIDThread = new Thread(osPriorityAboveNormal, 1280);
    motorThread = new Thread(osPriorityNormal, 384);
    // playNotesThread = new Thread(osPriorityNormal, 256);
    revolutionPIDThread = new Thread(osPriorityAboveNormal, 1280);

    // define input limits for PIDs
    speedController.setInputLimits(0.0, 200.0);
    positionController.setInputLimits(0.0, 2000.0);

    // Interrrupt
    I3.mode(PullNone);

    while(1) {
        // If there's a character to read from the serial port
        if (pc.readable()) {
            // Off
            L1H = 1;
            L2H = 1;
            L3H = 1;
            L1L = 0;
            L2L = 0;
            L3L = 0;

            // Remove interrupts
            I3.rise(&rps);
            I3.disable_irq();

            // Ensure singing off, clockwise, not counting revs.
            spinCW = true;
            isSinging = false;
            noRevs = true;
            isSpdCtrl = false;
            isPosCtrl = false;

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
                    if (desiredSpeedValue < 0)
                        spinCW = false;
                    desiredSpeedValue = abs(desiredSpeedValue);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    isSpdCtrl = true;
                    // Begin!
                    //fixedSpeedWait = 8.0f;
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting VPID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    break;

                // Is R first.
                case 'R':
                    noRevs = false;
                    // V also exists, because indexV before index (EOL)
                    if (indexV < index) {
                        desiredRevolutions = charsToFloat(command, 1, indexV - 1);
                        desiredSpeedValue = charsToFloat(command, indexV + 1, index - 1);
                        if (desiredSpeedValue < 0 || desiredRevolutions < 0)
                            spinCW = false;
                        desiredSpeedValue = abs(desiredSpeedValue);
                        desiredRevolutions = abs(desiredRevolutions);
                        // Set values
                        limitRevolutions = floor(desiredRevolutions - MAXIMUMBRAKE * (desiredSpeedValue/MAXIMUMSPEED));
                        if(desiredRevolutions - limitRevolutions < 10)
                            limitRevolutions = desiredRevolutions - 10;
                        if (limitRevolutions < 0)
                            limitRevolutions = 0;
                        revCounter = 0;
                        if (revCounter < limitRevolutions)
                            isSpdCtrl = true;
                        else
                            isPosCtrl = true;

                        speedTimer.reset();
                        speedTimer.start();
                        I3.enable_irq();

                       // fixedSpeedWait = 8.0f;
                        pc.printf("Starting Motor Thread\r\n");
                        motorThread->start(&fixedSpeedRevolutions);
                        pc.printf("Starting VPID Thread.\r\n");
                        speedPIDThread->start(&VPID);
                        pc.printf("Starting PPID Thread.\r\n");
                        revolutionPIDThread->start(&PPID);
                    }
                    // Only R
                    else {
                        desiredRevolutions = charsToFloat(command, 1, indexV - 1);
                        if (desiredRevolutions < 0)
                            spinCW = false;
                        desiredRevolutions = abs(desiredRevolutions);
                        // Set values
                        limitRevolutions = floor(desiredRevolutions - 10);
                        if (limitRevolutions < 0)
                            limitRevolutions = 0;
                        revCounter = 0;
                        speedTimer.reset();
                        speedTimer.start();
                        I3.enable_irq();
                        isPosCtrl = true;
                       // fixedSpeedWait = 8.0f;
                        pc.printf("Starting Motor Thread\r\n");
                        motorThread->start(&fixedSpeedRevolutions);
                        pc.printf("Starting PPID Thread.\r\n");
                        revolutionPIDThread->start(&PPID);
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
                    motorThread->start(&fixedSpeedRevolutions);
                    notePointer = 0;
                    // Run singing thread
                    pc.printf("Singing beginning.\r\n");
                    isSinging = true;
                    // playNotesThread->start(&playNotes);
                    speedPIDThread->start(&playNotes);
                    break;

                //Calibration speedKc, speedTi, speedTd
                // case 'K':
                //     speedKc = charsToFloat(command, 1, index - 1);
                //     speedController.setTunings(speedKc, speedTi, speedTd);
                //     // Start interrupts
                //     speedTimer.reset();
                //     speedTimer.start();
                //     I3.enable_irq();
                //     // Begin!
                //     fixedSpeedWait = 8.0f;
                //     // Run threads.
                //     pc.printf("Starting Motor Thread\r\n");
                //     motorThread->start(&fixedSpeed);
                //     pc.printf("Starting VPID Thread.\r\n");
                //     speedPIDThread->start(&VPID);
                //     break;
                // case 'I':
                //     speedTi = charsToFloat(command, 1, index - 1);
                //     speedController.setTunings(speedKc, speedTi, speedTd);
                //     // Start interrupts
                //     speedTimer.reset();
                //     speedTimer.start();
                //     I3.enable_irq();
                //     // Begin!
                //     fixedSpeedWait = 8.0f;
                //     // Run threads.
                //     pc.printf("Starting Motor Thread\r\n");
                //     motorThread->start(&fixedSpeed);
                //     pc.printf("Starting VPID Thread.\r\n");
                //     speedPIDThread->start(&VPID);
                //     break;
                // case 'D':
                //     speedTd = charsToFloat(command, 1, index - 1);
                //     speedController.setTunings(speedKc, speedTi, speedTd);
                //     // Start interrupts
                //     speedTimer.reset();
                //     speedTimer.start();
                //     I3.enable_irq();
                //     // Begin!
                //     fixedSpeedWait = 8.0f;
                //     // Run threads.
                //     pc.printf("Starting Motor Thread\r\n");
                //     motorThread->start(&fixedSpeed);
                //     pc.printf("Starting VPID Thread.\r\n");
                //     speedPIDThread->start(&VPID);
                //     break;

                case 'K':
                    noRevs = false;
                    controlKc = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    revCounter = 0;
                    fixedSpeedWait = 8.0f;
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);
                    break;
                case 'I':
                    noRevs = false;
                    controlTi = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    revCounter = 0;
                    fixedSpeedWait = 8.0f;
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);
                    break;
                case 'D':
                    noRevs = false;
                    controlTd = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    revCounter = 0;
                    fixedSpeedWait = 8.0f;
                    // Run threads.
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);
                    break;

                case 'J':
                    noRevs = false;
                    controlKc = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    // Begin!
                    revCounter = 0;
                    I3.enable_irq();
                    fixedSpeedWait = 8.0f;
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting VPID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);
                    break;
                case 'U':
                    noRevs = false;
                    controlTi = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    revCounter = 0;
                    fixedSpeedWait = 8.0f;
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting VPID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);
                    break;
                case 'S':
                    noRevs = false;
                    controlTd = charsToFloat(command, 1, index - 1);
                    positionController.setTunings(controlKc, controlTi, controlTd);
                    // Start interrupts
                    speedTimer.reset();
                    speedTimer.start();
                    I3.enable_irq();
                    // Begin!
                    revCounter = 0;
                    fixedSpeedWait = 8.0f;
                    pc.printf("Starting Motor Thread\r\n");
                    motorThread->start(&fixedSpeedRevolutions);
                    pc.printf("Starting VPID Thread.\r\n");
                    speedPIDThread->start(&VPID);
                    pc.printf("Starting PPID Thread.\r\n");
                    revolutionPIDThread->start(&PPID);

                // Braking function
                // case 'B':
                //     brakeRevCount = 0;
                //     I3.mode(PullNone);
                //     I3.rise(&brakeCount);
                //     I3.enable_irq();
                //     motorHigh = CWHigh[1];
                //     L1L = CWL1L[1];
                //     L2L = CWL2L[1];
                //     L3L = CWL3L[1];
                //     break;
                // case 'C':
                //     pc.printf("Brake Count: %d\n\r", brakeRevCount);
                //     break;

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