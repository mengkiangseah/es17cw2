#include "mbed.h"
// #include "rtos.h"
// #include "PID.h"

//PID controller configuration
// float PIDrate = 0.2;
// float Kc = 10.0;
// float Ti = 3.0;
// float Td = 0;
// float speedControl = 0;
// PID controller(Kc, Ti, Td, PIDrate);
// Thread VPIDthread;

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
BusOut motorLow(L1Lpin, L2Lpin, L3Lpin);
BusOut motorHigh(L1Hpin, L2Hpin, L3Hpin);

PwmOut L1Lpin

//Timeout function for rotating at set speed
// Timeout spinTimer;
// float spinWait = 10;
// float revsec = 0;

//Variables for spinning N revolutions
// int8_t targetRevs = 0;
// int8_t currRevs = 0;

//Timer used for calculating speed
// Timer speedTimer;
// float measuredRevs = 0, revtimer = 0;

// For connection
Serial pc(SERIAL_TX, SERIAL_RX);

// Some globals
float desiredSpeedValue = 0.0f;
float desiredRevolutions = 0.0f;
float measuredVelocity = 0.0f;
int8_t numberNotes = 0;
int8_t notePointer = 0;
bool spinCW = true;

// Drive states
int8_t CWLow[7] = {0x0, 0x2, 0x2, 0x1, 0x1, 0x4, 0x4};
int8_t CWHigh[7] = {0x0, 0x6, 0x3, 0x3, 0x5, 0x5, 0x6};

int8_t ACWLow[7] = {0x0, 0x4, 0x1, 0x1, 0x2, 0x2, 0x4};
int8_t ACWHigh[7] = {0x0, 0x5, 0x5, 0x3, 0x3, 0x6, 0x6};

// To store notes
int8_t noteArray[8] = {0};
int8_t timeArray[8] = {0};

// Mapping note to frequency
const float frequencyTable[14] = {8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f, 8000.0f};

// List of threads.
Thread singingSpeedThread;
Thread playsNotesThread;

// Reads the state.
inline int8_t readRotorState(){
	return (I1 + I2<<1 + I3<<2);
}

// Stops motor.
// inline void motorStop(){
// 	motorLow = 0x0;
// 	motorHigh = 0x7;
// }

// Sets output to next state.
// inline void motorOut(int8_t driveState){
// 	//Switch off all transistors to prevent shoot-through
//     motorStop();
//
// 	//Assign new values to motor output
// 	if(spinCW){
// 		motorLow = CWLow[driveState];
// 		motorHigh = CWHigh[driveState];
// 	}
//
//     else {
// 		motorLow = ACWLow[driveState];
// 		motorHigh = ACWHigh[driveState];
// 	}
// }

// Function running in singingSpeedThreads, approx 4ms period per phase
// Means 24ms, 41.6Hz
void singingSpeed(){
    while(1){
        motorHigh = 0x7;
        motorLow = 0x0;
        motorHigh = CWHigh[(I1 + I2*2 + I3*4)];
        motorLow = CWLow[(I1 + I2*2 + I3*4)];
        Thread::wait(4.0f);
    }
}

// Function running in playsNotesThread,
void playNotes(){
    while(1){

    }
}

//Converts char array from start to end into float
// Returns number of notes
int8_t charstoNotes(char* commandBuffer, int8_t start, int8_t end){
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
    while(current_ptr < end){
        // Each command is either 2 or 3 characters long.
        // If the 2nd charater is # or ^, it is three characters long.
        if(commandBuffer[current_ptr+1] == '#' || commandBuffer[current_ptr+1] == '^'){
            // The 3rd char is the length.
            timeArray[note_ptr] = commandBuffer[current_ptr+2] - '0';
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
        else{
            // 2nd char is time.
            timeArray[note_ptr] = commandBuffer[current_ptr+2] - '0';
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
float charsToFloat(char* commandBuffer, int8_t start, int8_t end){
    int8_t isPositive = 1;

    float partDecimal = 0.0;
    float partWhole = 0.0;

    // By default, indexDecimal after the last char
    int8_t indexDecimal = end + 1;

    // If first char is negative, set isPositive flag, and remove the - from
    // consideration.
    if (commandBuffer[start]  == '-'){
        isPositive = -1;
        start++;
    }

    // Decimal point either second last, or third last char.
    if (commandBuffer[end - 1] == '.'){
        indexDecimal = end - 1;
        partDecimal = float(commandBuffer[end] - '0')/10.0f;
    }

    else if (commandBuffer[end - 2] == '.'){
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

//Main function
int main()
{
    pc.printf("Startup!\n\r");
    motorHigh = 0x7;
    motorLow = 0x0;

    // Input buffer
    char command[ARRAYSIZE] = {0};
    char ch;
    // For tracking input and output.
    int index = 0;
    int indexV = 0;
    bool found = false;

    // speedTimer.reset();
    // speedTimer.start();
    // I3.mode(PullNone);
    // I3.rise(&rps);

    // VPIDthread.start(VPID);
    // pc.printf("%d", VPIDthread.get_priority());
    // VPIDthread.set_priority(osPriorityAboveNormal);

    // controller.setInterval(PIDrate);
    // controller.setMode(0);

    while(1) {

        //If there's a character to read from the serial port
        if (pc.readable()) {

            //Clear counters
            index = 0;

            //Read each value from the serial port until Enter key is pressed
            do {
                //Read character
                ch = pc.getc();
                //Print character to serial for visual feedback
                pc.putc(ch);
                //Add character to input array
                command[index++] = ch;  // put it into the value array and increment the index
                //d10 and d13 used for detecting Enter key on Windows/Unix/Mac
            } while(ch != 10 && ch != 13);

            // 10 and 13 are basically EOL symbols
            // index points to after EOL, so decrement to point to EOL
            index--;

            // Detect location of V, if at all.
            indexV = 0;
            found = false;

            while(indexV <= index && !found){
                if(command[indexV] == 'V'){
                    found = true;
                }
                indexV++;
            }
            // IndexV ends one more than index of V if V exists
            // OR ends at index+1, which in this case is after EOL. Decrement.
            indexV--;
            // IndexV is now V, or EOL.

            //Start new line on terminal for printing data
            pc.putc('\n');
            pc.putc('\r');

            //Analyse the input string
            switch (command[0]) {
                // Commands to kill all threads
                // Reset all values to zero

                // Only V
                case 'V':
                    // index is EOL, index - 1 is last char
                    desiredSpeedValue = charsToFloat(command, 1, index - 1);
                    // Run thread.
                    break;

                // Is R first.
                case 'R':
                    // V also exists, because indexV before index (EOL)
                    if (indexV < index){
                        desiredRevolutions = charsToFloat(command, 1, indexV - 1);
                        desiredSpeedValue = charsToFloat(command, indexV + 1, index - 1);
                        // Run thread.
                    }
                    // Only R
                    else {
                        desiredRevolutions = charsToFloat(command, 1, index - 1);
                        // Run thread.
                    }
                    break;
                // Needs to sing
                case 'T':
                   numberNotes = charstoNotes(command, 1, index - 1);
                   // Run normal speed thread
                   singingSpeedThread.start(singingSpeed);
                   // Run singing thread
                //    playNotesThread.start(playNotes);
                    break;
                // If something weird comes along.
                default:
                    // Commands to kill all threads
                    // Reset all values to zero
                    pc.printf("Error in received data.\n\r");
                    break;
            }

            pc.printf("desiredSpeed: %3.2f ", desiredSpeedValue);
            pc.printf(" desiredRevolutions: %3.2f\n\r", desiredRevolutions);

            // Clear buffer
            for(index = 0; index < 49; index++){
              command[index] = 0;
            }
        }
        wait(0.01);
    }

}

// case 's':
//     printf("Measured: %2.3f, revsec: %2.3f\r\n", measuredRevs, revsec);
//     printf("PID: %2.3f\r\n", speedControl);
//     break;
//
// case 'K':
//     vartens = command[1] - '0';
//     varunits = command[2] - '0';
//     vardecs = command[4] - '0';
//     Kc = float(vartens)*10 + float(varunits) + float(vardecs)/10;
//     printf("Kc: %2.1f\r\n", Kc);
//     controller.setTunings(Kc, Ti, Td);
//     break;
//
// case 'i':
//     vartens = command[1] - '0';
//     varunits = command[2] - '0';
//     vardecs = command[4] - '0';
//     Ti = float(vartens)*10 + float(varunits) + float(vardecs)/10;
//     printf("Ti: %2.1f\r\n", Ti);
//     controller.setTunings(Kc, Ti, Td);
//     break;
//
// case 'd':
//     vartens = command[1] - '0';
//     varunits = command[2] - '0';
//     vardecs = command[4] - '0';
//     Td = float(vartens)*10 + float(varunits) + float(vardecs)/10;
//     printf("Td: %2.1f\r\n", Td);
//     controller.setTunings(Kc, Ti, Td);
//     break;
//
// case 'P':
//     vartens = command[1] - '0';
//     varunits = command[2] - '0';
//     vardecs = command[4] - '0';
//     PIDrate = float(vartens)*10 + float(varunits) + float(vardecs)/10;
//     controller.setInterval(PIDrate);
//     controller.setMode(1);
//     printf("Rate: %2.1f\r\n", PIDrate);
//
// case 'b':
//     if(command[1]=='.') {
//         //Extract decimal rev/s
//         vardecs = command[2] - '0';
//
//         //If decimal point is in the third character (eg, V-0.1)
//     } else if(command[2]=='.') {
//         varunits = command[1] - '0';
//         vardecs = command[3] - '0';
//
//         //If decimal point is in the fourth character (eg, V-10.1)
//     } else if(command[3]=='.') {
//         vartens = command[1] - '0';
//         varunits = command[2] - '0';
//         vardecs = command[4] - '0';
//     }
//     bias = float(vartens)*10 + float(varunits) + float(vardecs)/10;
//     printf("Bias: %2.1f\r\n", bias);
//     controller.setBias(bias);
//     break;
