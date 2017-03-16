#include "mbed.h"
#include "rtos.h"
#include <string>
#include "PID.h"

//PID controller configuration
// float PIDrate = 0.2;
// float Kc = 10.0;
// float Ti = 3.0;
// float Td = 0;
// float speedControl = 0;
// PID controller(Kc, Ti, Td, PIDrate);
// Thread VPIDthread;

//Photointerrupter input pins
// #define I1pin D2
// #define I2pin D11
// #define I3pin D12

// //Incremental encoder input pins
// #define CHA   D7
// #define CHB   D8

// //Motor Drive output pins   //Mask in output byte
// #define L1Lpin D4           //0x01
// #define L1Hpin D5           //0x02
// #define L2Lpin D3           //0x04
// #define L2Hpin D6           //0x08
// #define L3Lpin D9           //0x10
// #define L3Hpin D10          //0x20

// //Define sized for command arrays
// #define ARRAYSIZE 8

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
//Drive state to output table
//const int8_t driveTable[6] = {0x38, 0x2C, 0x0E, 0x0B, 0x23, 0x32};

//const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};


//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
//const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};

const int8_t cwState[7] = {0x00, 0x23, 0x38, 0x32, 0x0E, 0x0B, 0x2C};
const int8_t AcwState[7] = {0x00, 0x0E, 0x23, 0x0B, 0x38, 0x2C, 0x32};


const int8_t FastStateCW[7] = {0x00, 0x32, 0x2C, 0x38, 0x0B, 0x23, 0x0E};
const int8_t FastStateACW[7] = {0x00, 0x2C, 0x0B, 0x0E, 0x32, 0x38, 0x23};

//Photointerrupter inputs
// DigitalIn I1(I1pin);
// DigitalIn I2(I2pin);
// InterruptIn I3(I3pin);

//Motor Drive outputs
// DigitalOut clk(LED1);

//NOTE, BusOut declares things in reverse (ie, 0, 1, 2, 3) compared to binary represenation
//BusOut motor(L1Hpin, L2Lpin, L2Hpin, L3Lpin, L3Hpin);//L1Lpin,
//PwmOut singpin(L1Lpin);
// BusOut motor(L1Lpin, L1Hpin, L2Lpin, L2Hpin, L3Lpin, L3Hpin);//L1Lpin,


//Timeout function for rotating at set speed
Timeout spinTimer;
float spinWait = 10;
float revsec = 0;

//Variables for spinning N revolutions
int8_t targetRevs = 0;
int8_t currRevs = 0;

//Timer used for calculating speed
Timer speedTimer;
float measuredRevs = 0, revtimer = 0;

Serial pc(SERIAL_TX, SERIAL_RX);

int8_t orState = 0;    //Rotor offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;
int8_t position = 0;

int8_t i=0;
int8_t quadraturePosition=0;
bool spinCW=0;

////Set a given drive state
//void motorOut(int8_t driveState)
//{
//
//    motor = 0x2A>>1;
//    singpin = 0;
//
//    if(!spinCW) {
//        motor = (AcwState[driveState]>>1);
//        singpin = AcwState[driveState]&&0x01;
//    } else {
//        motor = (cwState[driveState]>>1);
//        singpin = cwState[driveState]&&0x01;
//    }
//}

//Set a given drive state
// void motorOut(int8_t driveState)
// {
//     motor = 0x2A;

//     if(!spinCW) {
//         motor = AcwState[driveState];
//     } else {
//         motor = cwState[driveState];
//     }
// }

// inline void motorStop()
// {
//     //revsec set to zero prevents recurring interrupt for constant speed
//     revsec = 0;
//     wait(spinWait);
//     //0x2A turns all motor transistors off to prevent any power usage
//     motor = (0x2A>>1);
//     singpin = 0;
// }

// //Convert photointerrupter inputs to a rotor state
// inline int8_t readRotorState()
// {
//     return (I1 + I2<<1 + I3<<2);
// }

// //Basic synchronisation routine
// int8_t motorHome()
// {
//     //Put the motor in drive state 0 and wait for it to stabilise
//     motor=cwState[1];
// //    motorOut(1);
//     wait(1.0);

//     position = 0;
//     motorStop();
//     //Get the rotor state
//     return readRotorState();
// }

// void fixedSpeed()
// {
//     //If spinning is required, attach the necessary wait to the
//     //timeout interrupt to call this function again and
//     //keep the motor spinning at the right speed


//     intState = readRotorState();
//     //Increment state machine to next state
//     motorOut(intState);

// //    motorOut(I1 + I2<<1 + I3<<2);
//     if(revsec) spinTimer.attach(&fixedSpeed, spinWait);

// }

// void rps()
// {
//     speedTimer.stop();
//     revtimer = speedTimer.read_ms();
//     speedTimer.reset();
//     speedTimer.start();

//     measuredRevs = 1000/(revtimer);
//     quadraturePosition=0;
// }

// void VPID()
// {
//     while(1) {
//         controller.setProcessValue(measuredRevs);
//         speedControl = controller.compute();
//         spinWait = (1/(speedControl*6));
//         Thread::wait(PIDrate);
//     }
// }

//Set a given drive state
void singMotorOut(int8_t driveState){

    motor = 0x2A>>1;
    singpin = 0;

    motor = (cwState[driveState]>>1);
    singpin = float(cwState[driveState]&&0x01)/2;
}

// Takes freq and time, runs PWM at that freq for that time.
void pwnFreqTime(float freq, int8_t time){
    singpin.period(1.0f/freq);
    singpin.write(0.5f);
    wait(time);
    singpin.write(1.0);
}

// Just runs the motor 
void singSpinMotor(){
    while(true){
        singMotorOut(readMotorState());
        Thread::wait(5);
    }
}

//Main function
int main()
{
    pc.printf("spin\n\r");
    spinWait = 0.01;
    motorStop();

    //Run the motor synchronisation
    orState = motorHome();
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    pc.printf("Rotor origin: %x\n\r",orState);

    char command[ARRAYSIZE];
    int8_t index=0;
    int8_t units = 0, tens = 0, decimals = 0;
    char ch;
    int8_t vartens = 0, varunits = 0, vardecs = 0;
    int8_t hdrds = 0;
    float bias = 0;

    // speedTimer.reset();
    // speedTimer.start();
    I3.mode(PullNone);
    I3.rise(&rps);

    singpin.period_us(100);

    // VPIDthread.start(VPID);
    pc.printf("%d", VPIDthread.get_priority());
    // VPIDthread.set_priority(osPriorityAboveNormal);

    // controller.setInterval(PIDrate);
    // controller.setMode(0);

    while(1) {

        //If there's a character to read from the serial port
        if (pc.readable()) {

            //Clear index counter and control variables
            index = 0;

            //Read each value from the serial port until Enter key is pressed
            do {
                //Read character
                ch = pc.getc();
                //Print character to serial for visual feedback
                pc.putc(ch);
                //Add character to input array
                command[index++]=ch;  // put it into the value array and increment the index
                //d10 and d13 used for detecting Enter key on Windows/Unix/Mac
            } while(ch != 10 && ch != 13);

            //Start new line on terminal for printing data
            pc.putc('\n');
            pc.putc('\r');

            //Analyse the input string
            switch (command[0]) {
                    //If a V was typed...
                case 'V':
                    hdrds = 0, units = 0, tens = 0, decimals = 0;
                    //For each character received, subtract ASCII 0 from ASCII
                    //representation to obtain the integer value of the number
                    if(command[1]=='-') {
                        spinCW = 0;
                        //If decimal point is in the second character (eg, V-.1)
                        if(command[2]=='.') {
                            //Extract decimal rev/s
                            decimals = command[3] - '0';

                            //If decimal point is in the third character (eg, V-0.1)
                        } else if(command[3]=='.') {
                            units = command[2] - '0';
                            decimals = command[4] - '0';

                            //If decimal point is in the fourth character (eg, V-10.1)
                        } else if(command[4]=='.') {
                            tens = command[2] - '0';
                            units = command[3] - '0';
                            decimals = command[5] - '0';
                        } else if(command[4]=='.') {
                            hdrds = command[1] - '0';
                            tens = command[2] - '0';
                            units = command[3] - '0';
                            decimals = command[5] - '0';
                        }
                    } else {
                        spinCW = 1;
                        //If decimal point is in the second character (eg, V.1)
                        if(command[1]=='.') {
                            //Extract decimal rev/s
                            decimals = command[2] - '0';

                            //If decimal point is in the third character (eg, V0.1)
                        } else if(command[2]=='.') {
                            units = command[1] - '0';
                            decimals = command[3] - '0';

                            //If decimal point is in the fourth character (eg, V10.1)
                        } else if(command[3]=='.') {
                            tens = command[1] - '0';
                            units = command[2] - '0';
                            decimals = command[4] - '0';
                        } else if(command[4]=='.') {
                            hdrds = command[1] - '0';
                            tens = command[2] - '0';
                            units = command[3] - '0';
                            decimals = command[5] - '0';
                        }
                    }

                    //Calculate the number of revolutions per second required
                    revsec = float(hdrds)*100 + float(tens)*10 + float(units) + float(decimals)/10;
                    //Calculate the required wait period

                    spinWait = (1/revsec)/6;
                    controller.setSetPoint(revsec);
                    //Print values for verification
                    pc.printf("Rev/S: %2.4f\n\r", revsec);

                    //Run the function to start rotating at a fixed speed
                    fixedSpeed();
                    break;
                    //If anything unexpected was received

                case 's':
                    printf("Measured: %2.3f, revsec: %2.3f\r\n", measuredRevs, revsec);
                    printf("PID: %2.3f\r\n", speedControl);
                    break;

                case 'K':
                    vartens = command[1] - '0';
                    varunits = command[2] - '0';
                    vardecs = command[4] - '0';
                    Kc = float(vartens)*10 + float(varunits) + float(vardecs)/10;
                    printf("Kc: %2.1f\r\n", Kc);
                    controller.setTunings(Kc, Ti, Td);
                    break;
                case 'i':
                    vartens = command[1] - '0';
                    varunits = command[2] - '0';
                    vardecs = command[4] - '0';
                    Ti = float(vartens)*10 + float(varunits) + float(vardecs)/10;
                    printf("Ti: %2.1f\r\n", Ti);
                    controller.setTunings(Kc, Ti, Td);
                    break;
                case 'd':
                    vartens = command[1] - '0';
                    varunits = command[2] - '0';
                    vardecs = command[4] - '0';
                    Td = float(vartens)*10 + float(varunits) + float(vardecs)/10;
                    printf("Td: %2.1f\r\n", Td);
                    controller.setTunings(Kc, Ti, Td);
                    break;
                case 'P':
                    vartens = command[1] - '0';
                    varunits = command[2] - '0';
                    vardecs = command[4] - '0';
                    PIDrate = float(vartens)*10 + float(varunits) + float(vardecs)/10;
                    controller.setInterval(PIDrate);
                    controller.setMode(1);
                    printf("Rate: %2.1f\r\n", PIDrate);
                case 'b':
                    if(command[1]=='.') {
                        //Extract decimal rev/s
                        vardecs = command[2] - '0';

                        //If decimal point is in the third character (eg, V-0.1)
                    } else if(command[2]=='.') {
                        varunits = command[1] - '0';
                        vardecs = command[3] - '0';

                        //If decimal point is in the fourth character (eg, V-10.1)
                    } else if(command[3]=='.') {
                        vartens = command[1] - '0';
                        varunits = command[2] - '0';
                        vardecs = command[4] - '0';
                    }
                    bias = float(vartens)*10 + float(varunits) + float(vardecs)/10;
                    printf("Bias: %2.1f\r\n", bias);
                    controller.setBias(bias);
                    break;

                case 'T':
                    hdrds = 0, units = 0, tens = 0, decimals = 0;

                    //If decimal point is in the second character (eg, V.1)
                    if(command[1]=='.') {
                        //Extract decimal rev/s
                        decimals = command[2] - '0';

                        //If decimal point is in the third character (eg, V0.1)
                    } else if(command[2]=='.') {
                        units = command[1] - '0';
                        decimals = command[3] - '0';

                        //If decimal point is in the fourth character (eg, V10.1)
                    } else if(command[3]=='.') {
                        tens = command[1] - '0';
                        units = command[2] - '0';
                        decimals = command[4] - '0';
                    } else if(command[4]=='.') {
                        hdrds = command[1] - '0';
                        tens = command[2] - '0';
                        units = command[3] - '0';
                        decimals = command[5] - '0';
                    }
                    singpin.period_us(float(hdrds)*100 + float(tens)*10 + float(units) + float(decimals)/10);
                    break;

                default:
                    //Set speed variables to zero to stop motor spinning
                    //Print error message
                    motorStop();
                    pc.printf("Error in received data 0\n\r");
                    motorStop();
                    break;
            }
        }
        wait(0.01);
    }

}
