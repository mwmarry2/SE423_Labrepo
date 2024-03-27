//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void ADCC_ISR(void);
__interrupt void SWI1_HighestPriority(void);
__interrupt void SWI2_MiddlePriority(void);
__interrupt void SWI3_LowestPriority(void);


// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint32_t timecount = 0;
float adcc0result = 0.0;
float adcc1result = 0.0;
float adcc2result = 0.0;
float adcc3result = 0.0;
float gyro0 = 0.0;
float gyro1 = 0.0;
float gyro2 = 0.0;
float gyro3 = 0.0;
float sum4Z = 0.0;
float sumZ = 0.0;
float sum4X = 0.0;
float sumX = 0.0;
float zero4Z = 0.0;
float zeroZ = 0.0;
float zero4X = 0.0;
float zeroX = 0.0;
uint16_t power = 0;
float controleffort = -10.0;
float controls = 0.0;
float Left = 0.0;
float Right = 0.0;
int32_t raw = 0;
float encoder = 0.0;
float distL = 0.0;
float distR = 0.0;
float uLeft = 0.0;
float uRight = 0.0;
float v1 = 0.0;
float p_current1 = 0.0;
float p_old1 = 0.0;
float v2 = 0.0;
float p_current2 = 0.0;
float p_old2 = 0.0;
float prd20=0.0;
float prd2=0.0;
float vref = 0.0;
uint32_t QEP_maxvalue = 0xFFFFFFFFU;
float Cpos = 1.866*0.6; // 1.48;
float Vpos = 2.55*0.6;//2.561;
float Cneg = -1.666*0.6;//-1.517;
float Vneg = 2.55*0.6;//2.584;
float Kp = 3.0;
float Ki = 10.0;
float errKLeft = 0.0;
float errKRight = 0.0;
float IKRight = 0.0;
float IKLeft = 0.0;
float eRight = 0.0;
float eLeft = 0.0;
float eRight_old = 0.0;
float eLeft_old = 0.0;
float IKRight_old = 0.0;
float IKLeft_old = 0.0;
float turn = 0.0;
float Kp_turn = 3.0;
float e_steer = 0.0;
//for gyro stuff
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t gyroX_raw=0;
int16_t gyroY_raw=0;
int16_t gyroZ_raw=0;
int16_t accX_raw=0;
int16_t accY_raw=0;
int16_t accZ_raw=0;
int16_t temp=0;
int16_t nottemp=0;
float gXadj=0;
float gYadj=0;
float gZadj=0;
float aXadj=0;
float aYadj=0;
float aZadj=0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float zeroY = 0;
float sumY = 0;

extern datapts ladar_data[228]; //distance data from LADAR

extern float printLV1;
extern float printLV2;

extern float LADARrightfront;
extern float LADARfront;

extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern char G_command[]; //command for getting distance -120 to 120 degree
extern uint16_t G_len; //length of command
extern xy ladar_pts[228]; //xy data

extern uint16_t LADARpingpong;
extern float LADARxoffset;
extern float LADARyoffset;

extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

extern uint16_t NewLVData;

uint16_t LADARi = 0;
pose ROBOTps = {0,0,0}; //robot position
pose LADARps = {3.5/12.0,0,1};  // 3.5/12 for front mounting, theta is not used in this current code
float printLinux1 = 0;
float printLinux2 = 0;

float calc_v(float p_current, float p_old){ //(pos1-pos2)/time =velocity function
    float v = (p_current-p_old)/0.001;
    return v;
}
void setEPWM1A(float controleffort){ //Wheel connected to EPWM1A
    if (controleffort > 10){ //can't go above 10 otherwise floors to 10
        controleffort = 10.0;
    }
    if (controleffort < -10){//can't go below -10 otherwise floors to -10
        controleffort = -10.0;
    }
    prd20=EPwm1Regs.TBPRD*1/20; //breaks period up into 20 parts. 10 above 10 below 0
    prd2=EPwm1Regs.TBPRD*1/2; //this is needed bc duty cycke of 50 stops driving the wheels so this like our reference zero
    controls = controleffort*prd20+prd2; //this figures out using the controlseffort variable how much togive the motor based on the input.
    EPwm1Regs.CMPA.bit.CMPA = controls; //sets the wheel speed
}
void setEPWM2A(float controleffort){//Wheel connected to EPWM1A
    if (controleffort > 10){//can't go above 10 otherwise floors to 10
        controleffort = 10.0;
    }
    if (controleffort < -10){//can't go below -10 otherwise floors to -10
        controleffort = -10.0;
    }
    float controls = controleffort*EPwm2Regs.TBPRD/20+EPwm2Regs.TBPRD/2;//this figures out using the controlseffort variable how much togive the motor based on the input.
    EPwm2Regs.CMPA.bit.CMPA = controls; //sets the wheel speed

}
void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1; // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT; //gives the raw value
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/40000));// returns the left enc
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT; //gives the raw value
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/40000));// returns the right enc
}
float readEncWheel(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep3Regs.QPOSCNT; //gives the raw value
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/4000)); //returns loc of the wheelenc
}
void PostSWI1(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI1
}
void PostSWI3(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1; // Manually cause the interrupt for the SWI3
}
void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LS7366#2 CS
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    //PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.ADCC1_INT = &ADCC_ISR;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.EMIF_ERROR_INT = &SWI1_HighestPriority;
    PieVectTable.RAM_CORRECTABLE_ERROR_INT = &SWI2_MiddlePriority;
    PieVectTable.FLASH_CORRECTABLE_ERROR_INT = &SWI3_LowestPriority;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    DELAY_US(1000000);  // Delay 1 second giving LADAR Time to power on after system power on

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
    init_serialSCIC(&SerialC,19200);
    init_serialSCID(&SerialD,2083332);

    for (LADARi = 0; LADARi < 228; LADARi++) {
        ladar_data[LADARi].angle = ((3*LADARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
    }
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); //GPIO Pinname, CPU, Mux Index
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm12Regs.TBCTL.bit.CTRMODE = 0;
    EPwm12Regs.TBCTL.bit.CLKDIV = 0;
    EPwm12Regs.TBCTR = 0;
    EPwm12Regs.TBPRD = 2500;
    EPwm12Regs.CMPA.bit.CMPA = 0;
    EPwm12Regs.AQCTLA.bit.CAU = 1;
    EPwm12Regs.AQCTLA.bit.ZRO = 2;
    EPwm12Regs.TBPHS.bit.TBPHS = 0;

    //BLEOW IS THE SETUP FOR THE 1st SET OF WHEELS
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 1;
    EPwm1Regs.AQCTLA.bit.ZRO = 2;
    EPwm1Regs.TBPHS.bit.TBPHS = 0;
    //BLEOW IS THE SETUP FOR THE 2nd SET OF WHEELS
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    init_eQEPs();
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1; // For EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWMA2
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;
    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    //ADCC
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    DELAY_US(1000);

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0xB; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0xB; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 0x4; //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0xB; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 0x5; //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0xB; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0x3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT2; //for ADCB
    IER |= M_INT3;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable 3rd interrupt source of interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1; //SWI1
    PieCtrlRegs.PIEIER12.bit.INTx10 = 1; //SWI2
    PieCtrlRegs.PIEIER12.bit.INTx11 = 1; //SWI3  Lowest priority

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    char S_command[19] = "S1152000124000\n";//this change the baud rate to 115200
    uint16_t S_len = 19;
    serial_sendSCIC(&SerialC, S_command, S_len);


    DELAY_US(1000000);  // Delay letting LADAR change its Baud rate
    init_serialSCIC(&SerialC,115200);

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"aX:%.2f,aY:%.2f,aZ:%.2f gX: %.2f gY: %.2f gZ: %.2f \r\n",aXadj,aYadj,aZadj, gXadj, gYadj, gZadj);
            UART_printfLine(1,"gZ:%.2f,g4Z:%.2f",gZadj,gyro0);
            UART_printfLine(2,"VL:%.2f,VR:%.2f",v1,v2);
            UARTPrint = 0;}
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt


// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%5) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    serial_sendSCIC(&SerialC, G_command, G_len);
    UARTPrint = 1;
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        //UARTPrint = 1;
    }
}
__interrupt void SPIB_isr(void){
//    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.
//    nottemp=SpibRegs.SPIRXBUF;
//    accX_raw=SpibRegs.SPIRXBUF;
//    accY_raw=SpibRegs.SPIRXBUF;
//    accZ_raw=SpibRegs.SPIRXBUF;
//    temp=SpibRegs.SPIRXBUF;
//    gyroX_raw=SpibRegs.SPIRXBUF;
//    gyroY_raw=SpibRegs.SPIRXBUF;
//    gyroZ_raw=SpibRegs.SPIRXBUF;
//    //spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
//    //spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
//    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
//    gXadj=gyroX_raw/32767.0*250.0+6.3;
//    gYadj=gyroY_raw/32767.0*250.0-2.80;
//    gZadj=gyroZ_raw/32767.0*250.0-.78;
//    aXadj=accX_raw/32767.0*4.0;
//    aYadj=accY_raw/32767.0*4.0;
//    aZadj=accZ_raw/32767.0*4.0;
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

//
//    //_____________________________
//
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.
    nottemp=SpibRegs.SPIRXBUF;
    accX_raw=SpibRegs.SPIRXBUF;
    accY_raw=SpibRegs.SPIRXBUF;
    accZ_raw=SpibRegs.SPIRXBUF;
    temp=SpibRegs.SPIRXBUF;
    gyroX_raw=SpibRegs.SPIRXBUF;
    gyroY_raw=SpibRegs.SPIRXBUF;
    gyroZ_raw=SpibRegs.SPIRXBUF;

    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
    gXadj=gyroX_raw/32767.0*250.0 - zeroX;
    gYadj=gyroY_raw/32767.0*250.0 - zeroY;
    gZadj=gyroZ_raw/32767.0*250.0 - zeroZ;
    aXadj=accX_raw/32767.0*4.0;
    aYadj=accY_raw/32767.0*4.0;
    aZadj=accZ_raw/32767.0*4.0;
    PostSWI1();
    SPBISRInterruptCount++;

    // Acknowledge this interrupt to receive more interrupts from group 1
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

}
__interrupt void ADCC_ISR(void){

    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
{

    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    uint16_t i = 0;//for loop


    if (timecount < 1000){
        //this function makes it so for the first second all the gyros report zero so that we don't
        //have crazy values from initial noise that will mess up the normalization
        gyroX = 0;
        gyroY = 0;
        gyroZ = 0;
    }
    else if (timecount < 3000){
        //Between 1s and 3 seconds we are taking the gyro values and converting them to voltages

        gyroX = gyroX_raw/32767.0*250.0;
        gyroY = gyroY_raw/32767.0*250.0;
        gyroZ = gyroZ_raw/32767.0*250.0;
        //Once we have these voltages we add them to a sum which keeps track of all the gyros sum between 1 and 3 seconds
        sumX += gyroX;
        sumY += gyroY;
        sumZ += gyroZ;
        zeroX = sumX/2000;
        zeroY = sumY/2000;
        zeroZ = sumZ/2000;
    }

    if (newLinuxCommands == 1) {
        newLinuxCommands = 0;
        printLinux1 = LinuxCommands[0];
        printLinux2 = LinuxCommands[1];
        //value3 = LinuxCommands[2];
        //value4 = LinuxCommands[3];
        //value5 = LinuxCommands[4];
        //value6 = LinuxCommands[5];
        //value7 = LinuxCommands[6];
        //value8 = LinuxCommands[7];
        //value9 = LinuxCommands[8];
        //value10 = LinuxCommands[9];
        //value11 = LinuxCommands[10];
    }


    if (NewLVData == 1) {
        NewLVData = 0;
        printLV1 = fromLVvalues[0];
        printLV2 = fromLVvalues[1];
    }

    if((timecount%250) == 0) {
        DataToLabView.floatData[0] = ROBOTps.x;
        DataToLabView.floatData[1] = ROBOTps.y;
        DataToLabView.floatData[2] = (float)timecount;
        DataToLabView.floatData[3] = ROBOTps.theta;
        DataToLabView.floatData[4] = ROBOTps.theta;
        DataToLabView.floatData[5] = ROBOTps.theta;
        DataToLabView.floatData[6] = ROBOTps.theta;
        DataToLabView.floatData[7] = ROBOTps.theta;
        LVsenddata[0] = '*';  // header for LVdata
        LVsenddata[1] = '$';
        for (i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }





    //##############################################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
//
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    }




    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
//
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}
