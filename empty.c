#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include <driverlib/pwm.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "Board.h"
#include <math.h>
#include <ti/sysbios/knl/Swi.h>
#include <time.h>


#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "driverlib/timer.h"

#include "utils/uartstdio.c"

extern const Swi_Handle swi0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore0;
extern const ti_sysbios_knl_Semaphore_Handle Semaphore1;


//for timer task
double runtime = 0;

// Task stack sizes
#define STACKSIZE   1024
#define BUFFERSIZE 20

char *BUFFER1[BUFFERSIZE];
char *BUFFER2[BUFFERSIZE];


// Define the list of supported commands and their corresponding functions


void UARTPrintInt(uint32_t num);
void cmd_RIGHTCUSTOMSPEED(double PID);
void cmd_LEFTCUSTOMSPEED(double PID);
void cmd_FORWARD(const char* args);
void cmd_BACK(const char* args);
void cmd_STOP(const char* args);
void cmd_ERR(const char* args);
void cmd_ALLFORWARDFAST(const char* args);
void cmd_RIGHTFORWARDFAST(const char* args);
void cmd_RIGHTFORWARDSLOW(const char* args);
void cmd_LEFTFORWARDSLOW(const char* args);
void cmd_LEFTFORWARDFAST(const char* args);
void cmd_RIGHTSTOP(const char* args);
void cmd_LEFTSTOP(const char* args);
void cmd_START(const char* args);
void ComputePID();

// Define a structure to hold command information
typedef struct {
    const char* command;
    void (*function)(const char*);
} Command;


// Define the list of supported commands and their corresponding functions
Command commandList[] = {
    { "fr", cmd_RIGHTFORWARDFAST },
    { "bw", cmd_BACK },
    { "st", cmd_STOP },
    { "ER", cmd_ERR },
    { "rs", cmd_RIGHTFORWARDSLOW},
    { "ls", cmd_LEFTFORWARDSLOW},
    { "lf", cmd_LEFTFORWARDFAST},
    { "er", cmd_RIGHTSTOP},
    { "el", cmd_LEFTSTOP},
    { "go", cmd_START }
};




clock_t start, end;
double cpu_time_used;


void ConfigureTimer2A(){//timer for PID controller
    // Timer 2 setup code           50 MS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);           // enable Timer 2 periph clks
    TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);        // cfg Timer 2 mode - periodic

    uint32_t ui32Period = (SysCtlClockGet() /1000);                     // period = 1/20th of a second AKA 50MS
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);         // set Timer 2 period
    TimerPrescaleSet(TIMER2_BASE, TIMER_A, 50-1);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);        // enables Timer 2 to interrupt CPU

    TimerEnable(TIMER2_BASE, TIMER_A);                      // enable Timer 2
    //UARTprintf("Timer2A \n");
}

void ConfigureTimer1A(){//timer for reflectance sensor
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    uint32_t ui32Period = (SysCtlClockGet() / 1000);                  // period = 1/100th of a second AKA 10MS
       TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);         // set Timer 2 period
       TimerPrescaleSet(TIMER1_BASE, TIMER_A, 10 - 1);
       TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);        // enables Timer 1 to interrupt CPU

      TimerEnable(TIMER1_BASE, TIMER_A);
       //UARTprintf("Timer1A \n");
}

int reflectance(){
    //UARTPrintf("Reflectance");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                              GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    int cycles = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    SysCtlDelay(SysCtlClockGet()/80000); //12.5 us

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);

    while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6){
        SysCtlDelay(SysCtlClockGet()/1000000); //1 us
        cycles++;
    }
    if(cycles > 100){//on black line
                return 1;
            }
    else if (cycles <= 100)//off black line
    {
        return 0;
    }

}
int greenFlag = 0;
int counter=0;

void BlackLineInterruptHandler() // called every 10 ms
{

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER1_BASE, TIMER_A);// reset 10 ms timer
    counter++;                                          //THIS WILL BE USED TO FIND TOTAL RUN TIME IN MAZE. INCREMENTED EVERY 10MS THE ROBOT RUNS.
    Semaphore_post(Semaphore1);

    // this is gonna start blackline task
}
int bufferToggle;
int thinline = 0;
void blackLineTask(){//attach to timer
    int currentreflectance;
    while(1){
        int cycles = 0;
        Semaphore_pend(Semaphore1, BIOS_WAIT_FOREVER); //task unblocked every 10 [ms] by timer

    while (reflectance() == 1)       //WHILE SENSOR READING BLACK. allows one cycle of white incase of error in reading. sometimes the sensor would read one white in the middle of a black line and mess everything up. so this disregards one white reading in the middle of a black line.
        {
            cycles++;

        }

    if(cycles > 70){//number of cycles for thick line
        UARTprintf("%d", counter/90);
        PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);

        unsigned long timer = 750000000/7;// 60 sec
        unsigned long flash = 5000000; //1 sec
        int led = 2;
                while(timer > 0){ //1 minute timer
                    if(timer%(flash/4) == 0){
                        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, led);//red on
                        if(led == 2){
                            led = 0;
                        }
                        else if(led == 0){
                            led = 2;
                        }
                    }
                    timer--;
                }
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

       BIOS_exit(0); //exit bios after 1 minute

            }
    else if(cycles > 5){//thin line
        thinline++;
        greenFlag = 0;

    }
    uint32_t ui32Period = (SysCtlClockGet() /1000);
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);
    }

}


int leftDutyCycle;
int rightDutyCycle;
int pwmMAX = 6250; //(1/100 Hz)/(1/(40 MHz/64))


// Function prototypes for command handlers
void cmd_FORWARD(const char* args);
void cmd_BACK(const char* args);
void cmd_STOP(const char* args);
void cmd_ERR(const char* args);

//Bluetooth Commands
void cmd_START(const char* args){

}
//sets right motor speed to go forwards
void cmd_RIGHTCUSTOMSPEED(double PID){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (PID)*pwmMAX/100);
}
//sets left motor speed to go forwards
void cmd_LEFTCUSTOMSPEED( double PID){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PID)*pwmMAX/100);
}
//sets right motor speed to go backwards
void cmd_RIGHTCUSTOMSPEEDBACK(double PID){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (PID)*pwmMAX/100);
}
//sets left motor speed to go backwards
void cmd_LEFTCUSTOMSPEEDBACK( double PID){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (PID)*pwmMAX/100);
}
//both motors go forwards
void cmd_ALLFORWARDFAST(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Right Forward Fast");
    leftDutyCycle = 50;
    rightDutyCycle = 50;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, leftDutyCycle*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}



void cmd_RIGHTFORWARDFAST(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Right Forward Fast");
    leftDutyCycle = 50;
    rightDutyCycle = 50;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}

void cmd_LEFTFORWARDFAST(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Left Forward Fast");
    leftDutyCycle = 50;
    rightDutyCycle = 50;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}

void cmd_RIGHTFORWARDSLOW(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Right Forward Slow");
    leftDutyCycle = 15;
    rightDutyCycle = 15;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}

void cmd_LEFTFORWARDSLOW(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Left Forward Slow");
    leftDutyCycle = 15;
    rightDutyCycle = 15;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}

void cmd_RIGHTSTOP(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Right Stop");
    leftDutyCycle = 0;
    rightDutyCycle = 0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}

void cmd_LEFTSTOP(const char* args) {
    int LED = 2;
    //UARTprintf("\n");
    //UARTprintf("Left Stop");
    leftDutyCycle = 0;
    rightDutyCycle = 0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, leftDutyCycle*pwmMAX/100);

    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);//14=white 8= green 4=blue 2=red
}



void cmd_BACK(const char* args) {
    int LED = 4;
    //UARTprintf("\n");
    //UARTprintf("Backwards");

    leftDutyCycle = -50;
    rightDutyCycle = -50;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, leftDutyCycle*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, rightDutyCycle*pwmMAX/100);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, leftDutyCycle*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, rightDutyCycle*pwmMAX/100);
    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void cmd_STOP(const char* args) {
    int LED = 8;
    //UARTprintf("\n");
    //UARTprintf("Stop All");
    leftDutyCycle = 0;
    rightDutyCycle = 0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, leftDutyCycle*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, rightDutyCycle*pwmMAX/100);


    //UARTprintf("\n");
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, LED);
}

void cmd_ERR(const char* args) {
    //UARTprintf("Error");
}

int findDistanceFront() {  //PE3
    uint32_t adc0Val[1];
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {}
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, adc0Val);
   //this eq converts ADC value to cm
    double adcVal = (adc0Val[0] + adc0Val[1])/2;
    // calculate distance
    double distancef = -1.857*pow(10,-9)*pow(adcVal, 3) + 1.321*pow(10, -5)*pow(adcVal, 2) - 0.03355*adcVal + 35.537;
    return distancef;
}
uint32_t adc0Val2;
int findDistanceSide() {   //PE2
    uint32_t adc0Val[1];
    ADCProcessorTrigger(ADC0_BASE, 2);
    while (!ADCIntStatus(ADC0_BASE, 2, false)) {}
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, adc0Val);
    double adcVal = (adc0Val[0] + adc0Val[1])/2;
        // calculate distance
        double distances = -1.857*pow(10,-9)*pow(adcVal, 3) + 1.321*pow(10, -5)*pow(adcVal, 2) - 0.03355*adcVal + 35.537;
        adc0Val2 = adc0Val[0];
        return distances;
}


void btConfig()
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                          GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    UARTConfigSetExpClk(
               UART1_BASE, SysCtlClockGet(), 9600,
                (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);                               //PD6RX PD7TX
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    //UARTClockSourceSet(UART2_BASE, UART_CLOCK_PIOSC);
    UARTEnable(UART1_BASE);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(1, 9600, SysCtlClockGet());

}

void ADCConfigure(){//configures ADC for distance sensors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    ADCSequenceDisable(ADC0_BASE,3);
    ADCSequenceDisable(ADC0_BASE,2);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  //PE3
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  //PE2
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 2);

}

void PWMConfigure(){//Configure PWM for both motors, sets up PWM for both forwards and Backwards

    pwmMAX = 6250; //(1/100 Hz)/(1/(40 MHz/64))

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){};//wait for PWM to be ready

    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);

    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_2,pwmMAX);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,pwmMAX);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,0*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,0*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,0*pwmMAX/100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,0*pwmMAX/100);

    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

}


// Define PID parameters
#define Kp 20
#define Ki 1
#define Kd 0.1

// Define setpoint and initial PID values

void PIDInterruptHandler()// called every 50 ms
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    Semaphore_post(Semaphore0); // this is gonna start pid task

}

int pidCycles=0;
uint16_t ping[20];
uint16_t pong[20];
uint16_t*  currentbuffer=ping;
void switchBuffer()//toggle ping/pong buffer
{
    if (currentbuffer==ping){
        currentbuffer=pong;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 10); //yellow
    }
    else {
        currentbuffer=ping;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 4); //blue
    }
}

uint8_t upperBits;
uint8_t lowerBits;

void pingPongSWIHandler()//CALLED WHEN BUFFER IS FULL
{
    UARTprintf(":6");
    int i;
    for( i=0;i<20;i++){
        upperBits = currentbuffer[i] >> 8;
        lowerBits = currentbuffer[i] & 0xFF;

        UARTprintf("%c",upperBits);//print buffer
        UARTprintf("%c",lowerBits);//print buffer
        //UARTprintf("%i\n",currentbuffer[i]);
    }
    UARTprintf("\r\n");
    switchBuffer();  //switch from ping to pong buffer or pong to ping

}


int pidCounter = 0;
int lastError = 0;
//Game plan: pass distance from front/side sensor to find
// Update PWM for moto

int flag = 0;
int bufferIndex = 0;

void ComputePID() {
    int frontTime = 0;
    while(1){

    Semaphore_pend(Semaphore0, BIOS_WAIT_FOREVER);  //task unblocked every 50 [ms] by timer


    int desired = 14;

    int frontDistance = findDistanceFront();
    int adcValSide = findDistanceSide();

    int error = adcValSide - desired; //error in cm
    uint16_t error2 = (uint16_t)(abs(adc0Val2 - 1850));
    double P = Kp*fabs(error);
    double I = Ki*(error+lastError);
    double D = Kd*(error-lastError);

    double output = (P + I + D);
    lastError = error;


    if(error > 0){
                cmd_RIGHTCUSTOMSPEED(99-output);
                cmd_LEFTCUSTOMSPEED(99);
            }
            else if(error < 0){
                cmd_RIGHTCUSTOMSPEED(99);
                cmd_LEFTCUSTOMSPEED(99-output);
            }
    //Dead End
    if(frontDistance < 10){
        TimerDisable(TIMER1_BASE, TIMER_A);
        TimerDisable(TIMER2_BASE,TIMER_A);
        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 2);
        cmd_RIGHTCUSTOMSPEED(0);
        cmd_LEFTCUSTOMSPEED(0);
        //SysCtlDelay(1000);
        cmd_LEFTCUSTOMSPEEDBACK(99);
        cmd_RIGHTCUSTOMSPEED(99);
        while(findDistanceFront() <  22);
        cmd_LEFTCUSTOMSPEED(99);
        cmd_LEFTCUSTOMSPEEDBACK(0);

        TimerEnable(TIMER1_BASE,TIMER_A);
        TimerEnable(TIMER2_BASE,TIMER_A);


    }

    if(thinline==2)//after second thin line stop collecting and printing data also set numthinlines to 999 so it wont keep re-enter this if statement. print remai
         {
          int i;
          UARTprintf(":6");
          for( i=0;i<bufferIndex;i++){
              upperBits = currentbuffer[i] >> 8;
              lowerBits = currentbuffer[i] & 0xFF;
              UARTprintf("%c", upperBits);
              UARTprintf("%c", lowerBits);
              //UARTprintf("%i\n", currentbuffer[i]);
          }
          UARTprintf("\r\n");
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);//turn off LED
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
                   thinline=999;//do this so it doesnt repeat itself every 10ms
         }

    if((pidCycles%2==0)&&(thinline==1)) //every 100[ms] (every other PID cycle) && after first thin line and before second thin line
        {
            currentbuffer[bufferIndex]= (error2);
            if((bufferIndex == 19)) //every 40 PID cycles 2[s] we call SWI to print buffer.
            {
                Swi_post(swi0);//SWI
            }
            bufferIndex = (bufferIndex + 1)%20;

            }



    if(thinline==1){
            //UARTprintf("PID CYCLES: %d",pidCycles);
         pidCycles++;
         if(greenFlag == 0){//turn LED Green after 1st thin line


         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 8); //green
         greenFlag = 1;
         }


        }

    }

}

int frontdist, sidedist;


int main(void) {
        SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
                        | SYSCTL_OSC_MAIN);

        PWMConfigure();//start PWM
        btConfig();//start bluetooth
        ADCConfigure();//start ADC
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

        //UARTprintf("Hello World\n");
        ConfigureTimer2A();//start timers
        ConfigureTimer1A();
        lastError = findDistanceSide() - 10;
        char commandString[4];
        char uartstring[2];
        while(uartstring[0] != 'g' && uartstring[1] != 'o'){


        while (!UARTCharsAvail(UART1_BASE));//wait for bluetooth start command


                               uartstring[0] = UARTCharGet(UART1_BASE); //read input
                               uartstring[1] = UARTCharGet(UART1_BASE);


                               char commandString[4];
                               strncpy(commandString, uartstring, 2);
                               commandString[2] = '\0';

                               char* receivedArgs = uartstring;

                               int i;
                               int numCommands = sizeof(commandList) / sizeof(commandList[0]);
                               for(i = 0; i < numCommands; i++){ //execute input
                                   if(strcmp(commandString, commandList[i].command) == 0){
                                       commandList[i].function(receivedArgs);
                                       break;
                                   }
                               }

                               if(i == numCommands){
                                   cmd_ERR(receivedArgs);
                               }
        }
    // Initialize TI-RTOS
    start = clock();
    BIOS_start(); // Start the RTOS scheduler
}
