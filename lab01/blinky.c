//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// .. Leveraged from the TI Blinky example but no external dependencies
//
//*****************************************************************************

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

//
// This is a poor delay method.. Will delay appx 1us,
// Probably inaccurate
//
void
delayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

//
// This is a Very poor delay method.. Will delay appx 1ms.
// Probably inaccurate
//
void
delayMs(uint32_t ms)
{
    while(ms--)
    {
        delayUs(1000);
    }
}

int
main(void)
{
    volatile uint32_t ui32Loop;

    //
    // Enable the GPIO port that is used for the on-board LED.
    // Pause for a few moments afterwards to let the chip settle..
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    delayUs(10);

    //
    // setup the main loop delay for 1ms
    //
    ui32Loop = 1;

    //
    // Enable the GPIO pin for the LED.
    // RED (PF1) BLUE (PF2) GREEN (PF3)
    // Set the direction as output.
    // Enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R |= ((1<<3) + (1<<2) + (1<<1)); //output PF3 PF2 PF1
    GPIO_PORTF_DEN_R |= ((1<<3) + (1<<2) + (1<<1)); //enable PF3 PF2 PF1

    //
    // Enable the two switch pins as inputs and enable pullup
    // SW1 (PF4) SW2 (PF0)
    // SW2 must also be unlocked
    //

    GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock CR_R
    GPIO_PORTF_CR_R |=  (1<<0); //unlock PF0

    GPIO_PORTF_DIR_R &= ~((1<<4) + (1<<0)); //input(set to 0) PF4 PF0
    GPIO_PORTF_DEN_R |= ((1<<4) + (1<<0)); //enable PF4 PF0

    GPIO_PORTF_PUR_R |= ((1<<4) + (1<<0)); //enable pull up PF4 PF0



    //
    #define SW1 (~GPIO_PORTF_DATA_R & (1<<4))
    #define SW2 (~GPIO_PORTF_DATA_R & (1<<0))

    #define GREEN_ON (GPIO_PORTF_DATA_R |= (1<<3))
    #define BLUE_ON (GPIO_PORTF_DATA_R |= (1<<2))
    #define RED_ON (GPIO_PORTF_DATA_R |= (1<<1))

    #define GREEN_OFF (GPIO_PORTF_DATA_R &= ~(1<<3))
    #define BLUE_OFF (GPIO_PORTF_DATA_R &= ~(1<<2))
    #define RED_OFF (GPIO_PORTF_DATA_R &= ~(1<<1))
    
    //red
    uint32_t redCycleTime = 103; //red period in redCycleTime ms 
    uint32_t redLoopActive = 0;
    uint32_t heldSW1 = 0;
    int32_t redBlinkCounter = 0;    
    int32_t redBlinkTimer = 0;

    //blue
    uint32_t blueCycleTime = 119; //blue period in blueCycleTime  ms 
    uint32_t blueLoopActive = 0;
    uint32_t heldSW2 = 0;
    int32_t blueBlinkCounter = 0;    
    int32_t blueBlinkTimer = 0;

    //green
    uint32_t greenCycleTime = 1570;
    int32_t greenBlinkTimer = 0;

    //
    // Loop forever.
    //
    while(1)
    {
	//Red
	if((redLoopActive == 0) && SW1 && (heldSW1 == 0)) { //if SW1 is pressed and red is not blinking, start blinking
	    redLoopActive = 1;     //activate red loop to prevent double trigger
            redBlinkCounter = 20;  //set loop counter to 20
            redBlinkTimer = 0;     //set the period timer to 0
            heldSW1 = 1;
	}
        
        if(SW1 == 0) {
            heldSW1 = 0;
        }
        
        if(redLoopActive == 1) {
        redBlinkTimer = redBlinkTimer + 1; //increment the period timer by 1 ~every 1 ms

            if(redBlinkTimer > redCycleTime) {
            	RED_OFF;
		redBlinkCounter = redBlinkCounter - 1;
                redBlinkTimer = 0;
            }
            else if(redBlinkTimer <= (redCycleTime/2)) {
            RED_ON;
            }
            else {
            RED_OFF;
            }
	    
	    if(redBlinkCounter == 0) {
                redLoopActive = 0;
            }

        }


	//Blue
	if((blueLoopActive == 0) && SW2 && (heldSW2 == 0)) { //if SW1 is pressed and red is not blinking, start blinking
	    blueLoopActive = 1;     //activate red loop to prevent double trigger
            blueBlinkCounter = 10;  //set loop counter to 20
            blueBlinkTimer = 0;     //set the period timer to 0
            heldSW2 = 1;
	}
        
        if(SW2 == 0) {
            heldSW2 = 0;
        }
        
        if(blueLoopActive == 1) {
        blueBlinkTimer = blueBlinkTimer + 1; //increment the period timer by 1 ~every 1 ms

            if(blueBlinkTimer > blueCycleTime) {
            	BLUE_OFF;
		blueBlinkCounter = blueBlinkCounter - 1;
                blueBlinkTimer = 0;
            }
            else if(blueBlinkTimer <= (blueCycleTime/2)) {
            BLUE_ON;
            }
            else {
            BLUE_OFF;
            }
	    
	    if(blueBlinkCounter == 0) {
                blueLoopActive = 0;
            }

        }

	//Green
        greenBlinkTimer++;

	if(greenBlinkTimer > greenCycleTime) {
    	    GREEN_OFF;
            greenBlinkTimer = 0;
        }
        else if(greenBlinkTimer <= (greenCycleTime/2)) {
        GREEN_ON;
        }
        else {
        GREEN_OFF;
        }

	//loop delay
        delayMs(ui32Loop);
    }
}
