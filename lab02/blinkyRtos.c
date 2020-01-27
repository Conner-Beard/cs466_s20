/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"

/*local includes*/
#include "assert.h"

#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

static SemaphoreHandle_t _semBtn1 = NULL;
static SemaphoreHandle_t _semBtn2 = NULL;

uint32_t SystemCoreClock;

static void
_interruptHandlerPortF(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, 1);

    if (mask & SW1)
    {
        xSemaphoreGiveFromISR(_semBtn1, &xHigherPriorityTaskWoken);
    }
    
    if (mask & SW2)
    {       
        xSemaphoreGiveFromISR(_semBtn2, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    GPIOIntClear(GPIO_PORTF_BASE, mask);
}


static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //

    GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock CR_R
    GPIO_PORTF_CR_R |=  SW2; //unlock SW2 (PF0)

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1 );
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW2 );

    //
    // Set weak pull-up for switchs
    // This is a TiveDriver library function
    //
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz
    //                            (SYSDIV_4) 50.0 MHz
    //                            (SYSDIV_5) 40.0 MHz
    //                            (SYSDIV_6) 33.3 MHz
    //                            (SYSDIV_8) 25.0 MHz
    //                            (SYSDIV_10) 20.0 MHz
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

static void
_greenHeartbeat( void *notUsed)
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;

    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);
        
        vTaskDelay(green500ms / portTICK_RATE_MS);
    }
}

static void
_redBlink( void *notUsed)
{
    uint32_t red33ms = 33; 
    uint32_t ledOn = 0;
    uint32_t blinkCount = 0;
    uint32_t blinkActive = 0;

    BaseType_t semRes;

    //
    // Using TiveDriver Library,
    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    // Make pin B1 rising edge triggered interrupts.
    // Enable the B pin interrupts.
    //
    // This is put in the low priority task startup code because we want
    // to be very sure that the OS scheduler has started before we receive
    // any interrupts.
    //
    _semBtn1 = xSemaphoreCreateBinary();

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerPortF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW1, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTF_BASE, SW1);

    while(true)
    {
        semRes = xSemaphoreTake( _semBtn1, 0);
        
	if ((semRes == pdPASS) && (!blinkActive))
        {
            blinkCount = 20; //set up 20 blinks
	    blinkCount = blinkCount*2; //20 is really 40 for the loop below
	    blinkActive = 1;
        }
	
	if (blinkCount > 0) 
	{
            blinkCount--;         //decrement blink count
            ledOn = !ledOn;      //toggle ledOn
	    LED(LED_R, (ledOn)); //send command to LED
	}
	else
	{
	    blinkActive = 0; //allow sw to start a new cycle
	}

        vTaskDelay(red33ms / portTICK_RATE_MS);
    }
}

static void
_blueBlink( void *notUsed)
{
    uint32_t blue38ms = 38; 
    uint32_t ledOn = 0;
    uint32_t blinkCount = 0;
    uint32_t blinkActive = 0;

    BaseType_t semRes;

    //
    // Using TiveDriver Library,
    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    // Make pin B1 rising edge triggered interrupts.
    // Enable the B pin interrupts.
    //
    // This is put in the low priority task startup code because we want
    // to be very sure that the OS scheduler has started before we receive
    // any interrupts.
    //
    _semBtn2 = xSemaphoreCreateBinary();

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerPortF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW2, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTF_BASE, SW2);

    while(true)
    {
        semRes = xSemaphoreTake( _semBtn2, 0);
        
	if ((semRes == pdPASS) && (!blinkActive))
        {
            blinkCount = 10; //set up 10 blinks
	    blinkCount = blinkCount*2; //10 is really 20 for the loop below
	    blinkActive = 1;
        }
	
	if (blinkCount > 0) 
	{
            blinkCount--;        //decrement blink count
            ledOn = !ledOn;      //toggle ledOn
	    LED(LED_B, (ledOn)); //send command to LED
	}
	else
	{
	    blinkActive = 0; //allow sw to start a new cycle
	}

        vTaskDelay(blue38ms / portTICK_RATE_MS);
    }
}

int main( void )
{
    _setupHardware();

    xTaskCreate(_greenHeartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL, //parameters
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_redBlink,
                "red",
                configMINIMAL_STACK_SIZE,
                NULL, 
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );
    
    
    xTaskCreate(_blueBlink,
                "blue",
                configMINIMAL_STACK_SIZE,
                NULL, 
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );
    

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
