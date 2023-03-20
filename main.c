#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "gpio.h"
#include "systick.h"
#include "tmodel.h"
#include "systick.h"
#include "rtc.h"
#include "lcd.h"
#include "uart.h"
#include "ui.h"

#define SW1      0x10

enum SW1_states
{
  SW1_WAIT,
  SW1_PRESSED,
};
enum yellow_states
{
  YELLOW_OFF,
  YELLOW_ON,
};


INT8U button_pushed()
{
    return( !(GPIO_PORTF_DATA_R & 0x10) );  // SW1 at PF4
}
/*
 * The SW1 task must signal the sw1_sem whenever the <SW1> is released.
 */

void sw1(INT8U my_id, INT8U my_state, INT8U event, INT8U data);

/*
The RED_LED_TASK must flash the red LED at frequency of 0.2 Hz.*/
void red_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data );


void yellow_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data );
/*


The GREEN_LED_TASK will toggle the green LED, whenever the yellow or the red LED is turned ON.
 *
 * */
void green_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data);

int main(void)
/*****************************************************************************
*   Input    : NONE
*   Output   : Returns 0
*   Function : Init hardware and then loop forever
******************************************************************************/
{
  init_gpio();

  uart0_init( 9600, 8, 1, 'n' );

  init_rtcs();

  open_queue( Q_UART_TX );
  open_queue( Q_UART_RX );
  open_queue( Q_LCD );
/*
  start_task( TASK_RTC, rtc_task );
  start_task( TASK_DISPLAY_RTC, display_rtc_task );
  start_task( TASK_LCD, lcd_task );
  start_task( TASK_UART_TX, uart_tx_task );
  start_task( TASK_UART_RX, uart_rx_task );
  start_task( TASK_UI, ui_task );*/

  start_task( TASK_SW1, sw1 );
  start_task( TASK_RED_LED, red_led );
  start_task( TASK_YELLOW_LED, yellow_led );
  start_task( TASK_GREEN_LED, green_led );
  schedule();
  return( 0 );
}

/*
 * The SW1 task must signal the sw1_sem whenever the <SW1> is released.
 */

void sw1(INT8U my_id, INT8U my_state, INT8U event, INT8U data )
{switch(my_state)
{
case SW1_WAIT:
    if(button_pushed())                 //if SW1 pushed
    {
        set_state(SW1_PRESSED);         //change state
    }
    break;
case SW1_PRESSED:
    if(!button_pushed())                //if button released, change state and signal
    {
        set_state(SW1_WAIT);
        preset_sem(SEM_SW1, 1);         //signal semaphore
    }
    break;
}
}
/*
The RED_LED_TASK must flash the red LED at frequency of 0.2 Hz.*/
void red_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data )
{
    GPIO_PORTF_DATA_R ^= 0x02;
    wait(1000);
    if(!(GPIO_PORTF_DATA_R & 0x02))         //if the red led was just turned on we signal to the green led
        {
            preset_sem(SEM_LED_ON, 1);
        }

}

/*
The YELLOW_LED_TASK must turn ON the yellow LED when signaled through the SW1_SEM or if nothing has happened for 3 seconds.
The yellow LED must be turned ON for 0.5 second and then turned OFF again.*/
void yellow_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data )
{
    switch(my_state)
    {
    case YELLOW_OFF:
        GPIO_PORTF_DATA_R |= 0x04; //turn off the yellow led
        set_state(YELLOW_ON);    // change state
        wait(600);               // wait for 3 seconds
        break;

    case YELLOW_ON:
    GPIO_PORTF_DATA_R &= 0xFB;  // turn on the yellow led
    set_state(YELLOW_OFF);  // change state
    preset_sem(SEM_LED_ON, 1); // Signal to the green led
    wait(100);
    break;


    }



}

void green_led(INT8U my_id, INT8U my_state, INT8U event, INT8U data)
{
    if(wait_sem(SEM_LED_ON, 0))                     //wait for signal from either red or yellow led
    {
        GPIO_PORTF_DATA_R ^= 0x08;                  //toggle green led
    }
    else wait(1);
}
