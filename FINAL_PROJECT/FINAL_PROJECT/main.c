#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "TM4C123GH6PM.h"
#include "macros.h"

xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xLockSemaphore;
xSemaphoreHandle xMutex;
xQueueHandle xQueue;
TaskHandle_t  DriverHandle;

void ISR(void)										//interrupts for lock and jam (interrupt for the whole port, so the if conditions just show which interrupt happened
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		if(GPIO_PORTA_RIS_R == (1<<2)){
			xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );			//if xHigherPriorityTaskWoken == true, give the task with the higher 
		}																																						//priority the CPU, else stay in the same task
		else if (GPIO_PORTA_RIS_R == (1<<3)){
			xSemaphoreGiveFromISR( xLockSemaphore, &xHigherPriorityTaskWoken );		
		}
    GPIO_PORTA_ICR_R |= (1<<2) | (1<<3);		//Clear Interrupt Flag
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void TimerInit(void)
{
	SYSCTL_RCGCTIMER_R |= 0x01;
	TIMER0_CTL_R=0x00;
	TIMER0_CFG_R=0x00;
	TIMER0_TAMR_R=0x02;
	TIMER0_CTL_R=0x03;
}

void TimerDelay(int time)
{
	TIMER0_CTL_R=0x00;
	TIMER0_TAILR_R=16000*time-1;
	TIMER0_ICR_R=0x01;
	TIMER0_CTL_R |=0x03;
	while((TIMER0_RIS_R & 0x01)==0);
}


void INTERRUPTSInit(void)				//configure interrupts for both jam and lock
{
    SYSCTL_RCGCGPIO_R |= 0x01;	   			//Enable Port A
	
    __asm__("NOP; NOP; NOP; NOP;");					//while((SYSCTL_PRGPIO_R&0x00000001) == 0){};

    GPIO_PORTA_DIR_R &= ~((1 << 2) | (1 << 3));
    GPIO_PORTA_CR_R |= (1 << 2) | (1 << 3);
    GPIO_PORTA_PUR_R |= (1 << 2) | (1 << 3);	
    GPIO_PORTA_DEN_R |= (1 << 2) | (1 << 3);
	

    //Configure to detect FALLING edge
    GPIO_PORTA_IM_R &=0;
    GPIO_PORTA_IS_R &= ~((1<<2) | (1<<3));
    GPIO_PORTA_IEV_R &= ~((1<<2));
		GPIO_PORTA_IBE_R |= (1<<3);
    GPIO_PORTA_ICR_R |= (1<<2) | (1<<3);
    GPIO_PORTA_IM_R |= (1<<2) | (1<<3);	
	
	  //Enable Interrupt and set priority to 0
	  NVIC_PRI0_R |= (1<<7) | (1<<6) | (1<<5);
    NVIC_EN0_R |= (1<<0);
    
	
}


void ButtonInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x08;					    //Enable Port D
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port D as input
    GPIO_PORTD_DIR_R &= ~((1 << 0)|(1<<1)|(1<<2)|(1<<3));
    GPIO_PORTD_CR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
    GPIO_PORTD_PUR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);	
    GPIO_PORTD_DEN_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
}

void LimitInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x08;							//Enable Port D
	
    __asm__("NOP; NOP; NOP; NOP;");					//while((SYSCTL_PRGPIO_R&0x00000008) == 0){};
	
    GPIO_PORTD_DIR_R &= ~(1 << 6);
	  GPIO_PORTD_DIR_R &= ~(1 << 4);
    GPIO_PORTD_CR_R |= (1 << 6)|(1<<4);
    GPIO_PORTD_PUR_R |= (1 << 6)|(1<<4);
    GPIO_PORTD_DEN_R |= (1 << 6)|(1<<4);
	
		GPIO_PORTA_DIR_R &= ~(1 << 7);
    GPIO_PORTA_CR_R |= (1<<7);
    GPIO_PORTA_PUR_R |= (1<<7);
    GPIO_PORTA_DEN_R |= (1<<7);
}


void MotorInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x20;			    	//Enable Port F
	
    __asm__("NOP; NOP; NOP; NOP;");				//while((SYSCTL_PRGPIO_R&0x00000020) == 0){};
	
    GPIO_PORTF_DIR_R |= ((1 << 1)|(1 << 2)|(1<<3));
    GPIO_PORTF_CR_R |= (1 << 1)|(1 << 2)|(1<<3);
    GPIO_PORTF_DEN_R |= (1 << 1)|(1 << 2)|(1<<3);
}

void LockTask(void* pvParameters) 
{

    xSemaphoreTake(xLockSemaphore, 0);
    while (1) {
        xSemaphoreTake(xLockSemaphore, portMAX_DELAY);
			
				if (GET_BIT(GPIO_PORTA_DATA_R,3)==0)				//when the lock button is pressed, change the priority of the driver to be higher
				{
					GPIO_PORTF_DATA_R |= (1<<1); 
					vTaskPrioritySet(DriverHandle,2);
				}
				else if(GET_BIT(GPIO_PORTA_DATA_R,3)==1)		//else, stay the same priority
				{
					GPIO_PORTF_DATA_R &= ~(1<<1); 
					vTaskPrioritySet(DriverHandle,1);
				}
		}
}

void JamTask(void* pvParameters) 
{
    xSemaphoreTake(xBinarySemaphore, 0);
    while (1) {
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

        //Reverse motor direction
        GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
       
        TimerDelay(1000);

				//Stop motor
			  GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
   }  
}


void Queuerecieve(void* pvParameters) {
		int state;
		portBASE_TYPE xStatus;
		while(1)
		{
			xStatus = xQueueReceive(xQueue,&state,portMAX_DELAY);		
			if(state==1)					//stop the motor
			{
					GPIO_PORTF_DATA_R &= ~(1 << 3);
					GPIO_PORTF_DATA_R &= ~(1 << 2);
			}
			else if(state==2)			//open the window
			{
					GPIO_PORTF_DATA_R |= (1 << 3);
					GPIO_PORTF_DATA_R &= ~(1 << 2);
			}
			else if(state==3)			//close the window
			{
					GPIO_PORTF_DATA_R &= ~(1 << 3);
					GPIO_PORTF_DATA_R |= (1 << 2);
			}		
		}
}

void DriverTask(void* pvParameters){
		int state;
	  portBASE_TYPE xStatus;
		while(1)
		{
			xSemaphoreTake(xMutex,portMAX_DELAY );
			if (GET_BIT(GPIO_PORTD_DATA_R,0)==0)							//check if button is pressed
			{
				state=2;
				xStatus = xQueueSendToBack(xQueue,&state,0);
				vTaskDelay(1000); 
				if (GET_BIT(GPIO_PORTD_DATA_R,0)==0) 						//if button pressed >1000 ms, then window moves manually
					{
						while(GET_BIT(GPIO_PORTD_DATA_R,0)==0);
					}
     
				else if (GET_BIT(GPIO_PORTD_DATA_R,0)==1) 			//if button pressed <1000 ms, then window moves automatically
				{   					// limit switch													ON																	OFF   					(the three conditions to stop the automatic window)
							while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,0)==0 | GET_BIT(GPIO_PORTD_DATA_R,1)==0)); 
				}
				state=1;
				xStatus = xQueueSendToBack(xQueue,&state,0);
			}
			if (GET_BIT(GPIO_PORTD_DATA_R,1)==0){							//check if button is pressed
					state=3;
					xStatus = xQueueSendToBack(xQueue,&state,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,1)==0) 						//if button pressed >1000 ms, then window moves manually
					{
								while(GET_BIT(GPIO_PORTD_DATA_R,1)==0);
					}
					else if (GET_BIT(GPIO_PORTD_DATA_R,1)==1) 			//if button pressed <1000 ms, then window moves automatically
					{   						// limit switch													OFF																	ON   					(the three conditions to stop the automatic window)
								while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,1)==0 | GET_BIT(GPIO_PORTD_DATA_R,0)==0)); 
					}
					state=1;
					xStatus = xQueueSendToBack(xQueue,&state,0);
			
				}
				xSemaphoreGive(xMutex);
				TimerDelay(200);
		}
	}

void PassengerTask(void* pvParameters){
		int state;
		portBASE_TYPE xStatus;
	
		while(1)
		{
			xSemaphoreTake(xMutex,portMAX_DELAY );
			if (GET_BIT(GPIO_PORTD_DATA_R,2)==0){							//check if button is pressed
					state=2;
					xStatus = xQueueSendToBack(xQueue,&state,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,2)==0)						//if button pressed >1000 ms, then window moves manually
						{
							while(GET_BIT(GPIO_PORTD_DATA_R,2)==0);
						}
			 
					else if (GET_BIT(GPIO_PORTD_DATA_R,2)==1) 			//if button pressed <1000 ms, then window moves automatically
						{   				// limit switch													ON																	OFF   					(the three conditions to stop the automatic window)
							while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==0 | GET_BIT(GPIO_PORTD_DATA_R,3)==0)); 
						}
					state=1;
					xStatus = xQueueSendToBack(xQueue,&state,0);
			}
			
			if (GET_BIT(GPIO_PORTD_DATA_R,3)==0){							//check if button is pressed
					state=3;
					xStatus = xQueueSendToBack(xQueue,&state,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,3)==0)						//if button pressed >1000 ms, then window moves manually
					{
						while(GET_BIT(GPIO_PORTD_DATA_R,3)==0);
					}
				 
					else if (GET_BIT(GPIO_PORTD_DATA_R,3)==1) 			//if button pressed <1000 ms, then window moves automatically
					{   				// limit switch													OFF																	ON   					(the three conditions to stop the automatic window)
						while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==0 | GET_BIT(GPIO_PORTD_DATA_R,2)==0)); 
					}
					state=1;
					xStatus = xQueueSendToBack(xQueue,&state,0);
					
			}
			xSemaphoreGive(xMutex);
			vTaskDelay(100); 
		}
}

int main( void )
{
		INTERRUPTSInit();
		ButtonInit();
		LimitInit();
		MotorInit();
	  TimerInit();
	
	  xQueue = xQueueCreate(2,sizeof(int));
	  xMutex = xSemaphoreCreateMutex(); 

		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(xBinarySemaphore);
		vSemaphoreCreateBinary(xLockSemaphore);
	
	if( xBinarySemaphore != NULL )
		{
			xTaskCreate( JamTask, "jamTask", 200, NULL, 5, NULL );
			xTaskCreate( Queuerecieve, "recieveQueue", 200, NULL, 3, NULL );
			xTaskCreate( PassengerTask, "passenger", 270, NULL, 1, NULL );
			xTaskCreate( DriverTask, "driver", 270, NULL, 1, &DriverHandle );
			xTaskCreate( LockTask, "lock", 270, NULL, 4, NULL );
			vTaskStartScheduler();
		}
}