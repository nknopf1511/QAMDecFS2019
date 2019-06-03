/*
 * QAMDecFS2019.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : chaos
 */ 

//#include <avr/io.h>
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "stack_macros.h"
#include "semphr.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"

#define	PROTOCOL_BUFFER_SIZE			32

uint8_t ucGlobalProtocolBuffer_A[ PROTOCOL_BUFFER_SIZE ] = {}; // Buffer_A from Demodulator to ProtocolTask
uint8_t ucGlobalProtocolBuffer_B[ PROTOCOL_BUFFER_SIZE ] = {}; // Buffer_B from Demodulator to ProtocolTask

extern void vApplicationIdleHook( void );
void vPeak_to_bin(void *pvParameters);

TaskHandle_t peaktobinTask;
SemaphoreHandle_t xGlobalProtocolBuffer_A_Key;	//A-Resource for ucGlobalProtocolBuffer_A
SemaphoreHandle_t xGlobalProtocolBuffer_B_Key;	//A-Resource for ucGlobalProtocolBuffer_B


void vApplicationIdleHook( void )
{
	
}

int main(void)
{
	resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	xTaskCreate( vPeak_to_bin, (const char *) "Peaktobin", configMINIMAL_STACK_SIZE+10, NULL, 1, &peaktobinTask);

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"QAM-Test");
	vTaskStartScheduler();
	return 0;
}

void vPeak_to_bin(void *pvParameters) {
	(void) pvParameters;

	for(;;)
	{
		int qambit , value , i, j, l, m, a=0, b=0, x=0, y=0, t=0, u=0;  
		char Output, testwerte[32]={25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25},testwerte2[32]={25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25};
		
		if(xSemaphoreTake( xGlobalProtocolBuffer_A_Key, portMAX_DELAY )) //Buffer_A_Key Take
		{
			for (i = 0; i < 8; i++) //For Schleife mit 8 Wiederholungen für 8 Byte
			{
				for (j = 0; j < 4; j++) //For Schleife mit 4 Wiederholungen für 8 Bits (1Byte)
				{
					value = testwerte[x];
					x++;
					if((value < 10) && (value > 1)) //Die if werte sind nur ausgedacht und müssen an Tobis Ergebnisse angepasst werden
					{
						qambit = 0;
					}
					else if((value < 20) && (value > 11))
					{
						qambit = 1;
					}
					else if((value < 30) && (value > 21))
					{
						qambit = 2;;
					}
					else if((value < 40) && (value > 31))
					{
						qambit = 3;
					}
					Output = (Output << 2) | (qambit & 0x03);
				}
				ucGlobalProtocolBuffer_A[a] = Output;
				a++;
			}
			x=0;
			xSemaphoreGive(xGlobalProtocolBuffer_A_Key); //Buffer_A_Key Give
		}
		
		
		if(xSemaphoreTake( xGlobalProtocolBuffer_B_Key, portMAX_DELAY )) //Buffer_B_Key Take
		{
			for (l = 0; l < 8; l++) //For Schleife mit 8 Wiederholungen für 8 Byte
			{
				for (m = 0; m < 3; m++) //For Schleife mit 4 Wiederholungen für 8 Bits (1Byte)
				{
					value = testwerte2[y];
					y++;
					if((value < 10) && (value > 1))
					{
						qambit = 0;
					}
					else if((value < 20) && (value > 11))
					{
						qambit = 1;
					}
					else if((value < 30) && (value > 21))
					{
						qambit = 2;;
					}
					else if((value < 40) && (value > 31))
					{
						qambit = 3;
					}
					Output = (Output << 2) | (qambit & 0x03);
				}
				ucGlobalProtocolBuffer_B[b] = Output;
				b++;
			}
			y=0;
			xSemaphoreGive(xGlobalProtocolBuffer_B_Key); //Buffer_B_Key Take
		}
		
		// Testausgabe zur Ueberpruefung der Buffer-Inhalte
		for (int o = 0; o < 8; o++)
		{
			for (int k=0; k < 8; k++)
			{
				if( (ucGlobalProtocolBuffer_A[t] << k) & 0x80 )
				{
					vDisplayClear();
					vDisplayWriteStringAtPos(0,0,"1");
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				else
				{
					vDisplayClear();
					vDisplayWriteStringAtPos(0,0,"0");
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				t++;
			}
		}
		for (int p = 0; p < 8; p++)
		{
			for (int n=0; n < 8; n++)
			{
				if( (ucGlobalProtocolBuffer_B[u] << n) & 0x80 )
				{
					vDisplayClear();
					vDisplayWriteStringAtPos(1,0,"1");
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				else
				{
					vDisplayClear();
					vDisplayWriteStringAtPos(1,0,"0");
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				t++;
			}
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}