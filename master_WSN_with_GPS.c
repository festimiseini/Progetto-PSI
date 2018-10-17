/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"		/* deve essere sempre incluso quando si scrive un programma */
#include <stdio.h> 		/* serve solamente per utilizzare printf e sprintf */

#include "../../stm32_nucleo_projects/sincro_WSn/project-conf.h"	/* */
#include "stm32cube_hal_init.h"	/* serve per definire la sorgente di clock utilizzata */
#include "stm32l1xx_hal.h"	/* serve per definire il tipo di nucleo che è stata utilizzata */
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "stm32l1xx_hal_uart.h"
#include "simple-udp.h"
#include "stm32l1xx_hal_gpio.h"
#include "stm32l1xx_hal_dma.h"
#include "stm32l152xe.h"
#include "stm32l1xx_hal_rcc.h"

#define HELLO_INTERVAL  1 * CLOCK_SECOND
#define UDP_PORT 1234
int condition = 0;
static struct simple_udp_connection broadcast_connection;

//////////////////////////////////////////////////////
UART_HandleTypeDef huart3;
/* Single byte to store input */
uint8_t byte;
char GPS_msg[10] = {0};
char GPS_header[6] = {0};
uint8_t header_gps = 0;
uint8_t msg_gps = 0;
uint8_t count_header_gps = 0;
uint8_t count_msg_gps = 0;
char GPS_msg_to_compare[6] = "GPGGA,";
#define UART_RxBufferSize    512
uint8_t UART_RxBuffer[UART_RxBufferSize];
/////////////////////////////////////////////////////
uip_ipaddr_t addr;
int leng;
unsigned char receiver_buffer[10];
char msg_head[]="time";
char msg_head1[]="synchro";
char msg1[128];
char msg2[128];
char hour[2]={0};
char min[2]={0};
char sec[2]={0};
char ms[3]={0};
////////////////////////////////////////////////////////

static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  printf("Data received on port %d from port %d with length %d\n",
         receiver_port, sender_port, datalen);
}

///* UART3 Interrupt Service Routine */
void USART3_IRQHandler(void)
{
	//printf("\n");//necessario!!!!!
  HAL_UART_IRQHandler(&huart3);
}


void Init_UART(){
	Uart_Init();
	Uart_Configuration();
}


void Uart_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOC_CLK_ENABLE();

  /**USART3 GPIO Configuration
  PC10     ------> USART3_TX
  PC11     ------> USART3_RX
  */
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_10;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

}


void Uart_DeInit()
{
  /*##-1- Reset peripherals ##################################################*/
 __USART3_FORCE_RESET();
 __USART3_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

  /*##-3- Disable the NVIC for UART ##########################################*/
  HAL_NVIC_DisableIRQ(USART3_IRQn);
}


void Uart_Configuration()
{

	__USART3_CLK_ENABLE();

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

  if (HAL_UART_Receive_IT(&huart3, &byte, 1) != HAL_OK)
     {
	  Error_Handler();
     }

  huart3.pRxBuffPtr = (uint8_t*)UART_RxBuffer;
  huart3.RxXferSize = UART_RxBufferSize;
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
}



void create_bodymsg_tosend(char h[],char m[],char s[],char ms[],char msgtosend[]){




}

int count=0;
/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	     if (huart->Instance == USART3)     //current UART
	          {
	             HAL_UART_Receive_IT(&huart3, &byte, 1); //activate UART receive interrupt every time
	             printf("%c",byte);
	          }


	     	               if (header_gps == 1){
	     	            	   GPS_header[count_header_gps] = byte;
	     	            	   count_header_gps++;
	     	            	   if (count_header_gps==6){
	     	            		   count_header_gps = 0;
	     	            		   header_gps = 0;
	     	            		   if (strcmp(GPS_header,GPS_msg_to_compare) == 0){

	     	            			   printf("%i\n",count);
	     	            		   }
	     	            	   }
	     	               }

	     	               if (byte == 36){
	     	            	   header_gps = 1;
	     	               }
}
//	               printf("%c",(char) byte);

//	               if (msg_gps == 1){
//	            	   GPS_msg [count_msg_gps] = byte;
//	            	   count_msg_gps++;
//	            	   if(count_msg_gps == 10){
//	            		   msg_gps = 0;
//	            		   count_msg_gps = 0;
//	            		   memset(hour, 0, sizeof(hour));
//	            		   memset(min, 0, sizeof(min));
//	            		   memset(sec, 0, sizeof(sec));
//	            		   memset(ms, 0, sizeof(ms));
//	            		   hour[0]=GPS_msg[0];
//	            		   hour[1]=GPS_msg[1];
//	            		   min[0]=GPS_msg[2];
//	            		   min[1]=GPS_msg[3];
//	            		   sec[0]=GPS_msg[4];
//	            		   sec[1]=GPS_msg[5];
//	            		   ms[0]=GPS_msg[7];
//	            		   ms[1]=GPS_msg[8];
//	            		   ms[2]=GPS_msg[9];
	            		   //create_bodymsg_tosend(hour,min,sec,ms);
	            		   //printf("hour %s\n",hour);
	            		   //printf("min %s\n",min);
	            		   //printf("sec %s\n",sec);
	            		   //printf("millisec %s\n",ms);




				   
	            		   //printf("\n\r%s",GPS_msg);
				   
//  if(condition==0){
//	strcpy(msg2, msg_head1);
//	strcat(msg2, GPS_msg);
//    uip_create_linklocal_allnodes_mcast(&addr);
//    printf("messaggio inviato %s\n",msg2);
//	simple_udp_sendto(&broadcast_connection, msg2, strlen(msg2), &addr);
//	condition=1;
//	printf("%s   %d \n\r",msg2,strlen(msg2) );
//  }
//  else if(condition==1){
//	strcpy(msg1, msg_head);
//	strcat(msg1, GPS_msg);
//    uip_create_linklocal_allnodes_mcast(&addr);
//	simple_udp_sendto(&broadcast_connection, msg1, strlen(msg1), &addr);
//	printf("%s   %d \n\r",msg1,strlen(msg1) );
//	printf("\n");
//  }
//
//	            	   }
//	               }
//	               if (header_gps == 1){
//	            	   GPS_header[count_header_gps] = byte;
//	            	   count_header_gps++;
//	            	   if (count_header_gps==6){
//	            		   count_header_gps = 0;
//	            		   header_gps = 0;
//	            		   if (strcmp(GPS_header,GPS_msg_to_compare) == 0){
//		            		   msg_gps = 1;
//	            		   }
//	            	   }
//	               }
//
//	               if (byte == 36){
//	            	   header_gps = 1;
//	               }
//	          }
//}

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(hello_world_process, ev, data)
{
	static struct etimer etimer;

	PROCESS_BEGIN();

	simple_udp_register(&broadcast_connection, UDP_PORT, NULL, UDP_PORT, receiver);
    BSP_LED_Init(LED_GREEN);
	Uart_DeInit();
	Init_UART();

	printf("usart inizializzata\n");

	while (1){

//	  etimer_set(&etimer, 10*HELLO_INTERVAL);
//	  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etimer));
//	  Init_UART();
//	  BSP_LED_On(LED_GREEN);
//	  etimer_set(&etimer, 3*HELLO_INTERVAL);
//	  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etimer));
//	  Uart_DeInit();
//	  BSP_LED_Off(LED_GREEN);

	}

  PROCESS_END();
}


void Error_Handler(void)
{
  while(1)
  {
	  printf("error\n");
  }
}


