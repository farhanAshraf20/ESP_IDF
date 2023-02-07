/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include "driver/uart.h"
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "console_uart.h"
#include "app_spp_msg_prs.h"

#include "esp_spp_api.h"

#include <stdbool.h>

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define CONSOLE_UART_NUM    UART_NUM_0

#define C_W_Buff_size	122
#define R_Buff_size 	127


volatile uint16_t count = 0;
volatile uint16_t count1 = 0;
extern uint8_t *s_p_data; /* data pointer of spp_data */
uint8_t t_buff[C_W_Buff_size]={0};

uint8_t Rx_Data[R_Buff_size] = {0};

static QueueHandle_t uart_queue;
QueueHandle_t xQueue2;
extern esp_spp_cb_param_t lParam;
static spp_msg_prs_cb_t spp_msg_parser;
uint8_t CanNotSend = 0;


static const uart_config_t uart_cfg = {
		.baud_rate = 115200,   //1.5M
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 127,
};

extern void spp_msg_args_parser(char *buf, int len);

void UART_Rec_Spp_task( void * pvParameters )
{
	//uint8_t t_buff[C_W_Buff_size]={0};
	memset(t_buff,0,C_W_Buff_size);
	while(1){
		if(xQueue2 != NULL){
			if(lParam.srv_open.status == ESP_SPP_SUCCESS){
				/*if(!lParam.write.status){
					printf("SPP Write not Success[%d]\n", lParam.write.status);
				}else*/{
					if(CanNotSend){
						printf("SPP Cong[%d]\n", lParam.write.cong);
						printf("SPP Write not Success[%d]\n", lParam.write.status);
						vTaskDelay(5/ portTICK_RATE_MS);
					}else{
						if(xQueueReceive(xQueue2, &t_buff, portMAX_DELAY)== pdPASS){
							count++;
							int f_size = strlen((const char *)t_buff);
							printf("c[%d] -> SL[%d]\n",count,f_size);
							esp_spp_write(lParam.srv_open.handle, f_size, t_buff);
							s_p_data = t_buff;
						}
					}
				}
			}else{
				//printf("Fail dQueue\n");
			}
			vTaskDelay(2/ portTICK_RATE_MS);
		}
	}
	vTaskDelete(NULL);
}


void spp_msg_handler(char *buf, int len)
{
	ESP_LOGE(TAG_CNSL, "Command [%s]", buf);
	spp_msg_args_parser(buf, len);
}

static void console_uart_task(void *pvParameters)
{
	int len;
	uart_event_t event;
	spp_msg_prs_cb_t *parser = &spp_msg_parser;
	spp_msg_parser_reset_state(parser);
	spp_msg_parser_register_callback(parser, spp_msg_handler);
	spp_msg_show_usage();

	//#define TMP_BUF_LEN 120
#define TMP_BUF_LEN C_W_Buff_size

	uint8_t tmp_buf[TMP_BUF_LEN] = {0};
	uint16_t cachedLen = 0;

	for (;;) {
		//Waiting for UART event.
		if (xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			memset(tmp_buf,0,TMP_BUF_LEN);
			switch (event.type) {
			//Event of UART receving data
			case UART_DATA:
			{
				//uart_get_buffered_data_len(CONSOLE_UART_NUM, (unsigned int*)&cachedLen);
				len = uart_read_bytes(CONSOLE_UART_NUM, tmp_buf, event.size, event.timeout_flag);//(portTickType)portMAX_DELAY);portMAX_DELAY
				//ESP_LOGI(TAG_CNSL, "[UART DATA]: [%d]-->[%d]", event.size,len);

				if((event.size != 0)&&(len != 0))
				{
					count1++;
					printf("UART_DATA -> C[%d]	RL[%d]\n",count1,len);
					if(xQueueSend( xQueue2, (void *)tmp_buf,0) == pdPASS){
						uart_flush_input(CONSOLE_UART_NUM);
						memset(tmp_buf,'\0',TMP_BUF_LEN);
					}else{
						printf("Failed To Queue\n");
					}

					ESP_EARLY_LOGI("utask2", "Data waiting in myQueue : %d, Space available : %d", uxQueueMessagesWaiting(xQueue2), uxQueueSpacesAvailable(xQueue2));
				}
				break;
			}
			//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
			{
				ESP_LOGI(TAG_CNSL, "hw fifo overflow");
				break;
			}
			//Event of UART ring buffer full
			case UART_BUFFER_FULL:
			{
				uart_get_buffered_data_len(CONSOLE_UART_NUM, (unsigned int*)&cachedLen);
				ESP_LOGI(TAG_CNSL, "ring buffer full %d",cachedLen);
				break;
			}
			//Event of UART RX break detected
			case UART_BREAK:
			{
				ESP_LOGI(TAG_CNSL, "uart rx break");
				break;
			}
			//Event of UART parity check error
			case UART_PARITY_ERR:
			{
				ESP_LOGI(TAG_CNSL, "uart parity error");
				break;
			}
			//Event of UART frame error
			case UART_FRAME_ERR:
			{
				ESP_LOGI(TAG_CNSL, "uart frame error");
				break;
			}
			//Others
			default:
				break;
			}
		}
		vTaskDelay(2/ portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}


esp_err_t console_uart_init(void)
{
	esp_err_t ret;

	ret =  uart_param_config(CONSOLE_UART_NUM, &uart_cfg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG_CNSL, "Uart %d initialize err %04x", CONSOLE_UART_NUM, ret);
		return ret;
	}

	uart_set_pin(CONSOLE_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(CONSOLE_UART_NUM, 4028, 1024, 1, &uart_queue, 0);//4028
	xTaskCreate(console_uart_task, "uTask", 2048, NULL, 12, NULL);




	/* Create the task for Spp Tx Data, storing the handle. */
	xQueue2 = xQueueCreate( 20, C_W_Buff_size );
	xTaskCreate(
			UART_Rec_Spp_task,		/* Function that implements the task. */
			"utask2",				/* Text name for the task. */
			2048,      				/* Stack size in words, not bytes. */
			NULL,    				/* Parameter passed into the task. */
			12,						/* Priority at which the task is created. */
			NULL );      			/* Used to pass out the created task's handle. */

	return ESP_OK;
}
