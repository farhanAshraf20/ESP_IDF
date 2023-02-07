/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "stddef.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_bt_defs.h"
#include "console_uart.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"
#define SPP_SHOW_DATA 0

//#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/
#define SPP_SHOW_MODE SPP_SHOW_DATA


esp_bd_addr_t peer_bd_addr = {0};
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 3;
static const uint8_t inq_num_rsps = 0;
extern uint8_t CanNotSend;

static uint8_t peer_bdname_len;
static const char remote_device_name[] = "ESP_SPP_ACCEPTOR";
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

#define W_Buff_size 125
#define R_Buff_size 127

extern uint16_t count ;
extern uint16_t count1 ;


static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;


#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
#define SPP_DATA_LEN 120  //far
#else
#define SPP_DATA_LEN ESP_SPP_MAX_MTU
#endif

static uint8_t spp_data[SPP_DATA_LEN];
uint8_t *s_p_data = NULL; /* data pointer of spp_data */
extern uint8_t t_buff[SPP_DATA_LEN];

char *f_buffer;
int temp=0;

esp_bd_addr_t rem_bda = {0};
esp_spp_cb_param_t lParam;


QueueHandle_t xQueue1;

/* Task to be created. */
void Rec_Spp_task( void * pvParameters )
{
	uint8_t R_buff[R_Buff_size]={0};

	while(1){
		if( xQueue1 != NULL ){
			if( xQueueReceive(xQueue1, &R_buff, (TickType_t)(1000/portTICK_RATE_MS))){
				printf("Received data from queue ==> %s\n", R_buff);
				vTaskDelay(100/ portTICK_RATE_MS);
			}
			//printf("Received data from queue == %s/n", R_buff);
		}
	}
	vTaskDelete(NULL);
}

/* Function that creates a task. */
void f_task_create( void )

{
	// Create queue
	xQueue1 = xQueueCreate( 10, R_Buff_size );

	/* Create the task for Spp Rx Data, storing the handle. */
	xTaskCreate(
			Rec_Spp_task,       /* Function that implements the task. */
			"utask1",          /* Text name for the task. */
			2048,      /* Stack size in words, not bytes. */
			NULL,    /* Parameter passed into the task. */
			8,/* Priority at which the task is created. */
			NULL );      /* Used to pass out the created task's handle. */
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
	uint8_t *rmt_bdname = NULL;
	uint8_t rmt_bdname_len = 0;

	if (!eir) {
		return false;
	}

	rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
	if (!rmt_bdname) {
		rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
	}

	if (rmt_bdname) {
		if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
			rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
		}

		if (bdname) {
			memcpy(bdname, rmt_bdname, rmt_bdname_len);
			bdname[rmt_bdname_len] = '\0';
		}
		if (bdname_len) {
			*bdname_len = rmt_bdname_len;
		}
		return true;
	}

	return false;
}

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
	if (bda == NULL || str == NULL || size < 18) {
		return NULL;
	}

	uint8_t *p = bda;
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
			p[0], p[1], p[2], p[3], p[4], p[5]);
	return str;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	uint8_t i = 0;
	char bda_str[18] = {0};

	switch (event) {
	case ESP_SPP_INIT_EVT:
		if (param->init.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
			esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		} else {
			ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
		}
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		if (param->disc_comp.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
			for (i = 0; i < param->disc_comp.scn_num; i++) {
				ESP_LOGI(SPP_TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
						param->disc_comp.service_name[i]);
			}
			/* We only connect to the first found server on the remote SPP acceptor here */
			esp_spp_connect(sec_mask, role_slave, param->disc_comp.scn[0], peer_bd_addr);
		} else {
			ESP_LOGE(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
		}
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
				param->close.handle, param->close.async);
		break;
	case ESP_SPP_START_EVT:
		if (param->start.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
					param->start.scn);
			esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
			esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
		} else {
			ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
		}
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
		if (param->cl_init.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT handle:%d sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
		} else {
			ESP_LOGE(SPP_TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
		}
		break;
	case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
		/*
		 * We only show the data in which the data length is less than 128 here. If you want to print the data and
		 * the data rate is high, it is strongly recommended to process them in other lower priority application task
		 * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
		 * stack and also have a effect on the throughput!
		 * Far SPP Received --> Data.
		 */
		ESP_LOGI(SPP_TAG, "SPP Received ESP_SPP_DATA_IND_EVT len:%d handle:%d",param->data_ind.len, param->data_ind.handle);

		if (param->data_ind.len < R_Buff_size) {
			uint8_t myRecv[R_Buff_size] = {0};
			strncpy((char *)myRecv, (const char *)param->data_ind.data, ((param->data_ind.len)-1));
			xQueueSend( xQueue1, (void *)&myRecv, ( TickType_t ) 10 );
		}
#else
	gettimeofday(&time_new, NULL);
	data_num += param->data_ind.len;
	if (time_new.tv_sec - time_old.tv_sec >= 3) {
		print_speed();
	}
#endif
break;
	case ESP_SPP_CONG_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
		ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong:%d", param->cong.cong);
#endif
		if (param->cong.cong == 0) {
			/* Send the privous (partial) data packet or the next data packet. */
			ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT len:%d handle:%d cong:%d", param->write.len, param->write.handle,param->write.cong);
			esp_spp_write(param->write.handle, t_buff + SPP_DATA_LEN - s_p_data, s_p_data);
			CanNotSend = param->cong.cong;
		}
		else
		{
			CanNotSend = param->cong.cong;
		}
		break;
	case ESP_SPP_WRITE_EVT:

		if (param->write.status == ESP_SPP_SUCCESS) {
			if (s_p_data + param->write.len == t_buff + SPP_DATA_LEN) {
				/* Means the previous data packet be sent completely, send a new data packet */
				s_p_data = t_buff;
			} else {
				/*
				 * Means the previous data packet only be sent partially due to the lower layer congestion, resend the
				 * remainning data.
				 */
				s_p_data += param->write.len;
			}
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
			/*
			 * We only show the data in which the data length is less than 128 here. If you want to print the data and
			 * the data rate is high, it is strongly recommended to process them in other lower priority application task
			 * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
			 * stack and also have a effect on the throughput!
			 */
			ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len:%d handle:%d cong:%d", param->write.len, param->write.handle,param->write.cong);
			if (param->write.len < W_Buff_size) {
				//esp_log_buffer_hex("", spp_data, param->write.len);
				//Delay a little to avoid the task watch dog
				vTaskDelay(pdMS_TO_TICKS(2));
			}
#else
			gettimeofday(&time_new, NULL);
			data_num += param->write.len;
			if (time_new.tv_sec - time_old.tv_sec >= 3) {
				print_speed();
			}
#endif
		} else {
			/* Means the prevous data packet is not sent at all, need to send the whole data packet again. */
			ESP_LOGE(SPP_TAG, "ESP_SPP_WRITE_EVT status:%d", param->write.status);
		}

		/*        lParam.write.cong = param->write.cong;
        lParam.write.status = param->write.status;*/
		CanNotSend = param->write.cong | param->write.status;
		memcpy(&lParam, param, sizeof(esp_spp_cb_param_t));
		if (!param->write.cong) {
			/* The lower layer is not congested, you can send the next data packet now. */
			//esp_spp_write(param->write.handle, t_buff + SPP_DATA_LEN - s_p_data, s_p_data);
		} else {
			/*
			 * The lower layer is congested now, don't send the next data packet until receiving the
			 * ESP_SPP_CONG_EVT with param->cong.cong == 0.
			 */
			;
		}

		/*
		 * If you don't want to manage this complicated process, we also provide the SPP VFS mode that hides the
		 * implementation details. However, it is less efficient and will block the caller until all data has been sent.
		 */
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		if (param->open.status == ESP_SPP_SUCCESS)
		{
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
					param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
			memset(&lParam, 0, sizeof(esp_spp_cb_param_t));
			memcpy(&lParam, param, sizeof(esp_spp_cb_param_t));
			gettimeofday(&time_old, NULL);
		}

		break;
	case ESP_SPP_SRV_STOP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
				param->close.handle, param->close.async);
		break;
	case ESP_SPP_UNINIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
		break;
	default:
		break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	char bda_str[18] = {0};
	printf("Received event: %d\n", event);
	switch (event) {
	case ESP_BT_GAP_DISC_RES_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
		esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
		/* Find the target peer device name in the EIR data */
		for (int i = 0; i < param->disc_res.num_prop; i++)
		{
			if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR)
			{
				if (get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len))
				{
					esp_log_buffer_char(SPP_TAG, peer_bdname, peer_bdname_len);
					if (strlen(remote_device_name) == peer_bdname_len && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0)
					{
						memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
						/*Have found the target peer device, cancel the previous GAP discover procedure. And go on
						 * dsicovering the SPP service on the peer device*/
						esp_bt_gap_cancel_discovery();
						esp_spp_start_discovery(peer_bd_addr);
					}
				}
			}
			if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_RSSI)
			{
				ESP_LOGI(SPP_TAG, "Read rssi  : %d", *((int8_t*)(param->disc_res.prop[i].val)));
			}
		}//far
		break;
	case ESP_BT_GAP_AUTH_CMPL_EVT:{
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
			ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
					bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
			memcpy(rem_bda, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
		} else {
			ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
		}
		break;
	}
	case ESP_BT_GAP_PIN_REQ_EVT:{
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
		if (param->pin_req.min_16_digit) {
			ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
			esp_bt_pin_code_t pin_code = {0};
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
		} else {
			ESP_LOGI(SPP_TAG, "Input pin code: 1234");
			esp_bt_pin_code_t pin_code;
			pin_code[0] = '1';
			pin_code[1] = '2';
			pin_code[2] = '3';
			pin_code[3] = '4';
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
		}
		break;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	case ESP_BT_GAP_CFM_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
		break;
	case ESP_BT_GAP_KEY_NOTIF_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
		break;
	case ESP_BT_GAP_KEY_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
		break;
#endif

	case ESP_BT_GAP_MODE_CHG_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
				bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
		if(param->mode_chg.mode == 2){
			count = 0;
			count1 = 0;
		}
		break;

	default: {
		ESP_LOGI(SPP_TAG, "event: %d", event);
		break;
	}
	}
	return;
}

void app_main(void)
{
	char bda_str[18] = {0};

	f_buffer= "farhan\n";
	for(int i=0; i<strlen(f_buffer);i++)
	{
		spp_data[i] = f_buffer[i];
	}

	//Set UART log level
	esp_log_level_set(SPP_TAG, ESP_LOG_INFO);
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_init()) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}


#if (CONFIG_BT_SSP_ENABLED == true)
	//Set default parameters for Secure Simple Pairing
	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
	if (iocap == ESP_BT_IO_CAP_IN || iocap == ESP_BT_IO_CAP_IO) {
		console_uart_init();
		vTaskDelay(pdMS_TO_TICKS(500));
	}
#endif

/*
 * Set default parameters for Legacy Pairing
 * Use variable pin, input pin code when pairing
 */
	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	esp_bt_pin_code_t pin_code;
	esp_bt_gap_set_pin(pin_type, 0, pin_code);

	ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
	f_task_create();
}
