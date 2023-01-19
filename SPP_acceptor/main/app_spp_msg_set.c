/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
# include <stdlib.h>
#include <string.h>
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "app_spp_msg_set.h"
#include "app_spp_msg_prs.h"


extern esp_bd_addr_t rem_bda;
extern esp_spp_cb_param_t lParam;
extern uint8_t Rx_Data[126];

void spp_msg_show_usage(void)
{
    printf("########################################################################\n");
    printf("\nSPP Sending and Receiving  message Using Command Line \n\n");
    printf("########################################################################\n");
}

#define SPP_CMD_HANDLER(cmd)    static void spp_##cmd##_handler(int argn, char **argv)


SPP_CMD_HANDLER(help)
{
    spp_msg_show_usage();
}

SPP_CMD_HANDLER(ok)
{
    //esp_bt_gap_ssp_confirm_reply(peer_bd_addr, true);
	esp_bt_gap_ssp_confirm_reply(rem_bda, true);
}

SPP_CMD_HANDLER(key)
{
    if (argn != 2) {
        printf("Insufficient number of arguments");
    } else {
        printf("Input Paring Key: %s\n", argv[1]);
        int passkey = atoi(argv[1]);
        esp_bt_gap_ssp_passkey_reply(rem_bda, true, passkey);
    }
}

SPP_CMD_HANDLER(send)
{
	printf("%s->\n", __func__);
	if(lParam.open.status == ESP_SPP_SUCCESS)
	{
		int f_size = strlen((const char *)Rx_Data );
		if(f_size != 0)
		{
			printf("Rx data : %s\n",Rx_Data);
			esp_spp_write(lParam.srv_open.handle, strlen((const char *)Rx_Data), Rx_Data);
		}
		printf("<-\n");
	}
}

static spp_msg_hdl_t spp_cmd_tbl[] = {
    {0,    "h",   spp_help_handler},
    {5,    "ok",  spp_ok_handler},
    {10,   "key", spp_key_handler},
	{15, "send", spp_send_handler},
};

spp_msg_hdl_t *spp_get_cmd_tbl(void)
{
    return spp_cmd_tbl;
}

size_t spp_get_cmd_tbl_size(void)
{
    return sizeof(spp_cmd_tbl) / sizeof(spp_msg_hdl_t);
}
