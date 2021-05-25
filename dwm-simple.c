/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include <string.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

uint8_t i2cbyteTx[8];
uint8_t i2cbyteRx[59];
uint16_t lastDist = 0;
//uint8_t uwbData = 0;
//uint8_t sendingData = 0;
//uint8_t joinedFlag = 0;

void config_tag(void){
	int rv;
	dwm_cfg_t cfg;
	dwm_cfg_tag_t  cfg_tag;

	printf("trying to configure node as  tag\n");

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Configure device as TAG */
	cfg_tag.stnry_en = true;
	cfg_tag.loc_engine_en = true;
	cfg_tag.low_power_en = false;
	cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
	cfg_tag.common.fw_update_en = false;
	cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
	cfg_tag.common.ble_en = true;
	cfg_tag.common.led_en = false;
	cfg_tag.common.enc_en = false;

	if ((cfg.mode != DWM_MODE_TAG) ||
	(cfg.stnry_en != cfg_tag.stnry_en) ||
	(cfg.loc_engine_en != cfg_tag.loc_engine_en) ||
	(cfg.low_power_en != cfg_tag.low_power_en) ||
	(cfg.meas_mode != cfg_tag.meas_mode) ||
	(cfg.common.fw_update_en != cfg_tag.common.fw_update_en) ||
	(cfg.common.uwb_mode != cfg_tag.common.uwb_mode) ||
	(cfg.common.ble_en != cfg_tag.common.ble_en) ||
	//(cfg.common.enc_en != cfg_tag.common.enc_en) ||
	(cfg.common.led_en != cfg_tag.common.led_en)) {

		if(cfg.mode 			!= DWM_MODE_TAG) 		
         printf("mode: get = %d, set = %d\n", cfg.mode, 		DWM_MODE_ANCHOR);
		if(cfg.stnry_en     		!= cfg_tag.stnry_en)  		
         printf("acce: get = %d, set = %d\n", cfg.stnry_en, 	cfg_tag.stnry_en);
		if(cfg.loc_engine_en 		!= cfg_tag.loc_engine_en) 	
         printf("le  : get = %d, set = %d\n", cfg.loc_engine_en, cfg_tag.loc_engine_en);
		if(cfg.low_power_en		!= cfg_tag.low_power_en)	
         printf("lp  : get = %d, set = %d\n", cfg.low_power_en, 	cfg_tag.low_power_en);
		if(cfg.meas_mode 		!= cfg_tag.meas_mode) 		
         printf("meas: get = %d, set = %d\n", cfg.meas_mode, 	cfg_tag.meas_mode);
		if(cfg.common.fw_update_en 	!= cfg_tag.common.fw_update_en)	
			printf("fwup: get = %d, set = %d\n", cfg.common.fw_update_en, cfg_tag.common.fw_update_en);
		if(cfg.common.uwb_mode		!= cfg_tag.common.uwb_mode)	
         printf("uwb : get = %d, set = %d\n", cfg.common.uwb_mode, cfg_tag.common.uwb_mode);
		if(cfg.common.ble_en 		!= cfg_tag.common.ble_en)	
         printf("ble : get = %d, set = %d\n", cfg.common.ble_en, cfg_tag.common.ble_en);
		if(cfg.common.enc_en 		!= cfg_tag.common.enc_en)	
         printf("enc : get = %d, set = %d\n", cfg.common.enc_en, cfg_tag.common.enc_en);
		if(cfg.common.led_en 		!= cfg_tag.common.led_en)	
         printf("led : get = %d, set = %d\n", cfg.common.led_en, cfg_tag.common.led_en);

		APP_ERR_CHECK(rv = dwm_cfg_tag_set(&cfg_tag));

		printf("dwm_cfg_tag_set(&cfg_tag): %d \n", rv);
		dwm_reset();
	}
   	printf("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n","pass");
}
	
/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	int len;
	int i;

	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_LOC_READY:
		printf("\nT:%lu ", dwm_systime_us_get());
		if (p_evt->loc.pos_available) {
			printf("POS:[%ld,%ld,%ld,%u] ", p_evt->loc.pos.x,
					p_evt->loc.pos.y, p_evt->loc.pos.z,
					p_evt->loc.pos.qf);
		} else {
			printf("POS:N/A ");
		}

		for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
			printf("DIST%d:", i);

			printf("0x%04X", (unsigned int)(p_evt->loc.anchors.dist.addr[i] & 0xffff));
			if (i < p_evt->loc.anchors.an_pos.cnt) {
				printf("[%ld,%ld,%ld]",
						p_evt->loc.anchors.an_pos.pos[i].x,
						p_evt->loc.anchors.an_pos.pos[i].y,
						p_evt->loc.anchors.an_pos.pos[i].z);
			}

			printf("=[%lu,%u] ", p_evt->loc.anchors.dist.dist[i],
					p_evt->loc.anchors.dist.qf[i]);
					
			lastDist = p_evt->loc.anchors.dist.dist[i];
		}
		printf("\n");
		break;

	case DWM_EVT_USR_DATA_READY:
		len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
		if (len <= 0)
			break;

		printf("iot received, len=%d:", len);
		for (i = 0; i < len; ++i) {
			printf(" %02X", p_evt->usr_data[i]);
		}		
		//uwbData = p_evt->usr_data[0];
		break;

	case DWM_EVT_USR_DATA_SENT:
		//sendingData = 0;
		printf("iot sent\n");
		break;

	case DWM_EVT_BH_INITIALIZED_CHANGED:
		printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
		break;

	case DWM_EVT_UWBMAC_JOINED_CHANGED:
		printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
		//joinedFlag = 1;
		break;

	default:
		break;
	}
}

void i2c_thread(uint32_t data) {
	//uint8_t rgb_val = 100;
	int rv;
	uint16_t lastDistTemp = 0;
	
	while(1) {
		dwm_thread_delay(5); // Resolution is 10 ms => 50 ms (20 hz)
		
		/*
		memset(i2cbyteTx, 0x00, 8);
		rgb_val++;
		if(rgb_val > 100) {
			rgb_val = 0;
		}
		i2cbyteTx[0] = rgb_val;
		i2cbyteTx[1] = rgb_val;
		i2cbyteTx[2] = rgb_val;
		*/
		
		lastDistTemp = lastDist;
		if(lastDistTemp > 10000) {
			lastDistTemp = 10000;
		}
		//lastDistTemp = 10000 - lastDistTemp;
		lastDistTemp /= 100;
		i2cbyteTx[0] = lastDistTemp;
		i2cbyteTx[1] = lastDistTemp;
		i2cbyteTx[2] = lastDistTemp;
		
		//if(uwbData == 1) {
		//	i2cbyteTx[6] = 0xFF;
		//} else {
		//	i2cbyteTx[6] = 0x00;
		//}			
		
		rv = dwm_i2c_write(0x55, i2cbyteTx, 8, false);	
		dwm_thread_delay(1); // 10 ms pause needed otherwise the data read with "dwm_i2c_read" are the same as "i2cbyteTx".
		
		rv = dwm_i2c_read(0x55, i2cbyteRx, 59);
		/*
		if (rv == DWM_OK) {
			//printf("Data: %u %u %u %u %u\n", i2cbyteRx[0], i2cbyteRx[1], i2cbyteRx[2], i2cbyteRx[3], i2cbyteRx[4]);
			if(i2cbyteRx[1] >= 2) {
				uwbData = 1;
			} else {
				uwbData = 0;
			}
			if(joinedFlag == 1) {
				if(sendingData == 0) {
					sendingData = 1;
					rv = dwm_usr_data_write(&uwbData, 1, false);
					if(rv != DWM_OK) {
						printf("write err (%d)\n", rv);
					} else {
						printf("data written\n");
					}
				}
			}
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
		*/
				
	}
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
	dwm_cfg_t cfg;
	dwm_evt_t evt;
	int rv;
	uint8_t label[DWM_LABEL_LEN_MAX];
	uint8_t label_len = DWM_LABEL_LEN_MAX;
	dwm_status_t status;

	config_tag();
	
	rv = dwm_status_get(&status);
	if (rv == DWM_OK) {
		printf("loc_data: %d\n", status.loc_data);
		printf("uwbmac_joined: %d\n", status.uwbmac_joined);
		printf("bh_data_ready: %d\n", status.bh_data_ready);
		printf("bh_status_changed: %d\n", status.bh_status_changed);
		printf("bh_initialized: %d\n", status.bh_initialized);
		printf("uwb_scan_ready: %d\n", status.uwb_scan_ready);
		printf("usr_data_ready: %d\n", status.usr_data_ready);
		printf("usr_data_sent: %d\n", status.usr_data_sent);
		printf("fwup_in_progress: %d\n", status.fwup_in_progress);
	} else {
		printf("error\n");
	}	
	
	/* Initial message */
	printf(MSG_INIT);

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Update rate set to 1 second, stationary update rate set to 5 seconds */
	APP_ERR_CHECK(dwm_upd_rate_set(10, 10));

	/* Sensitivity for switching between stationary and normal update rate */
	APP_ERR_CHECK(dwm_stnry_cfg_set(DWM_STNRY_SENSITIVITY_NORMAL));

	/* Register event callback */
	dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);


	rv = dwm_label_read(label, &label_len);

	if (rv == DWM_OK) {
		printf("LABEL(len=%d):", label_len);
		for (rv = 0; rv < label_len; ++rv) {
			printf(" %02x", label[rv]);
		}
		printf("\n");
	} else {
		printf("can't read label len=%d, error %d\n", label_len, rv);
	}

	while (1) {
		rv = dwm_evt_wait(&evt);
	
		if (rv != DWM_OK) {
			printf("dwm_evt_wait, error %d\n", rv);
		} else {
			on_dwm_evt(&evt);
		}
	}
	
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl, hndl_i2c;
	int rv;

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
	
	rv = dwm_thread_create(THREAD_APP_PRIO, i2c_thread, (void*)NULL,
			"i2c", THREAD_APP_STACK_SIZE, &hndl_i2c);
	APP_ERR_CHECK(rv);	
	dwm_thread_resume(hndl_i2c);
}
