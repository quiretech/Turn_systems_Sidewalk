/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <sid_api.h>
#include <sid_error.h>

#include <state_notifier.h>
#include <board_events.h>

#if defined(CONFIG_SIDEWALK_DFU)
#include <nordic_dfu.h>
#endif

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>

#include <u_app_turn.h>

LOG_MODULE_REGISTER(board_events, CONFIG_SIDEWALK_LOG_LEVEL);


#if defined(CONFIG_TURN_APP)

void button_event_send_hello(app_ctx_t *application_ctx, turn_data_packet *data)
{

	struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err;
	static uint16_t counter = 0;

	struct sid_timespec curr_time = { 0 };
	static struct sid_msg msg;
	static struct sid_msg_desc desc;

	//data->header[0] 	= 0x00;
	//data->header[1] 	= 0x00;
	// data.data_id[0] = 0x33;
	// data.data_id[1] = 0x34;
	// data.data_id[2] = 0x35;

	// static struct sid_msg msg;
	// static struct sid_msg_desc desc;
	 sid_get_time(application_ctx->handle, SID_GET_GPS_TIME, &curr_time);	
	// //data.time = curr_time.tv_sec;
	 uint8_t i;
	 uint32_t temp_time = curr_time.tv_sec;
	// LOG_INF("Current time second is:%u",temp_time);
	//LOG_INF("Current time second is:%i",(data->time[0]));
    for(i = 0; i < 4; i++)
    {
    data->time[3-i] = (uint8_t)(temp_time >> (i*8) & 0x000000ff);
    //LOG_INF("Current time second is:%i",(data->time[3-i]));
    }

    //data->data[0] = "0091EB299763B";
    //strcpy(data->bin_id,"T3JMMV");
    //strcpy(data->data,"123456acbdefg");
    


	// //LOG_INF("Current time second is:%u",data.time);
	// LOG_INF("Size of data.time:%u",sizeof(data.time));
	// LOG_INF("Size of curr_time.sec:%u",sizeof(curr_time.tv_sec));
	// LOG_INF("Size of Data:%u",sizeof(data));
	
	// strcpy(data.bin_id,"T3JMMV");
	// data.event_id = 1;
	// strcpy(data.data,"123456acbdefg");
	// data.crc[0] = 0x10;
	// data.crc[1] = 0x01;
	// data.footer = 0xFF;



	err = sid_get_status(application_ctx->handle, &status);
	switch (err) {
	case SID_ERROR_NONE:
		break;
	case SID_ERROR_INVALID_ARGS:
		LOG_ERR("Sidewalk library is not initialzied!");
		return;
	default:
		LOG_ERR("Unknown error during sid_get_status() -> %d", err);
		return;
	}

	if (status.state != SID_STATE_READY && status.state != SID_STATE_SECURE_CHANNEL_READY) {
		LOG_ERR("Sidewalk Status is invalid!, expected SID_STATE_READY or SID_STATE_SECURE_CHANNEL_READY, got %d",
			status.state);
		return;
	}

    // NFC_EVE = 1,
    // UHF_EVE,
    // LID_OPEN,
    // LID_CLOSE,
    // BIN_75P,
    // BIN_FULL,
    // BATT_25P,
    // BATT_LOW,
    // LID_JAM,
    // SYS_CHECK,
    // SYS_OFF,
    // SID_MSG_EVE

    
uint16_t size_temp; 

switch (data->event_id){
    case NFC_EVE:
        size_temp = 31;
        break;
    case UHF_EVE:
        size_temp = 24;
        break;
    case LID_OPEN:
        size_temp = 11;
        break;
    case LID_CLOSE:
        size_temp = 11;
        break;
    case BIN_75P:
        size_temp = 12;
        break;
    case BIN_FULL:
        size_temp = 12;
        break;
    case BATT_25P:
        size_temp = 12;
        break;    
    case BATT_LOW:
        size_temp = 12;
        break;
    case LID_JAM:
        size_temp = 12;
        break;
    case SYS_CHECK:
        size_temp = 14;
        break;
    case SYS_OFF:
        size_temp = 12;
        break;
    case SID_MSG_EVE:
        size_temp = 31;
        break;
    default:
        size_temp = 25;
        break;
    }


	//msg = (struct sid_msg){ .data = (uint8_t *)&counter, .size = sizeof(uint8_t) };
	msg = (struct sid_msg){ .data = data, .size = size_temp };

 	desc = (struct sid_msg_desc){
		.type = SID_MSG_TYPE_NOTIFY,
		.link_type = SID_LINK_TYPE_ANY,
		.link_mode = SID_LINK_MODE_CLOUD,
	};

	err = sid_put_msg(application_ctx->handle, &msg, &desc);
	switch (err) {
	case SID_ERROR_NONE: {
		application_state_sending(&global_state_notifier, true);
		counter++;
		LOG_INF("queued data message id:%d", desc.id);
		break;
	}
	case SID_ERROR_TRY_AGAIN: {
		LOG_ERR("there is no space in the transmit queue, Try again.");
		break;
	}
	default:
		LOG_ERR("Unknown error returned from sid_put_msg() -> %d", err);
	}
}

#else // else for #if defined(CONFIG_TURN_APP)

void button_event_send_hello(app_ctx_t *application_ctx)
{
	struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err;

	static uint8_t counter = 0;
	static struct sid_msg msg;
	static struct sid_msg_desc desc;
    

	err = sid_get_status(application_ctx->handle, &status);
	switch (err) {
	case SID_ERROR_NONE:
		break;
	case SID_ERROR_INVALID_ARGS:
		LOG_ERR("Sidewalk library is not initialzied!");
		return;
	default:
		LOG_ERR("Unknown error during sid_get_status() -> %d", err);
		return;
	}

	if (status.state != SID_STATE_READY && status.state != SID_STATE_SECURE_CHANNEL_READY) {
		LOG_ERR("Sidewalk Status is invalid!, expected SID_STATE_READY or SID_STATE_SECURE_CHANNEL_READY, got %d",
			status.state);
		return;
	}

	msg = (struct sid_msg){ .data = (uint8_t *)&counter, .size = sizeof(uint8_t) };
	desc = (struct sid_msg_desc){
		.type = SID_MSG_TYPE_NOTIFY,
		.link_type = SID_LINK_TYPE_ANY,
		.link_mode = SID_LINK_MODE_CLOUD,
	};

	err = sid_put_msg(application_ctx->handle, &msg, &desc);
	switch (err) {
	case SID_ERROR_NONE: {
		application_state_sending(&global_state_notifier, true);
		counter++;
		LOG_INF("queued data message id:%d", desc.id);
		break;
	}
	case SID_ERROR_TRY_AGAIN: {
		LOG_ERR("there is no space in the transmit queue, Try again.");
		break;
	}
	default:
		LOG_ERR("Unknown error returned from sid_put_msg() -> %d", err);
	}
}

#endif // #if define(CONFIG_TURN_APP)

void button_event_set_battery(app_ctx_t *application_ctx)
{
	static uint8_t fake_bat_lev = 70;

	++fake_bat_lev;
	if (fake_bat_lev > 100) {
		fake_bat_lev = 0;
	}
	sid_error_t ret = sid_option(application_ctx->handle, SID_OPTION_BLE_BATTERY_LEVEL,
				     &fake_bat_lev, sizeof(fake_bat_lev));

	if (SID_ERROR_NONE != ret) {
		LOG_ERR("failed setting Sidewalk option!");
	} else {
		LOG_DBG("set battery level to %d", fake_bat_lev);
	}
}

#if defined(CONFIG_SIDEWALK_DFU_SERVICE_BLE)
void button_event_DFU(app_ctx_t *application_ctx)
{
	bool DFU_mode = true;

	(void)settings_save_one(CONFIG_DFU_FLAG_SETTINGS_KEY, (const void *)&DFU_mode,
				sizeof(DFU_mode));

	sid_deinit(application_ctx->handle);
	k_sleep(K_SECONDS(1));

	sys_reboot(SYS_REBOOT_COLD);
}

#endif /* CONFIG_SIDEWALK_DFU_SERVICE_BLE */

void button_event_factory_reset(app_ctx_t *application_ctx)
{
	sid_error_t ret = sid_set_factory_reset(application_ctx->handle);

	if (SID_ERROR_NONE != ret) {
		LOG_ERR("Notification of factory reset to sid api failed!");
	} else {
		LOG_DBG("Wait for Sid api to notify to proceed with factory reset!");
	}
}

void button_event_connection_request(app_ctx_t *application_ctx)
{
	struct sid_status status = { .state = SID_STATE_NOT_READY };
	sid_error_t err = sid_get_status(application_ctx->handle, &status);

	switch (err) {
	case SID_ERROR_NONE:
		break;
	case SID_ERROR_INVALID_ARGS:
		LOG_ERR("Sidewalk library is not initialzied!");
		return;
	default:
		LOG_ERR("Unknown error during sid_get_status() -> %d", err);
		return;
	}

	if (status.state == SID_STATE_READY) {
		LOG_WRN("Sidewalk ready, operation not valid");
		return;
	}

	bool next = !application_ctx->connection_request;

	LOG_INF("%s connection request", next ? "Set" : "Clear");
	sid_error_t ret = sid_ble_bcn_connection_request(application_ctx->handle, next);

	if (SID_ERROR_NONE == ret) {
		application_ctx->connection_request = next;
	} else {
		LOG_ERR("Connection request failed %d", ret);
	}
}
