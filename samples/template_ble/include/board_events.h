/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BOARD_EVENTS_H
#define BOARD_EVENTS_H

#include <application_thread.h>
#include <u_app_turn.h>

/**
 * @brief Application handler for sending hello message
 *
 * @param application_ctx
 */
#if defined(CONFIG_TURN_APP)
void button_event_send_hello(app_ctx_t *application_ctx,  turn_data_packet *data);
#else
void button_event_send_hello(app_ctx_t *application_ctx);
#endif //#if defined(CONFIG_TURN_APP)

/**
 * @brief Application handler for sending battery level
 *
 * @param application_ctx
 */
void button_event_set_battery(app_ctx_t *application_ctx);

/**
 * @brief Application handler for entering in DFU mode
 *
 * @param application_ctx
 */
void button_event_DFU(app_ctx_t *application_ctx);

/**
 * @brief Applicaiton handler for requesting factory reset
 *
 * @param application_ctx
 */
void button_event_factory_reset(app_ctx_t *application_ctx);

/**
 * @brief Application handler for toggling connection request state
 *
 * @param application_ctx
 */
void button_event_connection_request(app_ctx_t *application_ctx);

#endif /* BOARD_EVENTS_H */
