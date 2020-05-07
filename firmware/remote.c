/*
 * Remote control interface
 *
 * Copyright (c) 2020 Michael Buesch <m@bues.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "compat.h"
#include "remote.h"

#include "adc.h"
#include "arithmetic.h"
#include "crc.h"
#include "debug.h"
#include "outputsp.h"
#include "standby.h"
#include "uart.h"
#include "util.h"
#include "watchdog.h"
#include "battery.h"
#include "eeprom.h"


#define REMOTE_CHAN			UART_CHAN_8BIT_0
#define REMOTE_NR_SP			3u
#define REMOTE_STANDBY_DELAY_MS		5000u


/* Remote control message ID. */
enum remote_msg_id {
	MSGID_NOP,
	MSGID_ACK,
	MSGID_NACK,
	MSGID_PING,
	MSGID_PONG,
	MSGID_GET_CONTROL,
	MSGID_CONTROL,
	MSGID_GET_SETPOINTS,
	MSGID_SETPOINTS,
	MSGID_GET_BATVOLT,
	MSGID_BATVOLT,
};

/* Remote control message. On-wire format. */
struct remote_msg {
	uint8_t magic;
#define MSG_MAGIC	0xAAu
	uint8_t id;
	uint8_t reserved;
	union {
		struct {
		} _packed nop;

		struct {
		} _packed ack;

		struct {
		} _packed nack;

		struct {
		} _packed ping;

		struct {
		} _packed pong;

		struct {
		} _packed get_control;

		struct {
			uint8_t flags;
#define MSG_CTLFLG_ANADIS	0x01 /* Disable analog inputs. */
#define MSG_CTLFLG_EEPDIS	0x02 /* Disable EEPROM. */
		} _packed control;

		struct {
			uint8_t flags;
#define MSG_GETSPFLG_HSL	0x01 /* Get HSL instead of RGB. */
		} _packed get_setpoints;

		struct {
			uint8_t flags;
#define MSG_SPFLG_HSL		0x01 /* Set HSL instead of RGB. */
			uint8_t nr_sp;
			le16_t sp[REMOTE_NR_SP];
		} _packed setpoints;

		struct {
		} _packed get_batvolt;

		struct {
			uint16_t meas_mv;
			uint16_t drop_mv;
		} _packed batvolt;

		uint8_t padding[8];
	} _packed;
	uint8_t crc;
} _packed;


/* Remote control state. */
static struct {
	struct {
		uint8_t buf[sizeof(struct remote_msg)];
		uint8_t count;
		bool allocated;
		bool running;
	} tx;

	struct {
		uint8_t buf[sizeof(struct remote_msg)];
		uint8_t count;
		bool synchronized;
	} rx;

	uint16_t time_since_xfer_ms;
} remote;


/* Update the standby suppress state in the system.
 * Called with interrupts disabled. */
static void remote_update_standby_suppress(void)
{
	bool standby_suppress;
//	uint8_t i;
//	bool all_zero;

	if (!USE_REMOTE)
		return;

	standby_suppress = false;

	if (remote.time_since_xfer_ms < REMOTE_STANDBY_DELAY_MS)
		standby_suppress = true;

	if (!adc_analogpins_enabled()) {
/*TODO
		all_zero = true;
		for (i = 0u; i < NR_PWM; i++)
			all_zero &= output_setpoint_get(i, false) == 0u;
		if (!all_zero)
*/
			standby_suppress = true;
	}

	set_standby_suppress(STANDBY_SRC_REMOTE, standby_suppress);
}

/* Free a TX message buffer.
 * Called with interrupts disabled. */
static void put_tx_buffer(struct remote_msg *msg)
{
	/* There's only one TX buffer.
	 * Free it. */
	remote.tx.allocated = false;
}

/* Allocate a new message buffer for transmission.
 * Called with interrupts disabled. */
static struct remote_msg * get_tx_buffer(void)
{
	struct remote_msg *msg = NULL;

	if (!USE_REMOTE)
		return NULL;

	/* There's only one TX buffer.
	 * Return it, if it's not occupied. */
	if (!remote.tx.allocated) {
		remote.tx.allocated = true;

		msg = (struct remote_msg *)&remote.tx.buf[0];
		memset(msg, 0, sizeof(*msg));
		msg->magic = MSG_MAGIC;
	}

	return msg;
}

/* Transmit the next byte.
 * Called with interrupts disabled. */
static void tx_next_byte(void)
{
	if (!USE_REMOTE)
		return;

	uart_tx_byte(remote.tx.buf[remote.tx.count++],
		     REMOTE_CHAN);
	if (remote.tx.count >= sizeof(struct remote_msg)) {
		remote.tx.running = false;
		put_tx_buffer((struct remote_msg *)&remote.tx.buf[0]);
		uart_tx_enable(false, REMOTE_CHAN);
	}
}

/* Start transmission of a message.
 * Called with interrupts disabled. */
static void tx_start(struct remote_msg *msg)
{
	if (!USE_REMOTE)
		return;

	msg->crc = crc8(msg, sizeof(*msg) - 1);

	remote.tx.count = 0;
	remote.tx.running = true;

	uart_tx_enable(true, REMOTE_CHAN);
	if (uart_tx_is_ready(REMOTE_CHAN))
		tx_next_byte();

	remote.time_since_xfer_ms = 0u;
}

/* Handle a received message.
 * Called with interrupts disabled. */
static void remote_handle_rx_msg(const struct remote_msg *rxmsg)
{
	struct remote_msg *txmsg;
	struct eeprom_data *eedata;
	uint8_t crc;
	uint8_t i;

	if (!USE_REMOTE)
		return;

	if (rxmsg->magic != MSG_MAGIC) /* Invalid magic byte */
		goto error_rxmsg;
	crc = crc8(rxmsg, sizeof(*rxmsg) - 1);
	if (crc != rxmsg->crc) /* CRC error */
		goto error_rxmsg;

	eedata = eeprom_get_data();

	switch (rxmsg->id) {
	case MSGID_NOP:
		break;
	case MSGID_ACK:
		break;
	case MSGID_NACK:
		break;
	case MSGID_PING:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		txmsg->id = MSGID_PONG;

		tx_start(txmsg);
		break;
	case MSGID_PONG:
		break;
	case MSGID_GET_CONTROL:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		txmsg->id = MSGID_CONTROL;
		if (!adc_analogpins_enabled())
			txmsg->control.flags |= MSG_CTLFLG_ANADIS;

		tx_start(txmsg);
		break;
	case MSGID_CONTROL:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		/* Update control settings. */
		if (rxmsg->control.flags & MSG_CTLFLG_ANADIS)
			adc_analogpins_enable(false);
		else
			adc_analogpins_enable(true);

		/* Update settings in EEPROM. */
		if (eedata) {
			if (rxmsg->control.flags & MSG_CTLFLG_EEPDIS)
				eedata->flags |= EEPROM_FLAG_DIS;
			else
				eedata->flags &= (uint8_t)~EEPROM_FLAG_DIS;

			if (!(eedata->flags & EEPROM_FLAG_DIS)) {
				if (rxmsg->control.flags & MSG_CTLFLG_ANADIS)
					eedata->flags |= EEPROM_FLAG_ANADIS;
				else
					eedata->flags &= (uint8_t)~EEPROM_FLAG_ANADIS;
			}

			eeprom_store_data();
		}

		txmsg->id = MSGID_ACK;

		tx_start(txmsg);
		break;
	case MSGID_GET_SETPOINTS:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		txmsg->id = MSGID_NACK;

		if (rxmsg->get_setpoints.flags & MSG_GETSPFLG_HSL) {
			if (NR_PWM == REMOTE_NR_SP) {
				le16_t sp;

				for (i = 0u; i < REMOTE_NR_SP; i++) {
					sp = to_le16(output_setpoint_get(i, true));
					txmsg->setpoints.sp[i] = sp;
				}
				txmsg->setpoints.flags = MSG_SPFLG_HSL;
				txmsg->setpoints.nr_sp = REMOTE_NR_SP;

				txmsg->id = MSGID_SETPOINTS;
			}
		} else {
			if (NR_PWM <= REMOTE_NR_SP) {
				le16_t sp;

				for (i = 0u; i < NR_PWM; i++) {
					sp = to_le16(output_setpoint_get(i, false));
					txmsg->setpoints.sp[i] = sp;
				}
				txmsg->setpoints.flags = 0u;
				txmsg->setpoints.nr_sp = NR_PWM;

				txmsg->id = MSGID_SETPOINTS;
			}
		}

		tx_start(txmsg);
		break;
	case MSGID_SETPOINTS:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		txmsg->id = MSGID_NACK;

		if (rxmsg->setpoints.flags & MSG_SPFLG_HSL) {
			if (NR_PWM == REMOTE_NR_SP &&
			    rxmsg->setpoints.nr_sp == REMOTE_NR_SP) {
				uint16_t hsl[REMOTE_NR_SP];

				/* Update HSL output setpoints. */
				for (i = 0u; i < REMOTE_NR_SP; i++)
					hsl[i] = from_le16(rxmsg->setpoints.sp[i]);
				output_setpoint_convert_hsl2rgb(&hsl[0]);
				output_setpoint_set(IF_MULTIPWM(0u,) true, 0u);

				/* Update setpoints in EEPROM. */
				if (eedata && !(eedata->flags & EEPROM_FLAG_DIS)) {
					eedata->flags |= EEPROM_FLAG_SPHSL;
					build_assert(ARRAY_SIZE(eedata->setpoints) == REMOTE_NR_SP);
					for (i = 0u; i < REMOTE_NR_SP; i++)
						eedata->setpoints[i] = hsl[i];
					eeprom_store_data();
				}

				txmsg->id = MSGID_ACK;
			}
		} else {
			if (NR_PWM <= REMOTE_NR_SP &&
			    rxmsg->setpoints.nr_sp <= NR_PWM) {
				uint16_t rgb[REMOTE_NR_SP];

				/* Update RGB output setpoints. */
				for (i = 0u; i < rxmsg->setpoints.nr_sp; i++) {
					rgb[i] = from_le16(rxmsg->setpoints.sp[i]);
					output_setpoint_set(IF_MULTIPWM(i,) false, rgb[i]);
				}
				for (i = rxmsg->setpoints.nr_sp; i < NR_PWM; i++)
					output_setpoint_set(IF_MULTIPWM(i,) false, 0u);

				/* Update setpoints in EEPROM. */
				if (eedata && !(eedata->flags & EEPROM_FLAG_DIS)) {
					eedata->flags &= (uint8_t)~EEPROM_FLAG_SPHSL;
					build_assert(ARRAY_SIZE(eedata->setpoints) == REMOTE_NR_SP);
					for (i = 0u; i < rxmsg->setpoints.nr_sp; i++)
						eedata->setpoints[i] = rgb[i];
					for (i = rxmsg->setpoints.nr_sp; i < NR_PWM; i++)
						eedata->setpoints[i] = 0u;
					eeprom_store_data();
				}

				txmsg->id = MSGID_ACK;
			}
		}

		tx_start(txmsg);
		break;
	case MSGID_GET_BATVOLT:
		txmsg = get_tx_buffer();
		if (!txmsg)
			goto error_txalloc;

		txmsg->id = MSGID_BATVOLT;

		battery_get_voltage(&txmsg->batvolt.meas_mv,
				    &txmsg->batvolt.drop_mv);

		tx_start(txmsg);
		break;
	case MSGID_BATVOLT:
		break;
	default:
		goto error_rxmsg;
	}

	return;

error_txalloc:
	return;

error_rxmsg:
	remote.rx.synchronized = false;
	return;
}

/* Ready to send new byte.
 * Called with interrupts disabled. */
static void remote_tx_ready(void)
{
	if (!USE_REMOTE)
		return;

	if (remote.tx.running)
		tx_next_byte();
	else
		uart_tx_enable(false, REMOTE_CHAN);
}

/* New byte received.
 * Called with interrupts disabled. */
static void remote_rx(uint8_t data)
{
	const struct remote_msg *msg;

	if (!USE_REMOTE)
		return;

	remote.time_since_xfer_ms = 0u;

	if (remote.rx.synchronized) {
		remote.rx.buf[remote.rx.count++] = data;
		if (remote.rx.count >= sizeof(struct remote_msg)) {
			remote.rx.count = 0;
			msg = (const struct remote_msg *)&(remote.rx.buf[0]);
			remote_handle_rx_msg(msg);
		}
	} else {
		remote.rx.count = 0;
		remote.rx.buf[remote.rx.count++] = data;
		if (data == MSG_MAGIC)
			remote.rx.synchronized = true;
	}

	remote_update_standby_suppress();
}

/* Handle wake up from deep sleep.
 * Called with interrupts disabled. */
void remote_handle_deep_sleep_wakeup(void)
{
	if (!USE_REMOTE)
		return;

	remote_update_standby_suppress();
	remote.rx.synchronized = false;
}

/* Handle watchdog interrupt.
 * Called with interrupts disabled. */
void remote_handle_watchdog_interrupt(void)
{
	if (!USE_REMOTE)
		return;

	remote.time_since_xfer_ms = add_sat_u16(remote.time_since_xfer_ms,
						watchdog_interval_ms());

	remote_update_standby_suppress();
}

/* Restore settings from EEPROM data.
 * Called with interrupts disabled. */
void remote_restore_from_eeprom(void)
{
	const struct eeprom_data *eedata;
	uint8_t i;

	if (!USE_REMOTE)
		return;

	eedata = eeprom_get_data();
	if (!eedata)
		return;

	if (eedata->flags & EEPROM_FLAG_DIS)
		return; /* EEPROM is disabled. */

	if (eedata->flags & EEPROM_FLAG_ANADIS) {
		adc_analogpins_enable(false);
		if (eedata->flags & EEPROM_FLAG_SPHSL) {
			if (NR_PWM == REMOTE_NR_SP) {
				output_setpoint_convert_hsl2rgb(&eedata->setpoints[0]);
				output_setpoint_set(IF_MULTIPWM(0u,) true, 0u);
			}
		} else {
			for (i = 0u; i < NR_PWM; i++) {
				output_setpoint_set(IF_MULTIPWM(i,)
						    false,
						    eedata->setpoints[i]);
			}
		}
	} else
		adc_analogpins_enable(true);

	remote_update_standby_suppress();
}

/* Initialize remote control. */
void remote_init(void)
{
	if (!USE_REMOTE)
		return;

	build_assert(sizeof(struct remote_msg) == 12);

	remote.time_since_xfer_ms = UINT16_MAX;
	uart_register_callbacks(remote_tx_ready,
				remote_rx,
				REMOTE_CHAN);

	set_standby_suppress(STANDBY_SRC_REMOTE, false);
}
