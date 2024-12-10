/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements the GDB Remote Serial Debugging protocol packet
 * reception and transmission as well as some convenience functions.
 */

#include "general.h"
#include "gdb_if.h"
#include "gdb_packet.h"
#include "hex_utils.h"
#include "remote.h"

#include <stdarg.h>

typedef enum packet_state {
	PACKET_IDLE,
	PACKET_GDB_CAPTURE,
	PACKET_GDB_ESCAPE,
	PACKET_GDB_CHECKSUM_UPPER,
	PACKET_GDB_CHECKSUM_LOWER,
	PACKET_BMD_REMOTE_CAPTURE,
	PACKET_COMPLETE,
} packet_state_e;

static bool noackmode = false;

gdb_packet_s *gdb_packet_buffer(void)
{
	/* This has to be aligned so the remote protocol can re-use it without causing Problems */
	static gdb_packet_s BMD_ALIGN_DEF(8) packet_buffer;
	return &packet_buffer;
}

/* https://sourceware.org/gdb/onlinedocs/gdb/Packet-Acknowledgment.html */
void gdb_set_noackmode(bool enable)
{
	/*
	 * If we were asked to disable NoAckMode, and it was previously enabled,
	 * it might mean we got a packet we determined to be the first of a new
	 * GDB session, and as such it was not acknowledged (before GDB enabled NoAckMode),
	 * better late than never.
	 *
	 * If we were asked after the connection was terminated, sending the ack will have no effect.
	 */
	if (!enable && noackmode)
		gdb_if_putchar(GDB_PACKET_ACK, true);

	/* Log only changes */
	if (noackmode != enable)
		DEBUG_GDB("%s NoAckMode\n", enable ? "Enabling" : "Disabling");

	noackmode = enable;
}

#if ENABLE_DEBUG == 1
static void gdb_packet_debug(const char *const func, const gdb_packet_s *const packet)
{
	/* Log packet for debugging */
	DEBUG_GDB("%s: ", func);
	for (size_t i = 0; i < packet->size; i++) {
		const char value = packet->data[i];
		if (value >= ' ' && value < '\x7f')
			DEBUG_GDB("%c", value);
		else
			DEBUG_GDB("\\x%02X", (uint8_t)value);
	}
	DEBUG_GDB("\n");
}
#endif

static inline bool gdb_packet_is_reserved(const char character, gdb_packet_type_e type)
{
	/* Check if the character is a reserved GDB packet character */
	if (type == GDB_PACKET_BMD_REMOTE)
		return character == GDB_PACKET_START || character == REMOTE_RESP || character == REMOTE_SOM ||
			character == REMOTE_EOM;
	else
		return character == GDB_PACKET_START || character == GDB_PACKET_END || character == GDB_PACKET_ESCAPE ||
			character == GDB_PACKET_RUNLENGTH_START;
}

static uint8_t gdb_packet_checksum(const gdb_packet_s *const packet)
{
	/* Calculate the checksum of the packet */
	uint8_t checksum = 0;
	for (size_t i = 0; i < packet->size; i++) {
		const char character = packet->data[i];
		if (gdb_packet_is_reserved(character, packet->type))
			checksum += GDB_PACKET_ESCAPE + (character ^ GDB_PACKET_ESCAPE_XOR);
		else
			checksum += character;
	}
	return checksum;
}

gdb_packet_s *gdb_packet_receive(void)
{
	packet_state_e state = PACKET_IDLE; /* State of the packet capture */
	uint8_t rx_checksum = 0;

	/* Get the packet buffer */
	gdb_packet_s *const packet = gdb_packet_buffer();
	while (true) {
		const char rx_char = gdb_if_getchar();

		switch (state) {
		case PACKET_IDLE:
			if (rx_char == GDB_PACKET_START) {
				/* Start of GDB packet */
				state = PACKET_GDB_CAPTURE;
				packet->size = 0;
				packet->type = GDB_PACKET_STANDARD;
			}
#if CONFIG_BMDA == 0
			else if (rx_char == REMOTE_SOM) {
				/* Start of BMP remote packet */
				state = PACKET_BMD_REMOTE_CAPTURE;
				packet->size = 0;
				packet->type = GDB_PACKET_BMD_REMOTE;
			}
#endif
			/* EOT (end of transmission) - connection was closed */
			else if (rx_char == '\x04') {
				packet->data[0U] = rx_char;
				packet->data[1U] = '\0'; /* Null terminate */
				packet->size = 1U;
				packet->type = GDB_PACKET_UNDEFINED;
				return packet;
			}
			break;

		case PACKET_GDB_CAPTURE:
			if (rx_char == GDB_PACKET_START) {
				/* Restart GDB packet capture */
				packet->size = 0;
				break;
			}
			if (rx_char == GDB_PACKET_END) {
				/* End of GDB packet */

				/* Move to checksum capture */
				state = PACKET_GDB_CHECKSUM_UPPER;
				break;
			}

			/* Add to packet buffer, unless it is an escape char */
			if (rx_char == GDB_PACKET_ESCAPE)
				/* GDB Escaped char */
				state = PACKET_GDB_ESCAPE;
			else
				/* Add to packet buffer */
				packet->data[packet->size++] = rx_char;
			break;

		case PACKET_GDB_ESCAPE:
			/* Resolve escaped char */
			packet->data[packet->size++] = rx_char ^ GDB_PACKET_ESCAPE_XOR;

			/* Return to normal packet capture */
			state = PACKET_GDB_CAPTURE;
			break;

		case PACKET_GDB_CHECKSUM_UPPER:
			/* Checksum upper nibble */
			if (!noackmode)
				/* As per GDB spec, checksums can be ignored in NoAckMode */
				rx_checksum = unhex_digit(rx_char) << 4U; /* This also clears the lower nibble */
			state = PACKET_GDB_CHECKSUM_LOWER;
			break;

		case PACKET_GDB_CHECKSUM_LOWER:
			/* Checksum lower nibble */
			if (!noackmode) {
				/* As per GDB spec, checksums can be ignored in NoAckMode */
				rx_checksum |= unhex_digit(rx_char); /* BITWISE OR lower nibble with upper nibble */

				/* (N)Acknowledge packet */
				const bool checksum_ok = gdb_packet_checksum(packet) == rx_checksum;
				gdb_if_putchar(checksum_ok ? GDB_PACKET_ACK : GDB_PACKET_NACK, true);
				if (!checksum_ok) {
					/* Checksum error, restart packet capture */
					state = PACKET_IDLE;
					break;
				}
			}

			/* Null terminate packet */
			packet->data[packet->size] = '\0';

#if ENABLE_DEBUG == 1
			/* Log packet for debugging */
			gdb_packet_debug(__func__, packet);
#endif

			/* Return captured packet */
			return packet;

#if CONFIG_BMDA == 0
		case PACKET_BMD_REMOTE_CAPTURE:
			if (rx_char == REMOTE_SOM) {
				/* Oh dear, restart remote packet capture */
				packet->size = 0;
			} else if (rx_char == REMOTE_EOM) {
				/* Complete packet for processing */

				/* Null terminate packet */
				packet->data[packet->size] = '\0';

				/* Return captured packet */
				return packet;
			} else if (rx_char == GDB_PACKET_START) {
				/* A 'real' gdb packet, best stop squatting now */
				state = PACKET_GDB_CAPTURE;
				packet->size = 0;
				packet->type = GDB_PACKET_STANDARD;
			} else {
				/* Add character to packet */
				packet->data[packet->size++] = rx_char;
			}
			break;
#endif

		default:
			/* Something is not right, restart packet capture */
			state = PACKET_IDLE;
			break;
		}

		if (packet->size >= GDB_PACKET_BUFFER_SIZE)
			/* Buffer overflow, restart packet capture */
			state = PACKET_IDLE;
	}
}

static inline bool gdb_get_ack(const uint32_t timeout)
{
	/* Return true early if NoAckMode is enabled */
	if (noackmode)
		return true;

	/* Wait for ACK/NACK */
	return gdb_if_getchar_to(timeout) == GDB_PACKET_ACK;
}

static inline void gdb_if_putchar_escaped(const char value, const gdb_packet_type_e type)
{
	/* Escape reserved characters */
	if (gdb_packet_is_reserved(value, type)) {
		gdb_if_putchar(GDB_PACKET_ESCAPE, false);
		gdb_if_putchar((char)((uint8_t)value ^ GDB_PACKET_ESCAPE_XOR), false);
	} else {
		gdb_if_putchar(value, false);
	}
}

static inline char gdb_packet_start(const gdb_packet_type_e type)
{
	/* Return the start character of the packet */
	switch (type) {
	case GDB_PACKET_STANDARD:
		return GDB_PACKET_START;
	case GDB_PACKET_NOTIFICATION:
		return GDB_PACKET_NOTIFICATION_START;
	case GDB_PACKET_BMD_REMOTE:
		return REMOTE_RESP; /* RESP not SOM as we only send responses */
	default:
		return '\0';
	}
}

static inline char gdb_packet_end(const gdb_packet_type_e type)
{
	/* Return the start character of the packet */
	switch (type) {
	case GDB_PACKET_STANDARD:
	case GDB_PACKET_NOTIFICATION:
		return GDB_PACKET_END;
	case GDB_PACKET_BMD_REMOTE:
		return REMOTE_EOM;
	default:
		return '\0';
	}
}

void gdb_packet_send(const gdb_packet_s *const packet)
{
	/* Silently ignore invalid packets */
	if (packet->type == GDB_PACKET_UNDEFINED)
		return;

	/* Attempt packet transmission up to retries */
	const size_t retries = packet->type == GDB_PACKET_STANDARD ? GDB_PACKET_RETRIES : 1U;
	for (size_t attempt = 0U; attempt < retries; attempt++) {
		/* Write start of packet */
		gdb_if_putchar(gdb_packet_start(packet->type), false);

		/* Write packet data */
		for (size_t i = 0; i < packet->size; ++i)
			gdb_if_putchar_escaped(packet->data[i], packet->type);

		/* Write end of packet, flush if remote packet */
		gdb_if_putchar(gdb_packet_end(packet->type), packet->type == GDB_PACKET_BMD_REMOTE);

		/* Remote packets don't use checksum */
		if (packet->type != GDB_PACKET_BMD_REMOTE) {
			/* Write checksum and flush the buffer */
			const uint8_t checksum = gdb_packet_checksum(packet);
			gdb_if_putchar(hex_digit(checksum >> 4U), false);
			gdb_if_putchar(hex_digit(checksum & 0xffU), true);
		}

#if ENABLE_DEBUG == 1
		/* Log packet for debugging */
		gdb_packet_debug(__func__, packet);
#endif

		/* Wait for ACK/NACK on standard packets */
		if (packet->type != GDB_PACKET_STANDARD || gdb_get_ack(2000U))
			break;
	}
}

void gdb_putpacket_generic(const gdb_packet_type_e type, const char *const preamble, size_t preamble_size,
	const char *const data, size_t data_size, const bool hex_data)
{
	/*
	 * Create a packet using the internal packet buffer
	 * This destroys the previous packet in the buffer
	 * any packets obtained from gdb_packet_receive() will be invalidated
	 */
	gdb_packet_s *const packet = gdb_packet_buffer();
	packet->type = type;
	packet->size = 0;

	/*
	 * Copy the preamble and data into the packet buffer, limited by the buffer size
	 *
	 * This considers GDB_PACKET_BUFFER_SIZE to be the maximum size of the packet
	 * But it does not take into consideration the extra space needed for escaping
	 * This is safe because the escaping is done during the actual packet transmission
	 * but it will result in a packet larger than what we told GDB we could handle
	 */
	if (preamble != NULL && preamble_size > 0) {
		preamble_size = MIN(preamble_size, GDB_PACKET_BUFFER_SIZE);
		memcpy(packet->data, preamble, preamble_size);
		packet->size = preamble_size;
	}

	/* Add the data to the packet buffer and tranform it if needed */
	if (data != NULL && data_size > 0) {
		if (hex_data) {
			/* Hexify the data into the packet buffer, limited by the buffer size, hex data doubles in size */
			data_size = MIN(data_size, (GDB_PACKET_BUFFER_SIZE - preamble_size) / 2U);
			hexify(packet->data + packet->size, data, data_size);
			packet->size += data_size * 2U;
		} else {
			data_size = MIN(data_size, GDB_PACKET_BUFFER_SIZE - preamble_size);
			memcpy(packet->data + preamble_size, data, data_size);
			packet->size += data_size;
		}
	}

	/* Transmit the packet */
	gdb_packet_send(packet);
}

void gdb_putpacket_f(const char *const fmt, ...)
{
	/*
	 * Create a packet using the internal packet buffer
	 * This destroys the previous packet in the buffer
	 * any packets obtained from gdb_packet_receive() will be invalidated
	 */
	gdb_packet_s *const packet = gdb_packet_buffer();
	packet->type = GDB_PACKET_STANDARD;

	/*
	 * Format the string directly into the packet buffer
	 * This considers GDB_PACKET_BUFFER_SIZE to be the maximum size of the string
	 * But it does not take into consideration the extra space needed for escaping
	 * This is safe because the escaping is done during the actual packet transmission
	 * but it will result in a packet larger than what we told GDB we could handle
	 */
	va_list ap;
	packet->size = vsnprintf(packet->data, GDB_PACKET_BUFFER_SIZE + 1U, fmt, ap);
	va_end(ap);

	/* Transmit the packet */
	gdb_packet_send(packet);
}

void gdb_out(const char *const str)
{
	/**
     * Program console output packet
     * See https://sourceware.org/gdb/current/onlinedocs/gdb.html/Stop-Reply-Packets.html#Stop-Reply-Packets
     * 
     * Format; ‘O XX…’
     * ‘XX…’ is hex encoding of ASCII data, to be written as the program’s console output.
     * 
     * Can happen at any time while the program is running and the debugger should continue to wait for ‘W’, ‘T’, etc.
     * This reply is not permitted in non-stop mode.
     */
	gdb_putpacket("O", 1U, str, strnlen(str, GDB_OUT_PACKET_MAX_SIZE), true);
}

void gdb_voutf(const char *const fmt, va_list ap)
{
	/*
	 * We could technically do the formatting and transformation in a single buffer reducing stack usage
	 * But it is a bit more complex and likely slower, we would need to spread the characters out such
	 * that each occupies two bytes, and then we could hex them in place
	 * 
	 * If this stack usage proves to be a problem, we can revisit this
	 */
	char str_scratch[GDB_OUT_PACKET_MAX_SIZE + 1U];

	/* Format the string into the scratch buffer */
	vsnprintf(str_scratch, sizeof(str_scratch), fmt, ap);

	/* Delegate the rest of the work to gdb_out */
	gdb_out(str_scratch);
}

void gdb_outf(const char *const fmt, ...)
{
	/* Wrap the va_list version of gdb_voutf */
	va_list ap;
	va_start(ap, fmt);
	gdb_voutf(fmt, ap);
	va_end(ap);
}
