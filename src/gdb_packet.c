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
} packet_state_e;

static bool noackmode = false;

/* This has to be aligned so the remote protocol can re-use it without causing Problems */
static gdb_packet_s BMD_ALIGN_DEF(8) packet_buffer;

char *gdb_packet_buffer(void)
{
	/* Return the static packet data buffer */
	return packet_buffer.data;
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

packet_state_e consume_remote_packet(char *const packet, const size_t size)
{
#if CONFIG_BMDA == 0
	/* We got what looks like probably a remote control packet */
	size_t offset = 0;
	while (true) {
		/* Consume bytes until we either have a complete remote control packet or have to leave this mode */
		const char rx_char = gdb_if_getchar();

		switch (rx_char) {
		case '\x04':
			packet[0] = rx_char;
			/* EOT (end of transmission) - connection was closed */
			return PACKET_IDLE;

		case REMOTE_SOM:
			/* Oh dear, restart remote packet capture */
			offset = 0;
			break;

		case REMOTE_EOM:
			/* Complete packet for processing */

			/* Null terminate packet */
			packet[offset] = '\0';
			/* Handle packet */
			remote_packet_process(packet, offset);

			/* Restart packet capture */
			packet[0] = '\0';
			return PACKET_IDLE;

		case GDB_PACKET_START:
			/* A 'real' gdb packet, best stop squatting now */
			return PACKET_GDB_CAPTURE;

		default:
			if (offset < size)
				packet[offset++] = rx_char;
			else {
				packet[0] = '\0';
				/* Buffer overflow, restart packet capture */
				return PACKET_IDLE;
			}
		}
	}
#else
	(void)packet;
	(void)size;

	/* Hosted builds ignore remote control packets */
	return PACKET_IDLE;
#endif
}

gdb_packet_s *gdb_packet_receive(void)
{
	packet_state_e state = PACKET_IDLE; /* State of the packet capture */

	uint8_t checksum = 0;
	uint8_t rx_checksum = 0;

	while (true) {
		const char rx_char = gdb_if_getchar();

		switch (state) {
		case PACKET_IDLE:
			packet_buffer.data[0U] = rx_char;
			if (rx_char == GDB_PACKET_START) {
				/* Start of GDB packet */
				state = PACKET_GDB_CAPTURE;
				checksum = 0;
				packet_buffer.size = 0;
				packet_buffer.notification = false;
			}
#if CONFIG_BMDA == 0
			else if (rx_char == REMOTE_SOM) {
				/* Start of BMP remote packet */
				/*
				 * Let consume_remote_packet handle this
				 * returns PACKET_IDLE or PACKET_GDB_CAPTURE if it detects the start of a GDB packet
				 */
				state = consume_remote_packet(packet_buffer.data, GDB_PACKET_BUFFER_SIZE);
				checksum = 0;
				packet_buffer.size = 0;
			}
#endif
			/* EOT (end of transmission) - connection was closed */
			else if (rx_char == '\x04') {
				packet_buffer.data[1U] = '\0'; /* Null terminate */
				packet_buffer.size = 1U;
				return &packet_buffer;
			}
			break;

		case PACKET_GDB_CAPTURE:
			if (rx_char == GDB_PACKET_START) {
				/* Restart GDB packet capture */
				packet_buffer.size = 0;
				checksum = 0;
				break;
			}
			if (rx_char == GDB_PACKET_END) {
				/* End of GDB packet */

				/* Move to checksum capture */
				state = PACKET_GDB_CHECKSUM_UPPER;
				break;
			}

			/* Not start or end of packet, add to checksum */
			checksum += rx_char;

			/* Add to packet buffer, unless it is an escape char */
			if (rx_char == GDB_PACKET_ESCAPE)
				/* GDB Escaped char */
				state = PACKET_GDB_ESCAPE;
			else
				/* Add to packet buffer */
				packet_buffer.data[packet_buffer.size++] = rx_char;
			break;

		case PACKET_GDB_ESCAPE:
			/* Add to checksum */
			checksum += rx_char;

			/* Resolve escaped char */
			packet_buffer.data[packet_buffer.size++] = rx_char ^ GDB_PACKET_ESCAPE_XOR;

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
				gdb_if_putchar(rx_checksum == checksum ? GDB_PACKET_ACK : GDB_PACKET_NACK, true);
			}

			if (noackmode || rx_checksum == checksum) {
				/* Null terminate packet */
				packet_buffer.data[packet_buffer.size] = '\0';

#if ENABLE_DEBUG == 1
				/* Log packet for debugging */
				gdb_packet_debug(__func__, &packet_buffer);
#endif

				/* Return packet captured size */
				return &packet_buffer;
			}

			/* Restart packet capture */
			state = PACKET_IDLE;
			break;

		default:
			/* Something is not right, restart packet capture */
			state = PACKET_IDLE;
			break;
		}

		if (packet_buffer.size >= GDB_PACKET_BUFFER_SIZE)
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

static inline void gdb_putchar(const char value, uint8_t *const csum)
{
	/* Send the character to the GDB interface */
	gdb_if_putchar(value, false);

	/* Add to checksum */
	if (csum != NULL)
		*csum += value;
}

static inline void gdb_putchar_escaped(const char value, uint8_t *const csum)
{
	/* Escape reserved characters */
	if (value == GDB_PACKET_START || value == GDB_PACKET_END || value == GDB_PACKET_ESCAPE ||
		value == GDB_PACKET_RUNLENGTH_START) {
		gdb_putchar(GDB_PACKET_ESCAPE, csum);
		gdb_putchar((char)((uint8_t)value ^ GDB_PACKET_ESCAPE_XOR), csum);
	} else {
		gdb_putchar(value, csum);
	}
}

void gdb_packet_send(const gdb_packet_s *const packet)
{
	/* Attempt packet transmission up to retries */
	for (size_t attempt = 0U; attempt < GDB_PACKET_RETRIES; attempt++) {
		uint8_t csum = 0; /* Checksum of packet data */

		/* Write start of packet */
		gdb_putchar(packet->notification ? GDB_PACKET_NOTIFICATION_START : GDB_PACKET_START, NULL);

		/* Write packet data */
		for (size_t i = 0; i < packet->size; ++i)
			gdb_putchar_escaped(packet->data[i], &csum);

		/* Write end of packet */
		gdb_if_putchar(GDB_PACKET_END, false);

		/* Write checksum and flush the buffer */
		gdb_putchar(hex_digit(csum >> 4U), NULL);
		gdb_putchar(hex_digit(csum & 0xffU), NULL);
		gdb_if_flush();

#if ENABLE_DEBUG == 1
		/* Log packet for debugging */
		gdb_packet_debug(__func__, packet);
#endif

		/* Wait for ACK/NACK on standard packets */
		if (packet->notification || gdb_get_ack(2000U))
			break;
	}
}

void gdb_putpacket(const char *preamble, size_t preamble_size, const char *data, size_t data_size, bool hex_data)
{
	/*
	 * Create a packet using the internal packet buffer
	 * This destroys the previous packet in the buffer
	 * any packets obtained from gdb_packet_receive() will be invalidated
	 */
	packet_buffer.notification = false;
	packet_buffer.size = 0;

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
		memcpy(packet_buffer.data, preamble, preamble_size);
		packet_buffer.size = preamble_size;
	}

	/* Add the data to the packet buffer and tranform it if needed */
	if (data != NULL && data_size > 0) {
		if (hex_data) {
			/* Hexify the data into the packet buffer, limited by the buffer size, hex data doubles in size */
			data_size = MIN(data_size, (GDB_PACKET_BUFFER_SIZE - preamble_size) / 2U);
			hexify(packet_buffer.data + packet_buffer.size, data, data_size);
			packet_buffer.size += data_size * 2U;
		} else {
			data_size = MIN(data_size, GDB_PACKET_BUFFER_SIZE - preamble_size);
			memcpy(packet_buffer.data + preamble_size, data, data_size);
			packet_buffer.size += data_size;
		}
	}

	/* Transmit the packet */
	gdb_packet_send(&packet_buffer);
}

void gdb_putpacket_f(const char *const fmt, ...)
{
	/*
	 * Create a packet using the internal packet buffer
	 * This destroys the previous packet in the buffer
	 * any packets obtained from gdb_packet_receive() will be invalidated
	 */
	packet_buffer.notification = false;

	/*
	 * Format the string directly into the packet buffer
	 * This considers GDB_PACKET_BUFFER_SIZE to be the maximum size of the string
	 * But it does not take into consideration the extra space needed for escaping
	 * This is safe because the escaping is done during the actual packet transmission
	 * but it will result in a packet larger than what we told GDB we could handle
	 */
	va_list ap;
	packet_buffer.size = vsnprintf(packet_buffer.data, GDB_PACKET_BUFFER_SIZE + 1U, fmt, ap);
	va_end(ap);

	/* Transmit the packet */
	gdb_packet_send(&packet_buffer);
}

void gdb_put_notificationz(const char *const str)
{
	/*
	 * Create a packet using the internal packet buffer
	 * This destroys the previous packet in the buffer
	 * any packets obtained from gdb_packet_receive() will be invalidated
	 */
	packet_buffer.notification = true;

	packet_buffer.size = strnlen(str, GDB_PACKET_BUFFER_SIZE);
	memcpy(packet_buffer.data, str, packet_buffer.size);

	/* Transmit the packet */
	gdb_packet_send(&packet_buffer);
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
