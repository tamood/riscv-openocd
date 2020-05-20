/**************************************************************************
*   Copyright (C) 2020 by Tayyeb Mahmood                                  *
*   tayyeb.mahmood@gmail.com                                              *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
***************************************************************************/

/**
 * @file
 * Description: Xilinx Virtual Cable (XVC) is a TCP/IP-based protocol that 
 * acts like a JTAG cable and provides a means to access and debug your FPGA 
 * or SoC design without using a physical cable.
 * 
 * This capability helps facilitate hardware debug for designs that:
 * 
 *  - Have the FPGA in a hard-to-access location, where a "lab-PC" is not 
 *    close by 
 *  - Do not have direct access to the FPGA pins – e.g. the JTAG pins are 
 *    only accessible via a local processor interface
 *  - Need to efficiently debug Xilinx FGPA or SoC systems deployed in the 
 *    field to save on costly or impractical travel and reduce the time it 
 *    takes to debug a remotely located system.
 *
 * This code implements XVC 1.0 Protocol which was found here:
 * https://github.com/Xilinx/XilinxVirtualCable
 *
 * Many parts of the code are based on following works
 *  - xlnx-pcie-xvc.c   Author: Moritz Fischer  <moritzf@google.com>
 *    under GPL-2.0
 *  - bitbang.c         Author: Dominic Rath    <Dominic.Rath@gmx.de> 
 *    under GPL-2.0
 *  - remote-bitbang.c  Author: Richard Uhler   <ruhler@mit.edu> 
 *    under GPL-2.0 * 
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "bitbang.h"
#include <jtag/interface.h>
#include <jtag/commands.h>

#ifndef _WIN32
	#include <sys/un.h>
	#include <netdb.h>
#endif

#ifndef MIN
	#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif
#define XLNX_XVC_MAX_BITS	(248*8)

static char *xvc_host;

static FILE *xvc_file;
static int xvc_fd;

uint8_t v_ones[XLNX_XVC_MAX_BITS >> 3];
uint8_t v_zeros[XLNX_XVC_MAX_BITS >> 3];
uint8_t dummy[XLNX_XVC_MAX_BITS >> 3];

static int xvc_transact(size_t num_bits, uint8_t * tms, uint8_t * tdi,
				   uint8_t * tdo)
{
	int err = ERROR_OK;
	if(num_bits == 0)
	    return err;
	    
    size_t num_bytes = (num_bits + 7) >> 3;
        
    // Send command header
    static const char cmd_name[] = "shift:";
    if (fwrite(cmd_name, 1, 6, xvc_file) < 6) {
		LOG_ERROR("XVC cmd error: %s", strerror(errno));
		return ERROR_FAIL;
	}
	
    // Send command length
    if (fwrite(&num_bits, 1, 4, xvc_file) < 4) {
		LOG_ERROR("XVC len error: %s", strerror(errno));
		return ERROR_FAIL;
	}
        
    // Send tms
    if (fwrite(tms, 1, num_bytes, xvc_file) < num_bytes) {
		LOG_ERROR("XVC tms error: %s", strerror(errno));
		return ERROR_FAIL;
	}
        
    // Send tdi
    if (fwrite(tdi, 1, num_bytes, xvc_file) < num_bytes) {
		LOG_ERROR("XVC tdi error: %s", strerror(errno));
		return ERROR_FAIL;
	}
        
    // Receive tdo
    if (fread(tdo, 1, num_bytes, xvc_file) < num_bytes) {
		LOG_ERROR("XVC tdo error: %s", strerror(errno));
		return ERROR_FAIL;
	}
        
	return ERROR_OK;
}

static int xvc_execute_stableclocks(int num_cycles)
{
	uint8_t * tms = tap_get_state() == TAP_RESET ? v_ones : v_zeros;
	size_t left = num_cycles;
	size_t write;
	int err;
	
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		err = xvc_transact(write, tms, v_zeros, dummy);
		if (err != ERROR_OK)
			return err;
		left -= write;
	}

	return ERROR_OK;
}

static int xvc_state_move(size_t skip)
{
	int tms_scan = tap_get_tms_path(tap_get_state(),
					    tap_get_end_state()) >> skip;
	int tms_count = tap_get_tms_path_len(tap_get_state(),
					     tap_get_end_state()) - skip;
	int err;

	err = xvc_transact(tms_count, (uint8_t *)&tms_scan, v_zeros, dummy);
	if (err != ERROR_OK)
		return err;

	tap_set_state(tap_get_end_state());

	return ERROR_OK;
}



static int xvc_execute_runtest(int num_cycles)
{
	int err = ERROR_OK;

	tap_state_t tmp_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		err = xvc_state_move(0);
		if (err != ERROR_OK)
			return err;
	}

	size_t left = num_cycles;
	size_t write;

	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		err = xvc_transact(write, v_zeros, v_zeros, dummy);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	tap_set_end_state(tmp_state);
	if (tap_get_state() != tap_get_end_state())
		err = xvc_state_move(0);

	return err;
}

static int xvc_execute_pathmove(struct pathmove_command *cmd)
{
	size_t num_states = cmd->num_states;
	tap_state_t *path = cmd->path;
	uint32_t i;
    int tms = 0;

	for (i = 0; i < num_states; i++) {
			if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
				tms = tms;
			else if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
				tms |= (1 << i);
			else {
				LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					tap_state_name(tap_get_state()),
					tap_state_name(cmd->path[i]));
				break;
			}

			tap_set_state(path[i]);
	}
         
    if((i > 0) && (xvc_transact(i, (uint8_t *) &tms, v_zeros, dummy) != ERROR_OK))
		return ERROR_FAIL;
        
    tap_set_end_state(tap_get_state());
        
    if(i < num_states)
        return ERROR_JTAG_QUEUE_FAILED;
        
	return ERROR_OK;
}

static int xvc_execute_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		unsigned scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	
	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			tap_set_end_state(TAP_IRSHIFT);
		else
			tap_set_end_state(TAP_DRSHIFT);

		if (xvc_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
		tap_set_end_state(saved_end_state);
	}

	uint8_t * tdi;
	int err;
	size_t base=0;

	int left = scan_size;
	while (left) {
		int write = MIN(XLNX_XVC_MAX_BITS, left);
        left -= write;
        
		/* the last TMS should be a 1, to leave the state */
		int last_bit = write - 1;
        int last_tms_bits = 1 << (last_bit & 7);
        int last_tms_offs = last_bit >> 3;
                
		v_zeros[last_tms_offs] = left > 0 ? 0: last_tms_bits;

		tdi = (type != SCAN_IN) ? &buffer[base] : v_zeros;
		
		err = xvc_transact(write, v_zeros, tdi, tdi);
        v_zeros[last_tms_offs] = 0;
        base += (XLNX_XVC_MAX_BITS >> 3);
                
		if (err != ERROR_OK)
			return err;
	};

	if (tap_get_state() != tap_get_end_state())
		err = xvc_state_move(1);

	return err;
}

static int xvc_execute_reset(int trst, int srst)
{
	uint8_t c = 0x1F;
	return xvc_transact(5, &c, v_zeros, dummy);
}

static int xvc_execute_tms(struct jtag_command *cmd)
{
	int num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	
	size_t left, write;
	int err;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		err = xvc_transact(write, (uint8_t *) bits, v_zeros, dummy);
		if (err != ERROR_OK)
			return err;
		left -= write;
	};

	return ERROR_OK;
}

int xvc_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RESET:
				LOG_DEBUG_IO("reset trst: %i srst %i",
						cmd->cmd.reset->trst,
						cmd->cmd.reset->srst);
				if ((cmd->cmd.reset->trst == 1) ||
						(cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);
				if (xvc_execute_reset(cmd->cmd.reset->trst,
							cmd->cmd.reset->srst) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_RUNTEST:
				LOG_DEBUG_IO("runtest %i cycles, end in %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));
				assert(tap_is_state_stable(cmd->cmd.runtest->end_state));
				tap_set_end_state(cmd->cmd.runtest->end_state);
				if (xvc_execute_runtest(cmd->cmd.runtest->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				if (xvc_execute_stableclocks(cmd->cmd.stableclocks->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_TLR_RESET:
				LOG_DEBUG_IO("statemove end in %s",
						tap_state_name(cmd->cmd.statemove->end_state));
				assert(tap_is_state_stable(cmd->cmd.statemove->end_state));
				tap_set_end_state(cmd->cmd.statemove->end_state);
				if (xvc_state_move(0) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("pathmove: %i states, end in %s",
						cmd->cmd.pathmove->num_states,
						tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
				if (xvc_execute_pathmove(cmd->cmd.pathmove) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_SCAN:
				assert(tap_is_state_stable(cmd->cmd.scan->end_state));
				tap_set_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				LOG_DEBUG_IO("%s scan %d bits; end in %s",
						(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
						scan_size,
					tap_state_name(cmd->cmd.scan->end_state));
				type = jtag_scan_type(cmd->cmd.scan);
				if (xvc_execute_scan(cmd->cmd.scan->ir_scan, type, buffer,
							scan_size) != ERROR_OK)
					return ERROR_FAIL;
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
				LOG_DEBUG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_TMS:
				retval = xvc_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	return retval;
}

static int init_tcp(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;
	int fd = 0;

	LOG_INFO("Connecting to %s:%s",
			xvc_host ? xvc_host : "localhost",
			"2542");

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(xvc_host, "2542", &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp != NULL ; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd == -1)
			continue;

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(fd);
	}

	freeaddrinfo(result); /* No longer needed */

	if (rp == NULL) { /* No address succeeded */
		LOG_ERROR("Failed to connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return fd;
}

static int xvc_init(void)
{
	for(int i=0; i < (XLNX_XVC_MAX_BITS >> 3); i++)
	{
		v_ones[i]=0xFF;
		v_zeros[i]=0;
	}

	LOG_INFO("Initializing xvc driver");

	xvc_fd = init_tcp();

	if (xvc_fd < 0)
		return xvc_fd;

	xvc_file = fdopen(xvc_fd, "w+");
	if (xvc_file == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		close(xvc_fd);
		return ERROR_FAIL;
	}

	LOG_INFO("xvc driver initialized");
	return ERROR_OK;
}

static int xvc_quit(void)
{
	// Send command header
    static const char cmd_quit[] = "xvc connection close";
    if (fwrite(cmd_quit, 1, 20, xvc_file) < 20) {
		LOG_ERROR("XVC cmd error: %s", strerror(errno));
		return ERROR_FAIL;
	}
	if (EOF == fflush(xvc_file)) {
		LOG_ERROR("fflush: %s", strerror(errno));
		return ERROR_FAIL;
	}

	/* We only need to close one of the FILE*s, because they both use the same */
	/* underlying file descriptor. */
	if (EOF == fclose(xvc_file)) {
		LOG_ERROR("fclose: %s", strerror(errno));
		return ERROR_FAIL;
	}

	free(xvc_host);

	LOG_INFO("xvc interface quit");
	return ERROR_OK;
}


COMMAND_HANDLER(xvc_handle_xvc_host_command)
{
	if (CMD_ARGC == 1) {
		free(xvc_host);
		xvc_host = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration xvc_command_handlers[] = {
	{
		.name = "xvc_host",
		.handler = xvc_handle_xvc_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote XVC server.\n"
			"  if port is 0 or unset, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	COMMAND_REGISTRATION_DONE,
};

struct jtag_interface xvc_interface = {
	.name = "xilinx_virtual_cable",
	.execute_queue = &xvc_execute_queue,
	.transports = jtag_only,
	.commands = xvc_command_handlers,
	.init = &xvc_init,
	.quit = &xvc_quit,
};
