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

/* Provides main entry point.  Initialise subsystems and enter GDB
 * protocol loop.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "gdb_if.h"
#include "gdb_main.h"
#include "jtagtap.h"
#include "jtag_scan.h"

#include "target.h"

// main stack memory
uint8_t _main_stack[MAIN_STACK_SIZE] __attribute__ ((section(".stack.main")));

extern uint32_t __heap_end;
extern uint32_t __heap_start;
static char* heap_curr = 0; 

caddr_t _sbrk(int incr) 
{
    char *prev_heap_end;
    
    if (heap_curr == 0) 
    {
        heap_curr = (char*)(&__heap_start);
    }
    prev_heap_end = heap_curr;
    
    if (heap_curr + incr > (char*)&__heap_end) 
    {
        return (caddr_t)0;
    }
    
    heap_curr += incr;
    
    return (caddr_t) prev_heap_end;
}

int
main(int argc, char **argv)
{
#if defined(LIBFTDI)
	platform_init(argc, argv);
#else
	(void) argc;
	(void) argv;
    platform_init();
#endif
	PLATFORM_SET_FATAL_ERROR_RECOVERY();

	gdb_main();

	/* Should never get here */
	return 0;
}

