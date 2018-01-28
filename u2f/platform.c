/*
 * platform.c - platform specific hacks
 *
 * Copyright (C) 2018 Sean Cross,
 *                    Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of U2F firmware
 * Bootloader-spcific parts were copied ftom toboot.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "sys.h"

#include <mcu/efm32.h>

#define SYSRESETREQ 0x04

/* Perform reset (common for all Cortex-M* processors?) */
static void
nvic_system_reset (void)
{
  SCB->AIRCR = (0x05FA0000 | (SCB->AIRCR & 0x70) | SYSRESETREQ);
  asm volatile ("dsb");
  for (;;);
}

#define LOCKBITS_BASE   (0x0FE04000UL) /* Lock-bits page base address */
#define DEBUG_LOCK_WORD (LOCKBITS_BASE + (127 * 4))

const volatile uint32_t *dlw = (volatile uint32_t *) DEBUG_LOCK_WORD;

#if defined(ENFORCE_DEBUG_LOCK)
/* Debug lock EFM32HG device by clearing DEBUG LOCK WORD */
static void
debug_lock_maybe (void)
{
  uint8_t zero[] = { 0x0, 0x0, 0x0, 0x0 };
  if (*dlw != 0)
    flash_write ((uintptr_t) dlw, zero, sizeof (zero));
}
#endif

static void
busy_wait (void)
{
  int i;
  for (i = 0; i < 20000; i++)
    asm ("nop");
}

extern uint32_t _device_key_base;
extern uint32_t _auth_ctr_base;

uint32_t *boot_vectors = 0x0;

/* Erase app secrets and enter bootloader if outer PADs (PC1 and PE12)
are shorted. */
static void
erase_sercets_maybe (void)
{
  int in[4];
  int i;

  /* Setup PC1 as push-pull output */
  GPIO->P[2].MODEL &= ~0xF0UL;
  GPIO->P[2].MODEL |= 0x00000004UL << 4;

  /* Setup PE12 as input with pull-down */
  GPIO->P[4].MODEH &= ~0xF0000UL;
  GPIO->P[4].MODEH |= 0x00000002UL << 16;

  busy_wait ();

  /* toggle PC1 output couple of times to see if PE12 input changes
  accordingly */
  GPIO->P[2].DOUTSET = 2;

  for (i = 0; i < 4; i++)
    {
      busy_wait ();
      in[i] = GPIO->P[4].DIN & (1 << 12);
      GPIO->P[2].DOUTTGL = 2;
    }

  GPIO->P[2].DOUTTGL = 2;

  if (in[0] && !in[1] && in[2] && !in[3])
    {
      /* erase device key and auth counter */
      flash_erase_page ((uintptr_t) &_device_key_base);
      flash_erase_page ((uintptr_t) &_auth_ctr_base);

      /* erase first app page to make bootloader enter DFU mode */
      flash_erase_page ((uintptr_t) 0x4000);

      /* reset and boot bootloader */
      nvic_system_reset ();
    }

  GPIO->P[2].MODEL = 0;
  GPIO->P[2].MODEH = 0;
  GPIO->P[4].MODEL = 0;
  GPIO->P[4].MODEH = 0;
}

/* Preform platform-specific actions */
void
platform_init (void)
{

#if defined(ENFORCE_DEBUG_LOCK)
  debug_lock_maybe ();
#endif

  erase_sercets_maybe ();
}
