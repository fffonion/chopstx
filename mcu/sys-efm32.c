/*
 * sys-efm32.c - system vectors for EFM32HG
 *
 * Copyright (C) 2017-2018 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * This file is a part of Chpostx port to EFM32HG
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
#include "board.h"

extern uint8_t __ram_end__;
void reset (void);

void exit(int __status)
{}

static uint32_t
stack_entry[] __attribute__ ((section(".first_page.first_words"),used)) = {
  (uint32_t)&__ram_end__, (uint32_t)reset,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0x0070b0 /* don't allow to enter toboot by shorting outer pads */,
  0x106fb0 /* let toboot know that starting offset is 0x4000 */
};

const uint8_t __attribute__((section(".sys.board_name")))
sys_board_name[] = BOARD_NAME;

const uint8_t sys_version[8] __attribute__((section(".sys.version"))) = {
  3*2+2,	     /* bLength */
  0x03,		     /* bDescriptorType = USB_STRING_DESCRIPTOR_TYPE */
  /* sys version: "3.0" */
  '3', 0, '.', 0, '0', 0,
};


typedef void (*handler)(void);
extern handler vector_table[];

#define ADDR_VECTORS ((uint32_t)(&vector_table))
#define ADDR_SCR_VTOR 0xe000ed08

#define intr_disable()  asm volatile ("cpsid   i" : : : "memory")
#define intr_enable()  asm volatile ("cpsie   i" : : : "memory")

void
reset (void)
{
  uint32_t r3 = ADDR_SCR_VTOR;

  asm volatile ("str  %2, [%0]\n\t"   /* Set SCR->VTOR     */
    "ldr  %0, [%2]\n\t"     /* Stack address     */
    "msr  MSP, %0\n\t"    /* Exception handler stack. */
    "ldr  %0, [%2, #4]\n\t" /* The entry address */
    "bx %0\n\t"           /* Jump to the entry */
    ".align 2"
    : "=r" (r3)
    : "0" (r3), "r" (ADDR_VECTORS)
    : "memory");

  /* Never reach here. */
}

uint8_t *
sram_address (uint32_t offset)
{
  return ((uint8_t *)0x20000000) + offset;
}


#include "mcu/clk_gpio_init-efm32.c"

static void
set_led (int on)
{
  /* PB7 */
  const uint8_t pin = 7;
  const uint8_t port = 1;
  if (on)
    GPIO->P[port].DOUTCLR = 1 << pin; /* Clear: Light on  */
  else
    GPIO->P[port].DOUTSET = 1 << pin; /* Set: Light off  */
}

#define FLASH_PAGE_SIZE 1024

static inline void __attribute__ ((section(".ramtext")))
flash_write_enable (void)
{
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
}

static inline void __attribute__ ((section(".ramtext")))
flash_write_disable (void)
{
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

static inline void __attribute__ ((section(".ramtext")))
flash_wait_for_completion (void)
{
  while (MSC->STATUS & MSC_STATUS_BUSY) {}
}

static inline int __attribute__ ((section(".ramtext")))
flash_load_addr (uintptr_t addr)
{
  flash_wait_for_completion ();

  MSC->ADDRB = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  uint32_t status = MSC->STATUS;

  if (status & MSC_STATUS_INVADDR || status & MSC_STATUS_LOCKED)
    return -1;

  return 0;
}

static void __attribute__ ((section(".ramtext")))
flash_unlock (void)
{
  /* Unlock the MSC */
  MSC->LOCK = MSC_UNLOCK_CODE;

  flash_write_disable ();
}

static int __attribute__ ((section(".ramtext")))
flash_erase_page (uintptr_t addr)
{
  int status;

  flash_write_enable ();

  status = flash_load_addr (addr);

  if (status == 0)
    {
      intr_disable ();

      /* Erase the page */
      MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

      flash_wait_for_completion ();

      intr_enable ();
    }

  flash_write_disable ();

  return status;
}

static int
flash_program_halfword (uintptr_t addr, uint16_t data)
{

  return 1;
}


static int __attribute__ ((section(".ramtext")))
flash_write_word (uintptr_t dst_addr, uint32_t data)
{
  int status = flash_load_addr (dst_addr);

  if (status == 0)
    {
      intr_disable ();

      MSC->WDATA = data;
      MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

      intr_enable ();
    }

  flash_wait_for_completion ();

  return status;
}

static int __attribute__ ((section(".ramtext")))
flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len)
{
  const uint32_t *wsrc = (const uint32_t *) src;
  size_t words = len / 4;

  flash_write_enable ();

  while (words > 0)
    {
      if (flash_write_word (dst_addr, *wsrc) != 0)
        return -1;
      dst_addr += 4;
      wsrc++;
      words--;
    }

  flash_write_disable ();

  return 0;
}

static int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  const uint8_t *p;

  for (p = p_start; p < p_start + size; p++)
    if (*p != 0xff)
      return 0;

  return 1;
}

static void __attribute__((naked))
flash_erase_all_and_exec (void (*entry)(void))
{
//  uintptr_t addr = FLASH_START;
//#if defined(STM32F103_OVERRIDE_FLASH_SIZE_KB)
//  uintptr_t end = FLASH_START_ADDR + STM32F103_OVERRIDE_FLASH_SIZE_KB*1024;
//#else
//  uintptr_t end = FLASH_START_ADDR + (*FLASH_SIZE_REG)*1024;
//#endif
//  uint32_t page_size = 1024;
//  int r;
//
//  if (((*CHIP_ID_REG) & 0xfff) == 0x0414)
//    page_size = 2048;
//
//  while (addr < end)
//    {
//      r = flash_erase_page (addr);
//      if (r != 0)
//	break;
//
//      addr += page_size;
//    }
//
//  if (addr >= end)
//    (*entry) ();
//
  for (;;);
}


handler sys_vector[] __attribute__ ((section(".sys.vectors"))) = {
  clock_init,
  gpio_init,
  (handler)set_led,
  flash_unlock,
  (handler)flash_erase_page,
  (handler)flash_write,
  (handler)flash_program_halfword,
  (handler)flash_check_blank,
  (handler)flash_erase_all_and_exec,
  NULL,
};
