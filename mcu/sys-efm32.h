extern const uint8_t sys_version[8];
#if defined(USE_SYS3) || defined(USE_SYS_BOARD_ID)
extern const uint32_t sys_board_id;
extern const char *const sys_board_name;
#endif

typedef void (*handler)(void);
typedef void (*nonreturn_handler1)(void *)__attribute__((noreturn));
extern handler sys_vector[16];

static inline void
set_led (int on)
{
  void (*func) (int) = (void (*)(int))sys_vector[2];

  return (*func) (on);
}

#ifdef REQUIRE_CLOCK_GPIO_SETTING_IN_SYS
/* Provide the function entries.  */

static void __attribute__ ((used))
clock_init (void)
{
  (*sys_vector[0]) ();
}

static void __attribute__ ((used))
gpio_init (void)
{
  (*sys_vector[1]) ();
}
#endif

static inline void
flash_unlock (void)
{
  (*sys_vector[3]) ();
}

static inline int
flash_erase_page (uintptr_t addr)
{
  int (*func) (uintptr_t) = (int (*)(uintptr_t))sys_vector[4];

  return (*func) (addr);
}

static inline int
flash_program_halfword (uintptr_t addr, uint16_t data)
{
  int (*func) (uintptr_t, uint16_t) = (int (*)(uintptr_t, uint16_t))sys_vector[6];

  return (*func) (addr, data);

}

static inline int
flash_check_blank (const uint8_t *p_start, size_t size)
{
  int (*func) (const uint8_t *, int) = (int (*)(const uint8_t *, int))sys_vector[7];

  return (*func) (p_start, size);
}

static inline void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void))
{
  nonreturn_handler1 func = (nonreturn_handler1) sys_vector[8] ;

  (*func) (entry);
}




static inline int
flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len)
{
  int (*func) (uintptr_t, const uint8_t *, size_t)
    = (int (*)(uintptr_t, const uint8_t *, size_t))sys_vector[5];

  return (*func) (dst_addr, src, len);
}

static inline const uint8_t *
unique_device_id (void)
{
  const uint8_t *addr = (const uint8_t *)0xfe081f0;

  return addr;
}
