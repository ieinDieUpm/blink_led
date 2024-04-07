/**
 * @file port_system.c
 * @author Josué Pagán Ortiz (j.pagan@upm.es)
 * @brief Port layer for the system functions in the native platform.
 * @date 01-01-2024
 */
#include "port_system.h"
#include <sys/time.h>
#include <stdio.h>

uint32_t port_system_init()
{
  return 0;
}

uint32_t port_system_get_millis()
{
  struct timeval te;
  gettimeofday(&te, NULL);                                         // get current time
  long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // calculate milliseconds
  return milliseconds;
}

void port_system_delay_ms(uint32_t ms)
{
  uint32_t tickstart = port_system_get_millis();

  while ((port_system_get_millis() - tickstart) < ms)
  {
  }
}

void port_system_delay_until_ms(uint32_t *t, uint32_t ms)
{
  uint32_t until = *t + ms;
  uint32_t now = port_system_get_millis();
  if (until > now)
  {
    port_system_delay_ms(until - now);
  }
  *t = port_system_get_millis();
}