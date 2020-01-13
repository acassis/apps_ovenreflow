/****************************************************************************
 * examples/reflowoven/reflowoven_main.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>

#include <nuttx/sensors/zerocross.h>

#define BAUDRATE B9600
#define FALSE 0
#define TRUE 1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

  volatile int STOP = FALSE;
  int wait_flag = TRUE;
  char btdev[80] = "/dev/ttyS1";
  char tempdev[80] = "/dev/temp0";
  int status;

  int newdata = 0;

#define BUFFERED_IO
#define SIZE 4

enum oven_cmds
{
  NONE              = 0,
  READ_TEMPERATURE  = 1,
  SET_DUTY_CYCLE    = 2,
  SET_SENSOR_OFFSET = 3,
  SET_LCD_BACKLIGHT = 4,
  SET_LCD_CONTRAST  = 5,
  READ_SETTINGS     = 6
};

enum acknowledge
{
  ACK   = 0,
  NACK  = 1
};

#ifndef CONFIG_SENSORS_ZEROCROSS
#  error "CONFIG_ZEROCROSS is not defined in the configuration"
#endif

#ifndef CONFIG_EXAMPLES_ZEROCROSS_DEVNAME
#  define CONFIG_EXAMPLES_ZEROCROSS_DEVNAME "/dev/zc0"
#endif

#ifndef CONFIG_EXAMPLES_ZEROCROSS_SIGNO
#  define CONFIG_EXAMPLES_ZEROCROSS_SIGNO 13
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int count = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int validcmd(uint8_t cmd)
{
  if (cmd >= NONE && cmd <= READ_SETTINGS)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct sigevent event;

  /* struct termios newtio; */

  /* struct sigaction saio; */

  int fd_bt;
  int fd_temp;
  int fd_zc;
  int fd_out;
  int i;
  int rcvd;
  int ret;
  int delay = 0;
  int new_cmd = 0;
  int duty = 0;
  uint16_t temp;
  char buf[255];
  uint8_t cmd_data[12];
  uint8_t cmd_pos = 0;

  /* open the device in non-blocking way (read will return immediately) */

  fd_bt = open(btdev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_bt < 0)
    {
      perror(btdev);
      exit(1);
    }

  /* open the temperature device */

  fd_temp = open(tempdev, O_RDONLY);
  if (fd_temp < 0)
    {
      perror(tempdev);
      exit(1);
    }

  /* Open the zerocross device */

  fd_zc = open("/dev/zc0", O_RDONLY);
  if (fd_zc < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s\n",
              CONFIG_EXAMPLES_ZEROCROSS_DEVNAME);
      return EXIT_FAILURE;
    }

  /* Open the Output device to control the Oven heater */

  fd_out = open("/dev/out", O_RDONLY);
  if (fd_out < 0)
    {
      fprintf(stderr, "ERROR: Failed to open /dev/out\n");
      return EXIT_FAILURE;
    }

  /* Register to receive a signal on every zero cross event */

  event.sigev_notify = SIGEV_SIGNAL;
  event.sigev_signo  = CONFIG_EXAMPLES_ZEROCROSS_SIGNO;

  ret = ioctl(fd_zc, ZCIOC_REGISTER, (unsigned long)((uintptr_t)&event));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: ioctl(ZCIOC_REGISTER) failed!\n");
      return EXIT_FAILURE;
    }

  while (STOP == FALSE)
    {
      struct siginfo value;
      sigset_t set;
      ssize_t nread;

      /* Wait for a signal */

      (void)sigemptyset(&set);
      (void)sigaddset(&set, CONFIG_EXAMPLES_ZEROCROSS_SIGNO);
      ret = sigwaitinfo(&set, &value);
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: sigwaitinfo() failed!\n");
          return EXIT_FAILURE;
        }

      /* Calculate new duty cycle */

      delay = 250000 - (duty * 2400);

      /* Keep turned OFF during the duty cycle */

      for (i = 0; i < delay; i++)
        {
           asm("nop");
        }

      /* Turn ON for a constant time */

      /* enable_out(1); */

      for (i = 0; i < 30000; i++)
        {
           asm("nop");
        }

      /* enable_out(0); */

      rcvd = read(fd_bt, buf, SIZE);

      for (i = 0; i < rcvd; i++)
        {
          /* Show received byte */

          printf("0x%02X\n", (unsigned char) buf[i]);

          cmd_data[cmd_pos] = buf[i];

          if ((cmd_pos == 0 && cmd_data[0] != 0xaa) ||
              (cmd_pos == 1 && cmd_data[1] != 0x55) ||
              (cmd_pos == 2 && !validcmd(cmd_data[2])))
            {
              cmd_pos = 0;
            }
          else
            {
              cmd_pos++;
            }

          /* We don't have command more than 9 bytes */

          if (cmd_pos > 9)
            {
              cmd_pos = 0;
            }

          /* Verify if we got a valid command */

          if ((cmd_pos == 5) &&
              (cmd_data[2] == READ_TEMPERATURE ||
               cmd_data[2] == READ_SETTINGS))
            {
              cmd_pos = 0;
              new_cmd = 1;
              break;
            }

          if ((cmd_pos == 6) &&
              (cmd_data[2] == SET_DUTY_CYCLE    ||
               cmd_data[2] == SET_SENSOR_OFFSET ||
               cmd_data[2] == SET_LCD_BACKLIGHT ||
               cmd_data[2] == SET_LCD_CONTRAST))
            {
              cmd_pos = 0;
              new_cmd = 1;
              break;
            }
        }

      if (new_cmd)
        {
          /* Read temperature */

          read(fd_temp, &temp, 2);

          temp = temp / 4;

          /* printf("Temperature = %d!\n", temp);
           * printf("Duty cycle = %d!\n", duty);
           * printf("New command received = %d!\n", cmd_data[2]);
           */

          if (cmd_data[2] == READ_SETTINGS)
            {
              cmd_data[5] = ACK;
              cmd_data[6] = 1;
              cmd_data[7] = 50;
              cmd_data[8] = 50;

              write(fd_bt, cmd_data, 9);
              fsync(fd_bt);

              new_cmd = 0;
              cmd_pos = 0;
            }

          if (cmd_data[2] == READ_TEMPERATURE)
            {
              cmd_data[5] = ACK;
              cmd_data[6] = temp & 0xff;
              cmd_data[7] = (temp >> 8);
              cmd_data[8] = 1;

              write(fd_bt, cmd_data, 9);
              fsync(fd_bt);

              new_cmd = 0;
              cmd_pos = 0;
            }

          if (cmd_data[2] == SET_DUTY_CYCLE)
            {
              /* If duty cycle lesser than 25% temperature decreases
               * if (cmd_data[5] < 25)
               * {
               *   if (temp > 20)
               *     {
               *       temp--;
               *     }
               * }
               * else
               * {
               *   if (temp < 250)
               *     {
               *       temp++;
               *     }
               */

              duty = cmd_data[5];

              cmd_data[5] = ACK;
              cmd_data[6] = temp & 0xff;
              cmd_data[7] = (temp >> 8);
              cmd_data[8] = 1;

              write(fd_bt, cmd_data, 9);
              fsync(fd_bt);

              new_cmd = 0;
              cmd_pos = 0;
            }
        }

      fflush(stdout);
    }

  return 0;
}
