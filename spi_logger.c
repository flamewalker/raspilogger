/*
 * SPI master interface to communicate with I2C datalogger
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define ARRAY_SIZE 0xDC
#define SPEED 500000

uint8_t datalog[ARRAY_SIZE];

/*
 * myInterrupt:
 *********************************************************************************
 */

void myInterrupt (void)
{
  uint8_t x;
  unsigned char buffer;
  buffer = 0x04;
  wiringPiSPIDataRW(0,&buffer,1);
  buffer = 0x00;
  wiringPiSPIDataRW(0,&buffer,1);
  for (x = 1; x < ARRAY_SIZE; x++)
  {
    buffer = x;
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[x-1] = buffer;
  }
  buffer = 0xFF;
  wiringPiSPIDataRW(0,&buffer,1);
  datalog[0xDB] = buffer;
}

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (void)
{
  printf("Raspberry Pi wiringPi SPI program\n");

  if (wiringPiSPISetup (0, SPEED) < 0)
  {
    fprintf(stderr, "Unable to open SPI device 0: %s\n", strerror (errno));
    exit(1);
  }

  printf("Starting\n");
  wiringPiSetupGpio();
  wiringPiISR (24, INT_EDGE_RISING, &myInterrupt) ;
  printf("Waiting for interrupt. Press CTRL+C to stop\n");
  for (;;)
  {

  }
  return 0 ;
}
