/*
 * SPI master interface to communicate with I2C datalogger
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>         // Needs this for uint8_t
#include <time.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mysql/mysql.h>

#define SPEED 1000000       // SPI speed
#define INT_PIN 24          // BCM pin for interrupt from AVR
#define RESET_PIN 25        // BCM pin for reset AVR

#define array_size 0xDC
#define settings 0x72
#define systime 0x75
#define historical 0x8B
#define current 0xAC

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

char buff[20];

// Function declarations
void myInterrupt(void);

void finish_with_error(MYSQL *conn);

// Main program
int main(void)
{
  time_t now = time(NULL);
  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));

  fprintf(stdout, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  fprintf(stderr, "%s %s\n", buff, "Initialize error log. SPI_LOGGER ver 0.8.0");

  if (wiringPiSPISetup (0, SPEED) < 0)
  {
    now = time(NULL);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(stderr, "%s %s\n", buff, "Unable to open SPI device 0: %s", strerror(errno));
    exit(1);
  }

  wiringPiSetupGpio();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(INT_PIN, INPUT);
  pullUpDnControl(INT_PIN, PUD_DOWN);
  if (wiringPiISR (INT_PIN, INT_EDGE_RISING, &myInterrupt) < 0)
  {
    now = time(NULL);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(stderr, "%s %s\n", buff, "Unable to register ISR: %s", strerror(errno));
    exit(1);
  }
  fprintf(stdout, "Waiting for interrupt. Press CTRL+C to stop\n");
  while (1)
  {
    sleep(65);
    if (conn_alive == 1)
      conn_alive = 0;
    else
    {
      now = time(NULL);
      strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
      fprintf(stderr, "%s %s\n", buff, "SPI connection has not been in use for last 65 seconds, reset AVR!");
      digitalWrite(RESET_PIN, LOW);
      sleep(1);
      digitalWrite(RESET_PIN, HIGH);
    }
  }
  return 0 ;
}

void finish_with_error(MYSQL *conn)
{
  time_t now = time(NULL);
  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
  fprintf(stderr, "%s %s %s\n", buff, mysql_error(conn));
  mysql_close(conn);
  exit(1);
}

void myInterrupt(void)
{
  // Array for storing samples
  uint8_t datalog[array_size];

  // String to use for building MySQL query
  char sql_string[500] = "INSERT INTO TIME VALUES(NULL,NULL,NULL,NULL);";

  // Set watchdog flag for SPI connection alive
  conn_alive = 1;

  // Every C prog must have a x counter...
  uint8_t x;

  // Transmission buffer for SPI, preloaded with SAMPLE_DUMP command
  unsigned char buffer = 0x04;

  wiringPiSPIDataRW(0,&buffer,1);
  buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
  wiringPiSPIDataRW(0,&buffer,1);

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0x04)
  {
    time_t now = time(NULL);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(stderr, "%s %s\n", buff, "Sample collection returned error");
    return;
  }

  // Check how many blocks to expect from AVR
  buffer = 0xFE;			// Send as NO-OP
  wiringPiSPIDataRW(0,&buffer,1);
  int sample_sent = buffer;

  // Execute right amount of loops to receive correct number of blocks from AVR
  if (sample_sent & 8)			// SETTINGS block
  {
    buffer = 0x00;
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = 0x01 ; x <= settings ; x++)
    {
      buffer = x;
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[settings] = buffer;
  }

  if (sample_sent & 4)			// HISTORICAL block
  {
    buffer = 0x76;
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = 0x77 ; x <= historical ; x++)
    {
      buffer = x;
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[historical] = buffer;
  }

  if (sample_sent & 2)			// CURRENT block
  {
    buffer = 0x8C;
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = 0x8D ; x <= current ; x++)
    {
      buffer = x;
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[current] = buffer;
  }

  if (sample_sent & 1)			// SYSTIME block
  {
    buffer = 0x73;
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = 0x74 ; x <= systime; x++)
    {
      buffer = x;
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[systime] = buffer;
  }

  buffer = 0xAD;			// End sample sending
  wiringPiSPIDataRW(0,&buffer,1);

  // Write downloaded array to files with some formatting to fit better into SQL database
  FILE *fp;

  // SYSTIME table
  if (sample_sent & 1)
    sprintf(sql_string, "INSERT INTO TIME VALUES(NULL,NULL,'%02u:%02u',%u);", datalog[0x74], datalog[0x75], datalog[0x73]);

  // SETTINGS table
  if (sample_sent & 8)
  {
    fp = fopen("/tmp/settings.csv", "w");
    fprintf(fp,"NULL,%f,", (float)datalog[0]/2);
    for (x = 1 ; x < settings ; x++)
    {
      if (x ==0x06 || x ==0x07 || x == 0x13 || x == 0x15 || x == 0x1E || x == 0x1F || x == 0x3A || x== 0x41)
        fprintf(fp, "%d,", datalog[x]-40);
      else
	if (x == 0x31)
	{
	  fprintf(fp, "%u0,%u0,%u0,", datalog[x], datalog[x+1], datalog[x+2]);
	  x = 0x33;
	}
	else
          if (x == 0x16)
	  {
            fprintf(fp, "%u%u%u,", datalog[x], datalog[x+1], datalog[x+2]);
 	    x=0x18;
	  }
          else
            fprintf(fp, "%u,", datalog[x]);
    }
    fprintf(fp, "%u", datalog[x]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/settings.csv' INTO TABLE SETTINGS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // HISTORICAL table
  if (sample_sent & 4)
  {
    fp = fopen("/tmp/historical.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp,"%02u%02u%02u,", datalog[0x78],datalog[0x77],datalog[0x76]);
    fprintf(fp,"%02u%02u%02u,", datalog[0x7B],datalog[0x7A],datalog[0x79]);
    for (x=0x7C ; x < 0x85 ; x++)
        fprintf(fp, "%u," , datalog[x]);
    fprintf(fp,"%02u:%02u,", datalog[0x86],datalog[0x85]);
    fprintf(fp,"%u,", datalog[0x87]);
    fprintf(fp,"%02u%02u%02u,", datalog[0x8A],datalog[0x89],datalog[0x88]);
    fprintf(fp, "%u", datalog[0x8B]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/historical.csv' INTO TABLE HISTORICAL FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // CURRENT table
  if (sample_sent & 2)
  {
    fp = fopen("/tmp/current.csv", "w");
    fprintf(fp,"NULL,");
    for (x=historical+1 ; x < current-2 ; x++)
    {
      if (x == 0x8E || x == 0x94 || x == 0x95)
        fprintf(fp, "%d," , datalog[x]-40);
      else
        if (x == 0x8F)
          fprintf(fp, "%f,", (float)datalog[x++]+((float)datalog[x+1]/10));
        else
          fprintf(fp, "%u," , datalog[x]);
    }
    fprintf(fp, "%f,", (float)datalog[x++]/2);
    fprintf(fp, "%u,", datalog[x++]);
    fprintf(fp, "%u", datalog[x]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/current.csv' INTO TABLE CURRENT FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // MySQL magic happens here...
  MYSQL *conn = mysql_init(NULL);
  char *server = "localhost";
  char *user = "xxx";
  char *password = "xxx";
  char *database = "ctclog";

  if (!mysql_real_connect(conn, server, user, password, database, 0, NULL, CLIENT_MULTI_STATEMENTS))
    finish_with_error(conn);

  if (mysql_query(conn, sql_string))
    finish_with_error(conn);

  mysql_close(conn);
}
