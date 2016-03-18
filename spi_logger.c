/*
 * SPI master interface to communicate with I2C datalogger
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>         // Needs this for uint8_t
#include <time.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mysql/mysql.h>

#define ARRAY_SIZE 0xDC
#define SETTINGS 0x72
#define HISTORICAL 0x8B
#define CURRENT 0xAC

#define SPEED 1000000       // SPI speed
#define INT_PIN 24          // BCM pin for interrupt from AVR
#define RESET_PIN 25        // BCM pin for reset AVR


// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

char buff[20];

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

int main(void)
{
  time_t now = time(NULL);
  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));

  fprintf(stdout, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  fprintf(stderr, "%s %s\n", buff, "Initialise error log");

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

  uint8_t datalog[ARRAY_SIZE];

  // Set watchdog flag for SPI connection alive
  conn_alive = 1;

  // Every C prog must have a x counter...
  uint8_t x;

  // Transmission buffer for SPI, preloaded with SAMPLE_DUMP command
  unsigned char buffer = 0x04;

  wiringPiSPIDataRW(0,&buffer,1);
  buffer = 0x00;
  wiringPiSPIDataRW(0,&buffer,1);

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0x04)
  {
    time_t now = time(NULL);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(stderr, "%s %s\n", buff, "Sample collection returned error");
    return;
  }

  // Loop for sampling entire array from AVR and store it
  for (x = 0x01 ; x < ARRAY_SIZE ; x++)
  {
    buffer = x;
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[x-1] = buffer;
  }
  buffer = 0xFF;
  wiringPiSPIDataRW(0,&buffer,1);
  datalog[ARRAY_SIZE-1] = buffer;

  // Write downloaded array to files with some formatting to fit better into SQL database
  // Start with SETTINGS table
  FILE *fp;
  fp = fopen("/tmp/settings.csv", "w");
  fprintf(fp,"NULL,%f,", (float)datalog[0]/2);
  for (x = 1 ; x < SETTINGS ; x++)
  {
    if (x ==0x06 || x ==0x07 || x == 0x13 || x == 0x15 || x == 0x1E || x == 0x1F || x == 0x3A)
      fprintf(fp, "%d,", datalog[x]-40);
    else
      if (x == 0x16)
        fprintf(fp, "%u%u%u,", datalog[x], datalog[x++], datalog[x++]);
      else
        fprintf(fp, "%u,", datalog[x]);
  }
  fprintf(fp, "%u", datalog[x]);
  fclose(fp);

  // Continue with HISTORICAL table
  fp = fopen("/tmp/historical.csv", "w");
  fprintf(fp,"NULL,");
  fprintf(fp,"%u,",datalog[0x73]);
  fprintf(fp,"%02u:%02u,", datalog[0x74], datalog[0x75]);
  fprintf(fp,"%02u%02u%02u,", datalog[0x78],datalog[0x77],datalog[0x76]);
  fprintf(fp,"%02u%02u%02u,", datalog[0x7B],datalog[0x7A],datalog[0x79]);
  for (x=0x7C ; x < 0x85 ; x++)
      fprintf(fp, "%u," , datalog[x]);
  fprintf(fp,"%02u:%02u,", datalog[0x86],datalog[0x85]);
  fprintf(fp,"%u,", datalog[0x87]);
  fprintf(fp,"%02u%02u%02u,", datalog[0x8A],datalog[0x89],datalog[0x88]);
  fprintf(fp, "%u", datalog[0x8B]);
  fclose(fp);

  // Finish with SETTINGS table
  fp = fopen("/tmp/current.csv", "w");
  fprintf(fp,"NULL,");
  for (x=HISTORICAL+1 ; x < CURRENT-2 ; x++)
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

  // MySQL magic happens here...
  MYSQL *conn = mysql_init(NULL);
  char *server = "localhost";
  char *user = "xxx";
  char *password = "xxx";
  char *database = "ctclog";

  if (!mysql_real_connect(conn, server, user, password, database, 0, NULL, CLIENT_MULTI_STATEMENTS))
    finish_with_error(conn);

  if (mysql_query(conn, "LOAD DATA INFILE '/tmp/settings.csv' INTO TABLE SETTINGS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=null;\
      LOAD DATA INFILE '/tmp/historical.csv' INTO TABLE HISTORICAL FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=null;\
      LOAD DATA INFILE '/tmp/current.csv' INTO TABLE CURRENT FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=null;\
      INSERT INTO TIME VALUES(NULL,NULL)"))
    finish_with_error(conn);

  mysql_close(conn);
}
