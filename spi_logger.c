/*
 * SPI master interface to communicate with I2C datalogger
 * ver 0.8.1
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>           // Needs this for uint8_t
#include <time.h>             // Need this for time in logs
#include <string.h>
#include <wiringPi.h>         // WiringPi GPIO library
#include <wiringPiSPI.h>      // WiringPi SPI library
#include <mysql/mysql.h>      // MySQL library

#define PROG_VER "ver 0.8.1"  // Program version
#define SPEED 1000000         // SPI speed
#define INT_PIN 24            // BCM pin for interrupt from AVR
#define RESET_PIN 25          // BCM pin for reset AVR

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

// Watchdog flag to block interrupt firing more than once
uint8_t int_active = 0;

// Pointers to arrays for storing sample_template and sampled values
uint8_t *datalog, *sample_template;

// Variables for keeping track of areas of sampling
uint8_t array_size,settings,historical,systime,current,alarms,last24h,status;

// Function declarations
void init_arrays(void);

int find_reg(uint8_t reg);

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

void error_log(char *log_message, char *error_str);

// Main program
int main(void)
{
  // Initialize GPIO
  wiringPiSetupGpio();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(INT_PIN, INPUT);
  pullUpDnControl(INT_PIN, PUD_DOWN);

  // Create initial space before checking template file and setting actual values
  datalog=(uint8_t*)malloc(sizeof(uint8_t));
  sample_template=(uint8_t*)malloc(sizeof(uint8_t));

  init_arrays();

  fprintf(stdout, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  error_log("Initialize error log. SPI_LOGGER",PROG_VER);

  if (wiringPiSPISetup(0, SPEED) < 0)
  {
    error_log("Unable to open SPI device 0:", strerror(errno));
    free(datalog);
    free(sample_template);
    exit(1);
  }

  // Check if a sample is available before initializing interrupt and wait loop
  if (digitalRead(INT_PIN) == 1)
    myInterrupt();

  // Initialize interrupt handling procedure
  if (wiringPiISR(INT_PIN, INT_EDGE_RISING, &myInterrupt) < 0)
  {
    error_log("Unable to register ISR:", strerror(errno));
    free(datalog);
    free(sample_template);
    exit(1);
  }
  fprintf(stdout, "Waiting for interrupt. Press CTRL+C to stop\n");

  // Endless loop waiting for IRQ
  while (1)
  {
    sleep(65);
    if (conn_alive == 1)	// Check if the SPI communication has been active in the last 65 seconds
      conn_alive = 0;
    else
    {
      error_log("SPI connection has not been in use for last 65 seconds, reset AVR!","");
      digitalWrite(RESET_PIN, LOW);
      sleep(1);
      digitalWrite(RESET_PIN, HIGH);
    }
  }
  return 0 ;
}

void error_log(char *log_message, char *error_str)
{
  // Buffer for building datetime in log
  char buff[20];

  // Fetch actual time
  time_t now = time(NULL);

  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
  fprintf(stderr, "%s %s %s\n", buff, log_message, error_str);
} // End of error_log

void init_arrays(void)
{
  // Open sample_template file and load fresh values
  FILE *sp;
  sp = fopen("/home/pi/spi-logger/sample_template.dat", "r");
  int data,inc,count = 0;
  char ch;

  // First read the size of the sample_template array
  fscanf(sp,"%x%c", &data, &ch);
  array_size = data;

  // Make more room for arrays
  datalog=(uint8_t*)realloc(datalog,sizeof(uint8_t)*array_size);
  sample_template=(uint8_t*)realloc(sample_template,sizeof(uint8_t)*array_size);

  // Read size of area1
  fscanf(sp,"%x%c", &data, &ch);
  settings = data;

  // Read size of area2
  fscanf(sp,"%x%c", &data, &ch);
  systime = settings+data;

  // Read size of area3
  fscanf(sp,"%x%c", &data, &ch);
  historical = systime+data;

  // Read size of area4
  fscanf(sp,"%x%c", &data, &ch);
  current = historical+data;

  // Read size of area5
  fscanf(sp,"%x%c", &data, &ch);
  alarms = current+data;

  // Read size of area6
  fscanf(sp,"%x%c", &data, &ch);
  last24h = alarms+data;

  // Read size of area7
  fscanf(sp,"%x%c", &data, &ch);
  status = last24h+data;

  // Store sample_template in array
  while(EOF!=(inc=fscanf(sp,"%x%c", &data, &ch)) && inc == 2)
    sample_template[count++] = data;

  fclose(sp);
}

int find_reg(uint8_t reg)
{
  int i;
  for (i = 0 ; i < array_size ; i++)
  {
    if (sample_template[i] == reg)
      return datalog[i];
  }
  return 0xFF;
}

void finish_with_error(MYSQL *conn)
{
  error_log("MySQL error:", (char*) mysql_error(conn));
  mysql_close(conn);
  free(datalog);
  free(sample_template);
  exit(1);
}

void myInterrupt(void)
{
  // Check if we somehow have called this procedure already...
  if (int_active == 1)
  {
    error_log("Interrupt called when active!!","");
    return;
  }

  // Set watchdog flag to indicate we're handling this IRQ
  int_active = 1;

  // Transmission buffer for SPI, preloaded with SAMPLE_DUMP command
  unsigned char buffer = 0x04;

  wiringPiSPIDataRW(0,&buffer,1);	// Send first command and recieve whatever is in the buffer
  buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
  wiringPiSPIDataRW(0,&buffer,1);	// Buffer should contain answer to previous send command

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0x04)
  {
    if (buffer != 0x01)
    {
      // Buffer for passing error code (int) to function (char)
      char buf[3];
      sprintf(buf,"%u",buffer);
      error_log("Sample collection returned error:", buf);
    }
    int_active = 0;
    return;
  }

  // String to use for building MySQL query
  char sql_string[1000];

  // Every C prog must have an x counter... and a y variable
  uint8_t x,y;

  // Reset watchdog flag for SPI connection alive
  conn_alive = 1;

  // Check how many blocks to expect from AVR
  buffer = 0xFE;			// Send as NO-OP
  wiringPiSPIDataRW(0,&buffer,1);
  int sample_sent = buffer;

  // Execute right amount of loops to receive correct number of blocks from AVR
  if (sample_sent & 8)			// SETTINGS block (Area 1)
  {
    buffer = sample_template[0];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = 1 ; x < settings ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[settings-1] = buffer;
  }

  if (sample_sent & 1)			// SYSTIME block (Area 2)
  {
    buffer = sample_template[settings];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = settings+1 ; x < systime; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[systime-1] = buffer;
  }

  if (sample_sent & 16)			// ALARMS block (Area 5)
  {
    buffer = sample_template[current];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = current+1 ; x < alarms ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[alarms-1] = buffer;
  }

  if (sample_sent & 32)			// LAST_24H block (Area 6)
  {
    buffer = sample_template[alarms];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = alarms+1 ; x < last24h ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[last24h-1] = buffer;
  }

  if (sample_sent & 64)			// STATUS block (Area 7)
  {
    buffer = sample_template[last24h];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = last24h+1 ; x < status ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[status-1] = buffer;
  }

  if (sample_sent & 4)			// HISTORICAL block (Area 3)
  {
    buffer = sample_template[systime];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = systime+1 ; x < historical ; x++)
    {
      buffer =  sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[historical-1] = buffer;
  }

  if (sample_sent & 2)			// CURRENT block (Area 4)
  {
    buffer = sample_template[historical];
    wiringPiSPIDataRW(0,&buffer,1);
    for (x = historical+1 ; x < current ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFE;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[current-1] = buffer;
  }

  // Fix until AVR prog changes, after that delete this!
  if (sample_sent & 2)
    sample_sent |= 80;
  if (sample_sent & 4)
    sample_sent |= 32;
  //

  buffer = 0xAD;			// End sample sending
  wiringPiSPIDataRW(0,&buffer,1);

  // Write downloaded array to files with some formatting to fit better into SQL database
  FILE *fp;

  // SYSTIME table
  sprintf(sql_string, "INSERT INTO TIME VALUES(NULL,NULL,'%02u:%02u',%u,%u);", datalog[systime-4], datalog[systime-3], datalog[systime-2], datalog[systime-1]);

  // SETTINGS table
  if (sample_sent & 8)
  {
    fp = fopen("/tmp/settings.csv", "w");
    fprintf(fp,"NULL,%f,", (float)datalog[0]/2);
    for (x = 1 ; x < settings-1 ; x++)
    {
      y = sample_template[x];
      if (y ==0x06 || y ==0x07 || y == 0x13 || y == 0x15 || y == 0x1E || y == 0x1F || y == 0x3A || y == 0x41)
        fprintf(fp, "%d,", datalog[x]-40);
      else
	if (y == 0x31)
	{
	  fprintf(fp, "%u0,%u0,%u0,", datalog[x], datalog[x+1], datalog[x+2]);
	  x = x+2;
	}
	else
          if (y == 0x16)
	  {
            fprintf(fp, "%u%u%u,", datalog[x], datalog[x+1], datalog[x+2]);
 	    x = x+2;
	  }
          else
            fprintf(fp, "%u,", datalog[x]);
    }
    fprintf(fp, "%u", datalog[x]);
    fclose(fp);
    strcat(sql_string, "DELETE FROM SETTINGS; LOAD DATA INFILE '/tmp/settings.csv' INTO TABLE SETTINGS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // HISTORICAL table
  if (sample_sent & 4)
  {
    fp = fopen("/tmp/historical.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp,"%02u%02u%02u,", find_reg(0x78),find_reg(0x77),find_reg(0x76));
    fprintf(fp,"%u,", find_reg(0x87));
    fprintf(fp,"%02u%02u%02u,", find_reg(0x7B),find_reg(0x7A),find_reg(0x79));
    for (x=0x80 ; x < 0x84 ; x++)
        fprintf(fp, "%u," , find_reg(x));
    fprintf(fp,"%u", find_reg(0x84));
    fclose(fp);
    strcat(sql_string, "DELETE FROM HISTORICAL; LOAD DATA INFILE '/tmp/historical.csv' INTO TABLE HISTORICAL FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // CURRENT table
  if (sample_sent & 2)
  {
    fp = fopen("/tmp/current.csv", "w");
    fprintf(fp,"NULL,");
    for (x=historical+4 ; x < current-5 ; x++)
    {
      y = sample_template[x];
      if (y == 0x8E || y == 0x94 || y == 0x95)
        fprintf(fp, "%d," , datalog[x]-40);
      else
        if (y == 0x8F)
          fprintf(fp, "%u.%u,", datalog[x++], datalog[x]);
        else
          if (y == 0xAA)
            fprintf(fp, "%f,", (float)datalog[x]/2);
          else
            fprintf(fp, "%u," , datalog[x]);
    }
    fprintf(fp, "%u", datalog[x]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/current.csv' INTO TABLE CURRENT FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // ALARMS table
  if (sample_sent & 16)
  {
    fp = fopen("/tmp/alarms.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp, "%u,%u,%u,%u", datalog[current-4],datalog[current-3],datalog[current-2],datalog[current-1]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/alarms.csv' INTO TABLE ALARMS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // LAST_24H table
  if (sample_sent & 32)
  {
    fp = fopen("/tmp/last_24h.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp,"%02u:%02u,%u", find_reg(0x86),find_reg(0x85),find_reg(0x7F));
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/last_24h.csv' INTO TABLE LAST_24H FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // STATUS table
  if (sample_sent & 64)
  {
    fp = fopen("/tmp/status.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp, "%u,%u,%u,%u", datalog[historical],datalog[historical+1],datalog[historical+2],datalog[historical+3]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/status.csv' INTO TABLE STATUS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
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

  // Set watchdog flag to indicate we're finished
  int_active = 0;
}
