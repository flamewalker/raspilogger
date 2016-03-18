/*
 * SPI master interface to communicate with I2C datalogger
 * ver 0.8.2
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

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

// Buffer for building datetime in log
char buff[20];

// Pointers to arrays for storing sample_template and sampled values
uint8_t *datalog, *sample_template;

// Variables for keeping track of areas of sampling
uint8_t array_size,settings,historical,systime,current,alarms;

// Function declarations
void init_arrays(void);

int find_reg(uint8_t reg);

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

// Main program
int main(void)
{
  // Create initial space before checking template file and setting actual values
  datalog=(uint8_t*)malloc(sizeof(uint8_t));
  sample_template=(uint8_t*)malloc(sizeof(uint8_t));

  init_arrays();

  time_t now = time(NULL);
  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));

  fprintf(stdout, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  fprintf(stderr, "%s %s\n", buff, "Initialize error log. SPI_LOGGER ver 0.8.2");

  if (wiringPiSPISetup (0, SPEED) < 0)
  {
    now = time(NULL);
    strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(stderr, "%s %s\n", buff, "Unable to open SPI device 0: %s", strerror(errno));
    free(datalog);
    free(sample_template);
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
    free(datalog);
    free(sample_template);
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

void init_arrays(void)
{
  // Open sample_template file and load fresh values
  FILE *sp;
  sp = fopen("/home/pi/spi-logger/sample_template.dat", "r");
  int data,inc,count;
  char ch;
  count=0;

  // First read the size of the sample_template array
  fscanf(sp,"%x%c", &data, &ch);
  array_size = data;
  // Make room for arrays
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
  time_t now = time(NULL);
  strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
  fprintf(stderr, "%s %s %s\n", buff, mysql_error(conn));
  mysql_close(conn);
  free(datalog);
  free(sample_template);
  exit(1);
}

void myInterrupt(void)
{
  // String to use for building MySQL query
  char sql_string[500];

  // Reset watchdog flag for SPI connection alive
  conn_alive = 1;

  // Every C prog must have an x counter... and a y variable
  uint8_t x,y;

  // Transmission buffer for SPI, preloaded with SAMPLE_DUMP command
  unsigned char buffer = 0x04;

  wiringPiSPIDataRW(0,&buffer,1);
  buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
  wiringPiSPIDataRW(0,&buffer,1);

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0x04)
  {
    if (buffer != 0x01)
    {
       time_t now = time(NULL);
       strftime(buff, sizeof(buff), "%Y-%m-%d %H:%M:%S", localtime(&now));
       fprintf(stderr, "%s %s %d\n", buff, "Sample collection returned error:", buffer);
    }
    return;
  }

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
    strcat(sql_string, "LOAD DATA INFILE '/tmp/settings.csv' INTO TABLE SETTINGS FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // HISTORICAL table
  if (sample_sent & 4)
  {
    fp = fopen("/tmp/historical.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp,"%02u%02u%02u,", find_reg(0x78),find_reg(0x77),find_reg(0x76));
    fprintf(fp,"%u,", find_reg(0x87));
    fprintf(fp,"%02u%02u%02u,", find_reg(0x7B),find_reg(0x7A),find_reg(0x79));
    fprintf(fp,"%02u:%02u,", find_reg(0x86),find_reg(0x85));
    for (x=0x7F ; x < 0x84 ; x++)
        fprintf(fp, "%u," , find_reg(x));
    fprintf(fp,"%u", find_reg(0x84));
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/historical.csv' INTO TABLE HISTORICAL FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
  }

  // CURRENT table
  if (sample_sent & 2)
  {
    fp = fopen("/tmp/current.csv", "w");
    fprintf(fp,"NULL,");
    for (x=historical ; x < current-1 ; x++)
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
