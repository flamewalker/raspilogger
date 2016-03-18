/*
 * SPI master interface to communicate with I2C datalogger
 * ver 0.8.5
 */

#include <stdio.h>
#include <errno.h>		// For translating errno to plain text
#include <stdlib.h>
#include <stdint.h>		// Needs this for uint8_t
#include <time.h>		// Need this for time in logs
#include <string.h>
#include <wiringPi.h>		// WiringPi GPIO library
#include <wiringPiSPI.h>	// WiringPi SPI library
#include <mysql/mysql.h>	// MySQL library
#include <fcntl.h>		// Need this for named pipes

#define PROG_VER "ver 0.8.5" 	// Program version
#define SPEED 2000000		// SPI speed
#define INT_PIN 25		// BCM pin for interrupt from AVR
#define RESET_PIN 24		// BCM pin for reset AVR
#define FIFO_NAME "ctc_cmd"	// The named pipe

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

// Error counter for SPI connection
uint8_t error_count = 0;

// Watchdog flag to block interrupt firing more than once
uint8_t int_active = 0;

// Pointers to arrays for storing sample_template and sampled values
uint8_t *datalog, *sample_template;

// Variables for keeping track of areas of sampling
uint8_t array_size,settings,historical,systime,current,alarms,last24h,status;

// Transmission buffer for SPI (usable all over the place...)
unsigned char buffer = 0x00;

// Handle for named pipe
int readfd;

// Function declarations
void send_command(int, int);

int trySPIcon(void);

void init_arrays(void);

int find_reg(uint8_t);

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

void error_log(char *log_message, char *error_str);

// Main program
int main(void)
{
  // Greeting message to signal we're alive
  fprintf(stdout, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  error_log("Initialize error log. SPI_LOGGER",PROG_VER);

  // Initialize GPIO
  wiringPiSetupGpio();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(INT_PIN, INPUT);
  pullUpDnControl(INT_PIN, PUD_DOWN);

  // Initialize SPI communication
  if (wiringPiSPISetup(0, SPEED) < 0)
  {
    error_log("Unable to open SPI device 0:", strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Initialize the FIFO for commands
  if (access(FIFO_NAME, F_OK) == -1)		// Check if FIFO already exists
    if (mkfifo(FIFO_NAME, 0777) != 0)		// If not, then create FIFO
    {
      error_log("Could not create fifo:", FIFO_NAME);
      exit(EXIT_FAILURE);
    }
  readfd = open(FIFO_NAME, O_RDONLY | O_NONBLOCK);	// Open FIFO in read-only, non-blocking mode

  // Create initial space before checking template file and setting actual values
  datalog=(uint8_t*)malloc(sizeof(uint8_t));
  sample_template=(uint8_t*)malloc(sizeof(uint8_t));

  init_arrays();

  // Check if a sample is available before initializing interrupt and wait loop
  if (digitalRead(INT_PIN) == 1)
    myInterrupt();

  // Initialize interrupt handling procedure
  if (wiringPiISR(INT_PIN, INT_EDGE_RISING, &myInterrupt) < 0)
  {
    error_log("Unable to register ISR:", strerror(errno));
    free(datalog);
    free(sample_template);
    exit(EXIT_FAILURE);
  }

  // Endless loop waiting for IRQ
  fprintf(stdout, "Waiting for interrupt. Press CTRL+C to stop\n");

  while (1)
  {
    sleep(65);
    if (conn_alive == 1)	// Check if the SPI communication has been active in the last 65 seconds
      conn_alive = 0;
    else
    {
      if (trySPIcon() == 0)
      {
        error_log("SPI connection has not been in use for last 130 seconds, reset AVR!","");
        digitalWrite(RESET_PIN, LOW);
        sleep(1);
        digitalWrite(RESET_PIN, HIGH);
        error_count = 0;
      }
    }
  }
  return 0;
}

void send_command(int cmd, int arg)
{
  char buf[3];
  sprintf(buf,"%X", cmd);
  error_log("Command:", buf);
  buffer = 0xF1;			// Start COMMAND sequence
  wiringPiSPIDataRW(0,&buffer,1);
  buffer = cmd;				// Send cmd
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == 0xF1)
  {
    sprintf(buf,"%X", arg);
    error_log("Argument:", buf);
    buffer = arg;			// Send arg
    wiringPiSPIDataRW(0,&buffer,1);
  }
  buffer = 0xFF;			// Send as NO-OP
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == arg)
    error_log("Success!","");
  else
  {
    sprintf(buf,"%X", buffer);
    error_log("Failure!", buf);
  }
}

int trySPIcon(void)
{
  // Increase the number of times we have come here and break if more than 0
  if (error_count++ > 0)
    return 0;

  // Buffer for passing error code (int) to function (char)
  char buf[3];

  if (digitalRead(INT_PIN) == 1)	// Check if we somehow missed the interrupt
  {
    error_log("Somehow we have missed the interrupt, execute NOW!","");
    myInterrupt();
    error_count = 0;
    return 1;
  }
  else
  {
    // Load SPI transmission buffer with PING
    buffer = 0xFF;

    wiringPiSPIDataRW(0,&buffer,1);	// Send first command and recieve whatever is in the buffer
    buffer = 0xFF;			// Load with 0xFF again to PING
    wiringPiSPIDataRW(0,&buffer,1);	// Buffer should contain answer to previous send command

    switch(buffer)
    {
      case 0x00:
        error_log("AVR is out of I2C_SYNC with CTC!","");
        return 1;

      case 0x01:
        error_log("No new sample available for the last 65 seconds","");
        return 1;

      default:
        sprintf(buf,"%u",buffer);
        error_log("Strange fault:",buf);
        return 1;
    }
  }
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
  close(readfd);
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

  // Load SPI buffer with SAMPLE_DUMP command
  buffer = 0xF0;

  wiringPiSPIDataRW(0,&buffer,1);	// Send first command and recieve whatever is in the buffer
  buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
  wiringPiSPIDataRW(0,&buffer,1);	// Buffer should contain answer to previous send command

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0xF0)
  {
    // Buffer for passing error code (int) to function (char)
    char buf[3];
    sprintf(buf,"%u",buffer);
    error_log("Sample collection returned error:", buf);
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
  buffer = 0xFF;			// Send as NO-OP / PING
  wiringPiSPIDataRW(0,&buffer,1);
  int sample_sent = buffer;

  // If SYSTIME has changed, make sure we get a CURRENT sample also
  if (sample_sent & 1)
    sample_sent |= 2;

  // Execute right amount of loops to receive correct number of blocks from AVR
  if (sample_sent & 1)			// SYSTIME block (Area 2)
  {
    buffer = sample_template[settings];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "systime");
      int_active = 0;
      return;
    }
    for (x = settings+1 ; x < systime; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[systime-1] = buffer;
  }

  if (sample_sent & 8)			// SETTINGS block (Area 1)
  {
    buffer = sample_template[0];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "settings");
      int_active = 0;
      return;
    }
    for (x = 1 ; x < settings ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[settings-1] = buffer;
  }

  if (sample_sent & 16)			// ALARMS block (Area 5)
  {
    buffer = sample_template[current];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "alarms");
      int_active = 0;
      return;
    }
    for (x = current+1 ; x < alarms ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[alarms-1] = buffer;
  }

  if (sample_sent & 32)			// LAST_24H block (Area 6)
  {
    buffer = sample_template[alarms];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "last_24h");
      int_active = 0;
      return;
    }
    for (x = alarms+1 ; x < last24h ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[last24h-1] = buffer;
  }

  if (sample_sent & 64)			// STATUS block (Area 7)
  {
    buffer = sample_template[last24h];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "status");
      int_active = 0;
      return;
    }
    for (x = last24h+1 ; x < status ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[status-1] = buffer;
  }

  if (sample_sent & 4)			// HISTORICAL block (Area 3)
  {
    buffer = sample_template[systime];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "historical");
      int_active = 0;
      return;
    }
    for (x = systime+1 ; x < historical ; x++)
    {
      buffer =  sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[historical-1] = buffer;
  }

  if (sample_sent & 2)			// CURRENT block (Area 4)
  {
    buffer = sample_template[historical];
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == 0x01)
    {
      error_log("No sample available error!", "current");
      int_active = 0;
      return;
    }
    for (x = historical+1 ; x < current ; x++)
    {
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      datalog[x-1] = buffer;
    }
    buffer = 0xFF;			// Send as NO-OP
    wiringPiSPIDataRW(0,&buffer,1);
    datalog[current-1] = buffer;
  }

  buffer = 0xAD;			// End sample sending
  wiringPiSPIDataRW(0,&buffer,1);

  // The routine for checking if the named pipe has a command waiting
  int res;
  char str[6];

  res = read(readfd, str, 6);
  if(res == 6)
  {
    int cmd,arg;
    char ch;
    sscanf(str, "%2x%1c%2x", &cmd, &ch, &arg);
    if (cmd >= 0xDC && cmd <= 0xDE)
      send_command(cmd, arg);
  }
  else
    if (res != 0)
    {
      char buf[3];
      sprintf(buf,"%u",res);
      error_log("Read failed:", buf);
      error_log("Errno:", strerror(errno));
    }

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

  // ALARMS table
  if (sample_sent & 16)
  {
    fp = fopen("/tmp/alarms.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp, "%u,%u,%u,%u", datalog[alarms-4],datalog[alarms-3],datalog[alarms-2],datalog[alarms-1]);
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
    fprintf(fp, "%u,%u,%u,%u", datalog[status-4],datalog[status-3],datalog[status-2],datalog[status-1]);
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
