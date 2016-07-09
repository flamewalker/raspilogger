/*
 * SPI master interface to communicate with I2C datalogger
 * ver 0.8.7
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>				// For translating errno to plain text
#include <unistd.h>				// Need this for sleep
#include <stdint.h>				// Needs this for uint8_t
#include <time.h>				// Need this for time in logs
#include <wiringPi.h>				// WiringPi GPIO library
#include <wiringPiSPI.h>			// WiringPi SPI library
#include <mysql/mysql.h>			// MySQL library
#include <fcntl.h>				// Need this for named pipes
#include <signal.h>				// Need this for catching SIGIO

#define PROG_VER "ver 0.8.7-next"		// Program version
#define SPEED 2000000				// SPI speed in Hz. AVR in slave mode is only guaranteed to fosc/4 or lower, ie. 4 Mhz
#define INT_PIN 25				// BCM pin for interrupt from AVR
#define RESET_PIN 24				// BCM pin for reset AVR
#define FIFO_NAME "./ctc_cmd"			// The named pipe
#define CONF_NAME "./mysql.cnf"			// The configuration file for MySQL
#define SAMPLE_TEMPLATE "./sample_template.dat"	// The sample template file

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

// Error counter for SPI connection
uint8_t error_count = 0;

// Watchdog flag to block interrupt firing more than once
uint8_t int_active = 0;

// Pointers to arrays for storing sample_template and sampled values
uint8_t *datalog, *sample_template;

// OneWire temperature array
uint8_t temperaturearray[20];

// Variables for keeping track of areas of sampling
uint8_t array_size,settings,historical,systime,current,alarms,last24h,status;

// Transmission buffer for SPI (usable all over the place...)
unsigned char buffer = 0x00;

// Handle for named pipe
int readfd;

// Time related vars
time_t time_1, time_2, time_3;
uint8_t diff_t, diff_i, diff_s;

// Debug var
uint8_t dbg, duplicate_count, test4 = 0;
float err_ratio = 0;
float secs = 0;

// Function declarations
void fifo_reader(int);

void send_command(int, int);

void debug_msg(void);

int trySPIcon(void);

void init_arrays(void);

int find_reg(uint8_t);

uint8_t get_sample(uint8_t, uint8_t);

uint8_t get_onewire(uint8_t, uint8_t);

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

void error_log(char *log_message, char *error_str);

// Main program
int main(void)
{
  // Greeting message to signal we're alive
  printf("Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  error_log("Initialize error log. SPI_LOGGER",PROG_VER);

  // Initialize GPIO
  wiringPiSetupGpio();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(INT_PIN, INPUT);
  pullUpDnControl(INT_PIN, PUD_OFF);

  // Initialize SPI communication
  if (wiringPiSPISetup(0, SPEED) < 0)
  {
    error_log("Unable to open SPI device 0:", strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Initialize the FIFO for commands
  if (access(FIFO_NAME, F_OK) == -1)			// Check if FIFO already exists
    if (mkfifo(FIFO_NAME, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH) != 0)			// If not, then create FIFO
    {
      error_log("Could not create fifo:", FIFO_NAME);
      exit(EXIT_FAILURE);
    }
  readfd = open(FIFO_NAME, O_RDONLY | O_NONBLOCK);	// Open FIFO in read-only, non-blocking mode
  fcntl(readfd, F_SETOWN, getpid());			// Connect the FIFO to this programs PID
  fcntl(readfd, F_SETFL, O_ASYNC);			// Set the O_ASYNC on the FIFO
  signal(SIGIO, fifo_reader);				// Register our handler of SIGIO

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
  printf("Waiting for interrupt. Press CTRL+C to stop\n");

  // Var for error_log
  char buf[3];

  // Initialize time vars
  time_1 = time(NULL);
  time_2 = time_1;

  while (1)
  {
    time_3 = time(NULL);
    diff_t = time_2 + 70 - time_3;		// Calculate when 70 seconds have passed since last interrupt

    // Debug code
    if (dbg)
    {
      sprintf(buf, "%u )", diff_t);
      error_log("Sleep (", buf);
    }

    if ((sleep(diff_t)) == 0)
    {
      if (conn_alive == 1)			// Check if the SPI communication has been active in the last 65 seconds
        conn_alive = 0;
      else
        if (trySPIcon() == 0)
        {
	  debug_msg();
	  error_log("SPI connection has not been in use for last 140 seconds, reset AVR!","");
	  digitalWrite(RESET_PIN, LOW);
	  sleep(1);
	  digitalWrite(RESET_PIN, HIGH);
	  error_count = 0;
        }
    }
  }
  error_log("Unrecoverable ERROR!","");
  exit(EXIT_FAILURE);
}

void fifo_reader(int signum)
{
  // The routine for checking if the named pipe has a command waiting
  int res;
  char str[6];

  char buf[3];

  while((res=read(readfd, str, 6)) > 0)
  {
    if(res == 6)
    {
      int cmd,arg = 0;
      sscanf(str, "%2x %2x", &cmd, &arg);
      if (cmd >= 0xDC && cmd <= 0xDE)
        send_command(cmd, arg);
      else
      {
        if (cmd >= 0xA0 && cmd <= 0xAF)
        {
          buffer = cmd;					// Load with cmd to request variable
          wiringPiSPIDataRW(0,&buffer,1);		// Send first command and recieve whatever is in the buffer
          sprintf(buf,"%02X",buffer);
          error_log("Debug_msg, expect 0xAD:",buf);	// Should be 0xAD if previous sample was successfull
          buffer = 0xFF;				// Load with 0xFF to send PING
          wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
          sprintf(buf,"%u",buffer);
          error_log("Debug_msg, cmd result=:",buf);
        }
        else
        {
          switch(cmd)
          {
            case 0xB0:
              debug_msg();
              break;

            case 0xB1:
              sprintf(buf,"%.2f",err_ratio);
              error_log("Debug_msg, err_ratio=:",buf);
              sprintf(buf,"%.2f",secs);
              error_log("Debug_msg, secs=:",buf);
              break;

            default:
              dbg = !dbg;
              sprintf(buf, "%u", dbg);
              error_log("DEBUG MODE:", buf);
              if (dbg)
                debug_msg();
          }
        }
      }
    }
    else
    {
      char buf[3];
      sprintf(buf,"%u",res);
      error_log("Read failed:", buf);
      error_log("Errno:", strerror(errno));
    }
  }
}

void send_command(int cmd, int arg)
{
  char buf[3];
  sprintf(buf,"%02X", cmd);
  error_log("Command:", buf);
  buffer = 0xF1;			// Start COMMAND sequence
  wiringPiSPIDataRW(0,&buffer,1);
  buffer = cmd;				// Send cmd
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == 0xF1)
  {
    sprintf(buf,"%02X", arg);
    error_log("Argument:", buf);
    buffer = arg;			// Send arg
    wiringPiSPIDataRW(0,&buffer,1);
  }
  if (buffer == 0xFF)
    error_log("Command already in queue! Aborting!","");
  buffer = 0xFF;			// Send as NO-OP
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == arg)
    error_log("Success!","");
  else
  {
    sprintf(buf,"%02X", buffer);
    error_log("Failure!", buf);
  }
}

void debug_msg(void)
{
  char buf[3];
  error_log("------------------------------------------","");
  // Load SPI transmission buffer
  buffer = 0xA0;				// Load with A0 to request test1
  wiringPiSPIDataRW(0,&buffer,1);		// Send first command and recieve whatever is in the buffer
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, expect 0xAD:",buf);	// Should be 0xAD if previous sample was successfull
  buffer = 0xA1;				// Load with A1 to request test2
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%u",buffer);
  error_log("Debug_msg, test1=:",buf);
  buffer = 0xA2;				// Load with A2 to request test3
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%u",buffer);
  error_log("Debug_msg, test2=:",buf);
  buffer = 0xA3;				// Load with A3 to request count
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%u",buffer);
  error_log("Debug_msg, test3=:",buf);
  buffer = 0xA4;				// Load with A4 to request twi_rxBuffer[0]
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, count=:",buf);
  buffer = 0xA5;				// Load with A5 to request twi_rxBuffer[1]
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, twi_rxBuffer[0]=:",buf);
  buffer = 0xA6;				// Load with A6 to request twi_txBuffer[0]
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, twirtxBuffer[1]=:",buf);
  buffer = 0xA7;				// Load with A7 to request twi_txBuffer[1]
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, twi_txBuffer[0]=:",buf);
  buffer = 0xA8;				// Load with A8 to request slask_rx1
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, twi_txBuffer[1]=:",buf);
  buffer = 0xA9;				// Load with A9 to request slask_rx2
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, slask_rx1=:",buf);
  buffer = 0xAA;				// Load with AA to request slask_tx1
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, slask_rx2=:",buf);
  buffer = 0xAB;				// Load with AB to request slask_tx2
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, slask_tx1=:",buf);
  buffer = 0xFF;				// Load with FF to send PING
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  sprintf(buf,"%02X",buffer);
  error_log("Debug_msg, slask_tx2=:",buf);
  sprintf(buf,"%u",test4);
  error_log("Debug_msg, samples=:",buf);
  sprintf(buf,"%u",duplicate_count);
  error_log("Debug_msg, duplicate_count=:",buf);
  error_log("------------------------------------------","");
}

int trySPIcon(void)
{
  // Buffer for passing error code (int) to function (char)
  char buf[3];

  if (digitalRead(INT_PIN) == 1)		// Check if we somehow missed the interrupt
  {
    error_log("TrySPI: Somehow we have missed the interrupt, execute NOW!","");
    myInterrupt();
    error_count = 0;
    return 1;
  }
  else
  {
    // Set a new time for the next waiting period
    time_2 = time(NULL);

    // Increase the number of times we have come here and break if more than 0
    if (error_count++ > 0)
      return 0;

    debug_msg();

    // Check SPI transmission
    buffer = 0xF2;				// Load with F2 to request TWI RESET
    wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command, probably 0xAD if previos sample was OK
    buffer = 0xFF;				// Load with FF to send PING
    wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
    switch(buffer)
    {
      case 0x00:
        error_log("AVR is out of I2C_SYNC with CTC!","");
        return 1;

      case 0x01:
        error_log("No new sample available for the last 70 seconds","");
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
  fprintf(stderr, "%s %s\n", log_message, error_str);
} // End of error_log

void init_arrays(void)
{
  // Open sample_template file and load fresh values
  FILE *sp;
  sp = fopen(SAMPLE_TEMPLATE, "r");
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
  exit(EXIT_FAILURE);
}

uint8_t test_spi(uint8_t val)
{
  buffer = 0xFF;
  wiringPiSPIDataRW(0,&buffer,1);
  buffer = sample_template[val];
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == 0x01)
    return 0;
  else
    return 1;
}

uint8_t get_sample(uint8_t start, uint8_t stop)
{
  buffer = sample_template[start];
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == 0x01)
    if (test_spi(start) == 0)
    {
      // Buffer for passing error code (int) to function (char)
      char buf[3];
      sprintf(buf,"%02X", stop);
      error_log("Get_sample: No sample available error!", buf);
      int_active = 0;
      return 0;
    }

  // Every C prog must have an x counter
  uint8_t x;

  for (x = start+1 ; x < stop ; x++)
  {
    buffer = sample_template[x];
    if (wiringPiSPIDataRW(0,&buffer,1) < 0)
      error_log("Get_sample: Error when calling wiringPiSPIDataRW:", strerror(errno));

    if (buffer == sample_template[x-1])		// Check if we got back a copy of what we sent
    {
      char buf[3];
      sprintf(buf,"%02X", buffer);
      error_log("Get_sample: Duplicate answer from register:", buf);
      buffer = sample_template[x-1];
      usleep(1);				// Short delay to allow AVR to catch up
      wiringPiSPIDataRW(0,&buffer,1);
      buffer = sample_template[x];
      wiringPiSPIDataRW(0,&buffer,1);
      duplicate_count++;
    }
    datalog[x-1] = buffer;
  }
  buffer = 0xFF;				// Send as NO-OP
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == sample_template[stop-1])	// Check if we got back a copy of what we sent
  {
    char buf[3];
    sprintf(buf,"%02X", buffer);
    error_log("Get_sample: Duplicate answer from register:", buf);
    buffer = sample_template[stop-1];
    usleep(1);				// Short delay to allow AVR to catch up
    wiringPiSPIDataRW(0,&buffer,1);
    buffer = 0xFF;
    wiringPiSPIDataRW(0,&buffer,1);
    duplicate_count++;
  }
  datalog[stop-1] = buffer;
  return 1;
}

uint8_t get_onewire(uint8_t start, uint8_t stop)
{
  buffer = (0xB0 + start);
  wiringPiSPIDataRW(0,&buffer,1);
  // Every C prog must have an x counter
  uint8_t x;

  for (x = start+1 ; x < stop ; x++)
  {
    buffer = (0xB0 + x);
    wiringPiSPIDataRW(0,&buffer,1);
    if (buffer == (0xB0 + x - 1))		// Check if we got back a copy of what we sent
    {
      char buf[3];
      sprintf(buf,"%02X", buffer);
      error_log("Get_onewire: Duplicate answer from register:", buf);
      buffer = (0xB0 + x - 1);
      usleep(1);				// Short delay to allow AVR to catch up
      wiringPiSPIDataRW(0,&buffer,1);
      buffer = (0xB0 + x);
      wiringPiSPIDataRW(0,&buffer,1);
      duplicate_count++;
    }
    temperaturearray[x-1] = buffer;
  }
  buffer = 0xFF;
  wiringPiSPIDataRW(0,&buffer,1);
  if (buffer == (0xB0 + stop - 1))		// Check if we got back a copy of what we sent
  {
    char buf[3];
    sprintf(buf,"%02X", buffer);
    error_log("Get_onewire: Duplicate answer from register:", buf);
    buffer = (0xB0 + stop - 1);
    usleep(1);				// Short delay to allow AVR to catch up
    wiringPiSPIDataRW(0,&buffer,1);
    buffer = 0xFF;
    wiringPiSPIDataRW(0,&buffer,1);
    duplicate_count++;
  }
  temperaturearray[stop-1] = buffer;
  return 1;
}

void myInterrupt(void)
{
  char buf[3];

  // Check if we somehow have called this procedure already...
  if (int_active == 1)
  {
    error_log("ISR: Interrupt called when active!!","");
    return;
  }

  // Check if the signal pin is high, if not something is wrong...
  if (digitalRead(INT_PIN) == 0)
  {
    error_log("ISR: Interrupt pin is not active!","");
    return;
  }
  // Set watchdog flag to indicate we're handling this IRQ
  int_active = 1;

  // Load SPI buffer with SAMPLE_DUMP command
  buffer = 0xF0;
  wiringPiSPIDataRW(0,&buffer,1);	// Send first command and recieve whatever is in the buffer

  if (buffer != 0xAD && buffer != 0xAF && buffer != 0x01 && buffer != 0xFF)
  {
    sprintf(buf,"%02X",buffer);
    error_log("ISR: Throwaway value:", buf);
  }

  buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
  wiringPiSPIDataRW(0,&buffer,1);	// Buffer should contain answer to previous send command

  // Reset watchdog flags for SPI connection alive
  conn_alive = 1;
  error_count = 0;

  // Set time vars
  time_1 = time(NULL);
  diff_i = time_1 - time_2;
  diff_s = time_1 - time_3;
  time_2 = time_1;

  // Debug code
  if (dbg)
  {
    char buf[3];
    sprintf(buf, "%2u", diff_i);
    error_log("ISR: Time since last =", buf);
    sprintf(buf, "%2u", diff_s);
    error_log("ISR: Time since sleep=", buf);
  }

  // Check if SAMPLE_DUMP command was acknowledged
  if (buffer != 0xF0)
  {

    if (buffer != 0xF0)
    {
      sprintf(buf,"%02X",buffer);
      error_log("ISR: SAMPLE_DUMP not ack'd:", buf);
    }
    buffer = 0xFF;
    wiringPiSPIDataRW(0,&buffer,1);	// Send first command and recieve whatever is in the buffer

    if (buffer != 0xFE)
    {
    sprintf(buf,"%02X",buffer);
    error_log("ISR: Expect FE:", buf);
    }

    buffer = 0xFE;			// Load with 0xFE to command next reply to be how many blocks to expect, if any
    wiringPiSPIDataRW(0,&buffer,1);	// Buffer should contain answer to previous send command

    if (buffer != 0xFF)
    {
    sprintf(buf,"%02X",buffer);
    error_log("ISR: Expect FF:", buf);
    }

    if (buffer != 0xFF)
    {
      // Buffer for passing error code (int) to function (char)
      char buf[3];
      sprintf(buf,"%02X",buffer);
      error_log("ISR: Sample collection returned error:", buf);
      int_active = 0;
      buffer = 0xAD;
      wiringPiSPIDataRW(0,&buffer,1);
      return;
    }
  }

  // String to use for building MySQL query, last time I checked it was 1044 chars... So remember to add if needed
  char sql_string[1050];

  // Every C prog must have an x counter... and a y variable
  uint8_t x,y;

  // Check how many blocks to expect from AVR
  buffer = 0xFF;			// Send as NO-OP / PING
  wiringPiSPIDataRW(0,&buffer,1);

  int sample_sent = buffer;

  // If SYSTIME has changed, make sure we get a CURRENT sample also
  if (sample_sent & 1)
    sample_sent |= 2;

  // Execute right amount of loops to receive correct number of blocks from AVR
  if (sample_sent & 1)			// SYSTIME block (Area 2)
    if(!get_sample(settings, systime))
      return;
  if (sample_sent & 2)			// CURRENT block (Area 4)
    if(!get_sample(historical, current) || !get_onewire(0, 2))
      return;
  if (sample_sent & 4)			// HISTORICAL block (Area 3)
    if(!get_sample(systime, historical))
      return;
  if (sample_sent & 8)			// SETTINGS block (Area 1)
    if(!get_sample(0, settings))
      return;
  if (sample_sent & 16)			// ALARMS block (Area 5)
    if(!get_sample(current, alarms))
      return;
  if (sample_sent & 32)			// LAST_24H block (Area 6)
    if(!get_sample(alarms, last24h))
      return;
  if (sample_sent & 64)			// STATUS block (Area 7)
    if(!get_sample(last24h, status))
      return;

  buffer = 0xAD;			// End sample sending
  wiringPiSPIDataRW(0,&buffer,1);

  // Debug code
  buffer = 0xA1;				// Load with A1 to request test2
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  test4 = buffer;
  if (secs > 0)
    secs = (secs + (float)diff_i/test4) / 2;
  else
  {
    secs = (float)diff_i/test4;
    sprintf(buf, "%.2f", secs);
    error_log("ISR: Secs=", buf);
  }

  if (dbg)
  {
    sprintf(buf, "%.2f", (float)diff_i/test4);
    error_log("ISR: Seconds between=", buf);
  }
  buffer = 0xFF;				// Load with FF to PING
  wiringPiSPIDataRW(0,&buffer,1);		// Buffer should contain answer to previous send command
  if (buffer > 0 && buffer != 64)		// Buffer contains test2, if test2 > 0, then an error has occured
  {
    sprintf(buf, "%2u", buffer);
    error_log("ISR: test2 check=", buf);
    debug_msg();
  }
  if (buffer & 64)
  {
    buffer = 0xA0;
    wiringPiSPIDataRW(0,&buffer,1);
    buffer = 0xFF;
    wiringPiSPIDataRW(0,&buffer,1);
    if (err_ratio > 0)
      err_ratio = (err_ratio + (float)buffer/test4) / 2;
    else
    {
      err_ratio = (float)buffer/test4;
      sprintf(buf, "%.2f", err_ratio);
      error_log("ISR: Err_ratio=", buf);
    }
  }
  buffer = 0xAF;
  wiringPiSPIDataRW(0,&buffer,1);

  // Write downloaded array to files with some formatting to fit better into SQL database
  FILE *fp;

  // SYSTIME table
  sprintf(sql_string, "INSERT INTO TIME VALUES(NULL,NULL,'%02u:%02u',%u,%u);", datalog[systime-4], datalog[systime-3], datalog[systime-2], datalog[systime-1]);

  // SETTINGS table
  if (sample_sent & 8)
  {
    fp = fopen("/tmp/settings.csv", "w");
    fprintf(fp,"NULL,%.1f,", (float)datalog[0]/2);
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
            fprintf(fp, "%.1f,", (float)datalog[x]/2);
          else
            fprintf(fp, "%u," , datalog[x]);
    }
    fprintf(fp, "%u", datalog[x]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/current.csv' INTO TABLE CURRENT FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");

    fp = fopen("/tmp/onewire.csv", "w");
    fprintf(fp,"NULL,");
    fprintf(fp, "%u.%02u", temperaturearray[0], temperaturearray[1]);
    fclose(fp);
    strcat(sql_string, "LOAD DATA INFILE '/tmp/onewire.csv' INTO TABLE ONEWIRE FIELDS TERMINATED BY ',' LINES TERMINATED BY '\n' SET log_idx=LAST_INSERT_ID();");
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
  if (mysql_options(conn, MYSQL_READ_DEFAULT_FILE, CONF_NAME) != 0)
    finish_with_error(conn);

  if (!mysql_real_connect(conn, NULL, NULL, NULL, NULL, 0, NULL, CLIENT_MULTI_STATEMENTS))
    finish_with_error(conn);

  if (mysql_query(conn, sql_string))
    finish_with_error(conn);

  mysql_close(conn);

  // Set watchdog flag to indicate we're finished
  int_active = 0;
}
