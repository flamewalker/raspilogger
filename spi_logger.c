/*
 * SPI master interface to communicate with I2C datalogger
 * ver 0.8.8
 */

#include <stdlib.h>
#include <stdio.h>				// File I/O and such
#include <string.h>				// String handling functions
#include <unistd.h>				// Need this for sleep
#include <stdint.h>				// Needs this for uint8_t
#include <time.h>				// Need this for time in logs
#include <fcntl.h>				// Need this for named pipes
#include <signal.h>				// Need this for catching SIGIO
#include <wiringPi.h>				// WiringPi GPIO library
#include <mysql/mysql.h>			// MySQL library

#include <sys/ioctl.h>				// For SPI comms
#include <linux/types.h>			// For SPI comms
#include <linux/spi/spidev.h>			// Spidev functions

#define PROG_VER "ver 0.8.8-next.8"		// Program version
#define INT_PIN 25				// BCM pin for interrupt from AVR
#define RESET_PIN 24				// BCM pin for reset AVR
#define DHW_PIN 22				// BCM pin for signal DHW active
#define FIFO_NAME "./ctc_cmd"			// The named pipe
#define CONF_NAME "./mysql.cnf"			// The configuration file for MySQL
#define SAMPLE_TEMPLATE "./sample_template.dat"	// The sample template file
#define INT_TICK_1 100				// Threshold for error reporting of interrupt errors
#define INT_TICK_2 150				// Threshold for error reporting of interrupt errors
#define INT_TICK_3 200				// Threshold for error reporting of interrupt errors
#define INT_TICK_4 250				// Threshold for error reporting of interrupt errors
#define INT_TICK_5 300				// Threshold for error reporting of interrupt errors
#define ERROR_RATIO 0.1				// Threshold for error reporting of I2C errors

// SPIDEV variables
static const char *device = "/dev/spidev1.0";
static uint8_t mode = 0;
static uint32_t speed = 3000000;		// SPI speed in Hz. AVR in slave mode is only guaranteed to fosc/4 or lower, ie. 4MHz
static uint8_t bits = 8;
static uint8_t tx_buffer = 0x00;
static uint8_t rx_buffer = 0x00;
static int spihandle;

// Watchdog flag to check if SPI connection is alive
uint8_t conn_alive = 0;

// Error counter for SPI connection
uint8_t error_count = 0;

// Watchdog flag to block interrupt firing more than once
uint8_t int_active = 0;

// Pointers to arrays for storing sample_template and sampled values
uint8_t *datalog, *sample_template = NULL;

// Variables for keeping track of areas of sampling
uint8_t array_size, settings, historical, systime, current, alarms, last24h, status, temperature = 0;

// Handle for named pipe
int readfd;

// Time related vars
time_t time_1, time_2, time_3, dhw_start;
uint8_t diff_t, diff_i, diff_s;
clock_t tick_1, tick_2, tick_3;

// DHW active flag
uint8_t dhw = 0;

// Debug var
uint16_t not_active_count_1, not_active_count_2, not_active_count_3, not_active_count_4, not_active_count_5 = 0;
uint8_t dbg = 0;
uint8_t duplicate_count, test4 = 0;
float err_ratio = 0.0;
uint32_t tot_samples = 0;
uint32_t tot_error = 0;
float secs = 0.0;

// Function declarations
int spiTxRx(int fd, uint8_t *txDat, uint8_t *rxDat);

void fifo_reader(int);

void send_command(uint8_t, uint8_t);

void send_digi_command(uint8_t, uint8_t);

void set_temp(uint8_t);

void debug_msg(void);

int trySPIcon(void);

void init_arrays(void);

uint8_t get_sample(uint8_t, uint8_t);

void myInterrupt(void);

void finish_with_error(MYSQL *conn);

// Main program
int main(void)
{
  // Necessary to remove buffering on stdout to have anything written to journal
  setvbuf(stdout, NULL, _IONBF, 0);

  // Greeting message to signal we're alive
  fprintf(stderr, "Raspberry Pi SPI Master interface to AVR CTC-logger\n");
  fprintf(stderr, "Initialize error log. SPI_LOGGER %s\n",PROG_VER);

  // Initialize GPIO
  wiringPiSetupGpio();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(DHW_PIN, OUTPUT);
  digitalWrite(DHW_PIN, LOW);
  pinMode(INT_PIN, INPUT);
  pullUpDnControl(INT_PIN, PUD_OFF);

  // Initialize SPI communication
  spihandle = open(device, O_RDWR);
  ioctl (spihandle, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  ioctl (spihandle, SPI_IOC_WR_BITS_PER_WORD, &bits);
  ioctl (spihandle, SPI_IOC_WR_MODE, &mode);

  // Initialize the FIFO for commands
  if (access(FIFO_NAME, F_OK) == -1)							// Check if FIFO already exists
    if (mkfifo(FIFO_NAME, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH) != 0)			// If not, then create FIFO
    {
      fprintf(stderr, "Could not create FIFO: %s\n", FIFO_NAME);
      exit(EXIT_FAILURE);
    }
  if (chmod(FIFO_NAME, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH) != 0)	// Change permissions on FIFO
  {
    fprintf(stderr, "Could not change permissions on FIFO: %s\n", FIFO_NAME);
    exit(EXIT_FAILURE);
  }
  readfd = open(FIFO_NAME, O_RDONLY | O_NONBLOCK);		// Open FIFO in read-only, non-blocking mode
  fcntl(readfd, F_SETOWN, getpid());				// Connect the FIFO to this programs PID
  fcntl(readfd, F_SETFL, O_ASYNC);				// Set the O_ASYNC on the FIFO
  signal(SIGIO, fifo_reader);					// Register our handler of SIGIO

  init_arrays();

  // Check if a sample is available before initializing interrupt and wait loop
  if (digitalRead(INT_PIN) == 1)
  {
    fprintf(stderr, "MAIN: INT_PIN active during setup, calling ISR\n");
    myInterrupt();
  }

  // Initialize interrupt handling procedure
  if (wiringPiISR(INT_PIN, INT_EDGE_RISING, &myInterrupt) < 0)
  {
    perror("Unable to register ISR");
    free(datalog);
    datalog = NULL;
    free(sample_template);
    sample_template = NULL;
    close(readfd);
    close(spihandle);
    exit(EXIT_FAILURE);
  }
  // Init the tick counter
  tick_1 = clock();

  // Initialize time vars
  time_1 = time(NULL);
  time_2 = time_1;

  // Endless loop waiting for IRQ from AVR
  while (1)
  {
    time_3 = time(NULL);
    diff_t = time_2 + 70 - time_3;		// Calculate when 70 seconds have passed since last interrupt

    // Debug code
    if (dbg)
      fprintf(stderr, "Debug msg, Sleep ( %u )\n", diff_t);

    // DHW code
	if (dhw)
	  if ((time_3 - dhw_start) > 180)	// If 180 seconds have passed since we started DHW production
	  {
		digitalWrite(DHW_PIN, LOW);
		dhw = 0;
	    set_temp(60);
		fprintf(stderr, "DHW stop\n");
	  }

    if ((sleep(diff_t)) == 0)
    {
      if (conn_alive == 1)				// Check if the SPI communication has been active in the last 70 seconds
        conn_alive = 0;
      else
        if (trySPIcon() == 0)
        {
	      debug_msg();
	      fprintf(stderr, "SPI connection has not been in use for last 140 seconds, reset AVR!\n");
	      digitalWrite(RESET_PIN, LOW);
	      sleep(1);
	      digitalWrite(RESET_PIN, HIGH);
	      error_count = 0;
        }
    }
  }
  close(readfd);
  close(spihandle);
  free(datalog);
  datalog = NULL;
  free(sample_template);
  sample_template = NULL;
  fprintf(stderr, "Unrecoverable ERROR!\n");
  exit(EXIT_FAILURE);
}

int spiTxRx(int fd, uint8_t *txDat, uint8_t *rxDat)
{
  int ret;
  struct spi_ioc_transfer spi =
  {
    .tx_buf = (unsigned long)txDat,
    .rx_buf = (unsigned long)rxDat,
    .len = 1,
    .delay_usecs = 0,
    .speed_hz = speed,
    .bits_per_word = 8
  };
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
  if (ret < 1)
  {
    fprintf(stderr, "spiTxRx: Cant send message\n");
    return -1;
  }
  return 0;
}

void fifo_reader(int signum)
{
  // The routine for checking if the named pipe has a command waiting
  int res, cmd, arg, data = 0;
  char str[11];

  while((res = read(readfd, str, 11)) > 0)
  {
	switch(res)
	{
	  case 3:		// command only (HEX)
        sscanf(str, "%x", &cmd);
		switch(cmd)
        {
          case 0xB0:
            debug_msg();
            break;

          case 0xB1:
            err_ratio = (float)tot_error / (float)tot_samples * 100.0;
            fprintf(stderr, "Debug_msg, err_ratio=: %.4f%\n", err_ratio);
            fprintf(stderr, "Debug_msg, tot_error=: %u\n", tot_error);
            fprintf(stderr, "Debug_msg, tot_samples=: %u\n", tot_samples);
            fprintf(stderr, "Debug_msg, secs=: %.2f\n", secs);
            fprintf(stderr, "Debug_msg, not_active_count_1=: %u\n", not_active_count_1);
            fprintf(stderr, "Debug_msg, not_active_count_2=: %u\n", not_active_count_2);
            fprintf(stderr, "Debug_msg, not_active_count_3=: %u\n", not_active_count_3);
            fprintf(stderr, "Debug_msg, not_active_count_4=: %u\n", not_active_count_4);
            fprintf(stderr, "Debug_msg, not_active_count_5=: %u\n", not_active_count_5);
            break;

          case 0xB2:
            dbg = !dbg;
            fprintf(stderr, "DEBUG MODE: %u\n", dbg);
            if (dbg)
              debug_msg();
		    break;

	  case 0xB3:
		  	digitalWrite(DHW_PIN, HIGH);
		  	fprintf(stderr, "DHW start\n");
			dhw = 1;
			set_temp(30);
			dhw_start = time(NULL);
			break;

          default:
		    if (cmd >= 0xA0 && cmd <= 0xAF)
            {
              tx_buffer = cmd;												// Load with cmd to request variable
              spiTxRx(spihandle, &tx_buffer, &rx_buffer);						// Send first command and receive whatever is in the buffer
              fprintf(stderr, "Debug_msg, expect 0xAD: %02X\n", rx_buffer);	// Should be 0xAD if previous sample was successful
              tx_buffer = 0xFF;												// Load with 0xFF to send PING
              spiTxRx(spihandle, &tx_buffer, &rx_buffer);						// Buffer should contain answer to previous send command
              fprintf(stderr, "Debug_msg, cmd result=: %u\n", rx_buffer);
            }
			else
            {
		      fprintf(stderr, "FIFO_READ: Read failed: %u\n", res);
			  fprintf(stderr, "FIFO_READ: Cmd: %x\n", cmd);
			}
        }
	    break;

	  case 6:		// command and argument (HEX HEX/UINT)
        sscanf(str, "%x%x", &cmd, &arg);
        if (cmd >= 0xDC && cmd <= 0xDE)
          send_command(cmd, arg);
        else
        {
          if (cmd == 0xF2)
          {
            sscanf(str, "%x%u", &cmd, &arg);
            set_temp(arg);
          }
	  else
	  {
	    fprintf(stderr, "FIFO_READ: Read failed: %u\n", res);
	    fprintf(stderr, "FIFO_READ: Cmd: %x\tArg: %u\n", cmd, arg);
	  }
        }
	break;

      default:		// possibly command, argument and data (HEX UINT UINT)
        sscanf(str, "%x%u%u", &cmd, &arg, &data);
        if (cmd == 0xF3)
          send_digi_command(arg, data);
        else
        {
          fprintf(stderr, "FIFO_READ: Read failed: %u\n", res);
          fprintf(stderr, "FIFO_READ: Cmd: %x\tArg: %u\tData: %u\n", cmd, arg, data);
          perror("FIFO_READ");
        }
    }
  }
}

void send_command(uint8_t cmd, uint8_t arg)
{
  fprintf(stderr, "SEND_COMMAND: Command: %02X\n", cmd);
  tx_buffer = 0xF1;								// Start COMMAND sequence
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  tx_buffer = cmd;								// Send cmd
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == 0xF1)
  {
    fprintf(stderr, "SEND_COMMAND: Argument: %02X\n", arg);
    tx_buffer = arg;							// Send arg
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  }
  if (rx_buffer == 0xFF)
    fprintf(stderr, "SEND_COMMAND: Command already in queue! Aborting!\n");
  tx_buffer = 0xFF;								// Send as NO-OP
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == arg)
    fprintf(stdout, "SEND_COMMAND: Success!\n");
  else
    fprintf(stderr, "SEND_COMMAND: Failure! %02X\n", rx_buffer);
}

void send_digi_command(uint8_t cmd, uint8_t data)
{
  fprintf(stdout, "SEND_DIGI_COMMAND: Cmd+Adr: %u\n", cmd);
  tx_buffer = 0xF3;								// Start COMMAND sequence
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  tx_buffer = cmd;								// Send cmd+adr
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == 0xF3)
  {
    fprintf(stdout, "SEND_DIGI_COMMAND: Data: %u\n", data);
    tx_buffer = data;							// Send data
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  }
  tx_buffer = 0xFF;								// Send as NO-OP to receive answer from Digipot
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  fprintf(stdout, "SEND_DIGI_COMMAND: Answer: %u\n", rx_buffer);
}

void set_temp(uint8_t temp)
{
  fprintf(stdout, "SET_TEMP: Temp: %u\n", temp);
  tx_buffer = 0xF2;								// Start SET_TEMP sequence
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  tx_buffer = temp;								// Send temp
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == 0xF2)
  {
    tx_buffer = 0xFF;							// Send as NO-OP to receive answer from AVR
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    fprintf(stdout, "SET_TEMP: CTC Hot water: %u\n", rx_buffer);
  }
}

void debug_msg(void)
{
  fprintf(stderr, "------------------------------------------\n");
  // Load SPI transmission buffer
  tx_buffer = 0xA0;												// Load with A0 to request test1
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Send first command and receive whatever is in the buffer
  fprintf(stderr, "Debug_msg, expect 0xAF: %02X\n", rx_buffer);			// Should be 0xAF if previous sample was successful
  tx_buffer = 0xA1;												// Load with A1 to request test2
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, test1=: %u\n", rx_buffer);
  tx_buffer = 0xA2;												// Load with A2 to request test3
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, test2=: %u\n", rx_buffer);
  tx_buffer = 0xA3;												// Load with A3 to request count
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, test3=: %u\n", rx_buffer);
  tx_buffer = 0xA4;												// Load with A4 to request twi_rxBuffer[0]
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, count=: %02X\n", rx_buffer);
  tx_buffer = 0xA5;												// Load with A5 to request twi_rxBuffer[1]
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, twi_rxBuffer[0]=: %02X\n", rx_buffer);
  tx_buffer = 0xA6;												// Load with A6 to request twi_txBuffer[0]
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, twi_rxBuffer[1]=: %02X\n", rx_buffer);
  tx_buffer = 0xA7;												// Load with A7 to request twi_txBuffer[1]
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, twi_txBuffer[0]=: %02X\n", rx_buffer);
  tx_buffer = 0xA8;												// Load with A8 to request slask_rx1
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, twi_txBuffer[1]=: %02X\n", rx_buffer);
  tx_buffer = 0xA9;												// Load with A9 to request slask_rx2
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, slask_rx1=: %02X\n", rx_buffer);
  tx_buffer = 0xAA;												// Load with AA to request slask_tx1
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, slask_rx2=: %02X\n", rx_buffer);
  tx_buffer = 0xAB;												// Load with AB to request slask_tx2
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, slask_tx1=: %02X\n", rx_buffer);
  tx_buffer = 0xFF;												// Load with FF to send NO-OP/PING
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);					// Buffer should contain answer to previous send command
  fprintf(stderr, "Debug_msg, slask_tx2=: %02X\n", rx_buffer);
  fprintf(stderr, "Debug_msg, samples=: %u\n",test4);
  fprintf(stderr, "Debug_msg, duplicate_count=: %u\n",duplicate_count);
  fprintf(stderr, "------------------------------------------\n");
}

int trySPIcon(void)
{
  if (digitalRead(INT_PIN) == 1)				// Check if we somehow missed the interrupt
  {
    fprintf(stderr, "TrySPI: Somehow we have missed the interrupt, execute NOW!\n");
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
    tx_buffer = 0xF2;							// Load with F2 to request TWI RESET
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command, probably 0xAF if previous sample was OK
	usleep(1);									// Short delay to allow AVR to catch up
    tx_buffer = 0xFF;							// Load with FF to send PING
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command
    switch(rx_buffer)
    {
      case 0x00:
        fprintf(stderr, "TrySPI: AVR is out of I2C_SYNC with CTC!\n");
        return 1;

      case 0x01:
        fprintf(stderr, "TrySPI: No new sample available for the last 70 seconds\n");
        return 1;

      default:
        fprintf(stderr, "TrySPI: Strange fault: %u\n", rx_buffer);
        return 1;
    }
  }
}

void init_arrays(void)
{
  // Open file containing the sample templare and load fresh values
  FILE *fp;
  if ((fp = fopen(SAMPLE_TEMPLATE, "r")) == NULL)
  {
    perror("INIT_ARRAYS");
    exit(EXIT_FAILURE);
  }

  // Make initial room for the array holding the sample template
  if ((sample_template = (uint8_t*)calloc(0, sizeof(uint8_t))) == NULL)
  {
    perror("INIT_ARRAYS");
    fclose(fp);
    exit(EXIT_FAILURE);
  }

  int data, count = 0;
  char ch;

  // Store SETTINGS in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  settings = count;
  ch = 0;

  // Store SYSTIME in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  systime = count;
  ch = 0;

  // Store HISTORICAL in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  historical = count;
  ch = 0;

  // Store CURRENT in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  current = count;
  ch = 0;

  // Store ALARMS in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  alarms = count;
  ch = 0;

  // Store LAST_24H in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  last24h = count;
  ch = 0;

  // Store STATUS in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  status = count;
  ch = 0;

  // Store ONEWIRE in array
  while(ch != ':' && EOF != fscanf(fp,"%x%c", &data, &ch))	// Read HEX values until ":"
  {
    sample_template = realloc(sample_template, count+1);
    sample_template[count++] = data;
  }
  temperature = count;

  // Total array size
  array_size = count;

  // Make room for the array holding the sampled values
  if ((datalog = (uint8_t*)calloc(array_size, sizeof(uint8_t))) == NULL)
  {
    perror("INIT ARRAYS");
    fclose(fp);
    exit(EXIT_FAILURE);
  }

  fclose(fp);
}

void finish_with_error(MYSQL *conn)
{
  fprintf(stderr, "MySQL error: %s\n", (char*) mysql_error(conn));
  mysql_close(conn);
  free(datalog);
  datalog = NULL;
  free(sample_template);
  sample_template = NULL;
  close(readfd);
  close(spihandle);
  exit(EXIT_FAILURE);
}

uint8_t test_spi(uint8_t val)
{
  tx_buffer = 0xFF;
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  usleep(1);									// Short delay to allow AVR to catch up
  tx_buffer = sample_template[val];
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == 0x01)
    return 0;
  else
    return 1;
}

uint8_t get_sample(uint8_t start, uint8_t stop)
{
  tx_buffer = sample_template[start];
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  usleep(1);						// Short delay to allow AVR to prepare the ISR correctly

  // Every C prog must have an x counter
  uint8_t x;

  for (x = start+1 ; x < stop ; x++)
  {
    tx_buffer = sample_template[x];
    if (spiTxRx(spihandle, &tx_buffer, &rx_buffer) < 0)
      perror("Get_sample: Error when calling spiTxRx");

    if (rx_buffer == sample_template[x-1])		// Check if we got back a copy of what we sent
    {
      fprintf(stderr, "Get_sample: Duplicate answer from register: %02X\n", rx_buffer);
      fprintf(stderr, "Get_sample: X: %u\n", x);
      fprintf(stderr, "Get_sample: Start: %u\n", start);
      fprintf(stderr, "Get_sample: Stop: %u\n", stop);
      tx_buffer = sample_template[x-1];
      spiTxRx(spihandle, &tx_buffer, &rx_buffer);
      usleep(1);								// Short delay to allow AVR to catch up
      tx_buffer = sample_template[x];
      spiTxRx(spihandle, &tx_buffer, &rx_buffer);
      duplicate_count++;
    }
    datalog[x-1] = rx_buffer;
  }
  tx_buffer = 0xFF;								// Send as NO-OP
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);
  if (rx_buffer == sample_template[stop-1])		// Check if we got back a copy of what we sent
  {
    fprintf(stderr, "Get_sample: Duplicate answer from register: %02X\n", rx_buffer);
    fprintf(stderr, "Get_sample: X: %u\n", x);
    fprintf(stderr, "Get_sample: Start: %u\n", start);
    fprintf(stderr, "Get_sample: Stop: %u\n", stop);
    tx_buffer = sample_template[stop-1];
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    usleep(1);									// Short delay to allow AVR to catch up
    tx_buffer = 0xFF;
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    duplicate_count++;
  }
  datalog[stop-1] = rx_buffer;
  return 1;
}

void myInterrupt(void)
{
  tick_2 = clock();		// Get current tick
  tick_3 = tick_2 - tick_1;	// Calculate how many ticks have elapsed since last time

  // Check if we somehow have called this procedure already...
  if (int_active == 1)
  {
    fprintf(stderr, "ISR: Interrupt called when active!!\n");
    return;
  }

  // Check if the signal pin is high, if not something is wrong...
  if (digitalRead(INT_PIN) == 0)
  {
    if (tick_3 < INT_TICK_1)
      not_active_count_1++;
    if (tick_3 >= INT_TICK_1 && tick_3 < INT_TICK_2)
      not_active_count_2++;
    if (tick_3 >= INT_TICK_2 && tick_3 < INT_TICK_3)
      not_active_count_3++;
    if (tick_3 >= INT_TICK_3 && tick_3 < INT_TICK_4)
      not_active_count_4++;
    if (tick_3 >= INT_TICK_4)
    {
      not_active_count_5++;
      fprintf(stderr, "ISR: Interrupt pin is not active!   Tick_3: %ld   Count: %u\n", tick_3, not_active_count_5);
    }
    return;
  }

  // Set watchdog flag to indicate we're handling this IRQ
  int_active = 1;

  // Load SPI buffer with SAMPLE_DUMP command
  tx_buffer = 0xF0;
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Send first command and recieve whatever is in the buffer

  if (rx_buffer != 0xAD && rx_buffer != 0xAF && rx_buffer != 0x01 && rx_buffer != 0xFF)
    fprintf(stderr, "ISR: Throwaway value: %02X\n", rx_buffer);

  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xFE;								// Load with 0xFE to command next reply to be how many blocks to expect, if any
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command

  // Reset watchdog flags for SPI connection alive
  conn_alive = 1;
  error_count = 0;

  // Set time vars
  time_1 = time(NULL);		// Get current time
  diff_i = time_1 - time_2;	// Time since we last were here, i.e. between interrupts
  diff_s = time_1 - time_3;	// Time since we started a new 70 sec waiting period
  time_2 = time_1;			// Set time to now

  // Debug code
  if (dbg)
  {
    fprintf(stderr, "ISR: Time since last = %2u\n", diff_i);
    fprintf(stderr, "ISR: Time since sleep= %2u\n", diff_s);
  }

  // Check if SAMPLE_DUMP command was acknowledged, TODO: Rewrite routine to catch errors...
  if (rx_buffer != 0xF0)
  {
    fprintf(stderr, "ISR: SAMPLE_DUMP not ack'd: %02X\n", rx_buffer);

    usleep(1);					// Small delay to ensure AVR acknowledges the command
    tx_buffer = 0xFF;
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Send first command and receive whatever is in the buffer

    if (rx_buffer != 0xFE)
      fprintf(stderr, "ISR: Expect FE: %02X\n", rx_buffer);

    usleep(1);					// Small delay to ensure AVR acknowledges the command
    tx_buffer = 0xFE;							// Load with 0xFE to command next reply to be how many blocks to expect, if any
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command

    if (rx_buffer != 0xFF)
    {
      fprintf(stderr, "ISR: Expect FF: %02X\n", rx_buffer);
      fprintf(stderr, "ISR: Sample collection returned error: %02X\n", rx_buffer);
      int_active = 0;

      usleep(1);					// Small delay to ensure AVR acknowledges the command
      tx_buffer = 0xF0;
      spiTxRx(spihandle, &tx_buffer, &rx_buffer);
      return;
    }
  }

  // Every C prog must have an x counter... and a y variable
  uint8_t x,y;

  // Check how many blocks to expect from AVR
  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xFF;				// Send as NO-OP / PING
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);

  int sample_sent = rx_buffer;

  // Execute right amount of loops to receive correct number of blocks from AVR
  if (sample_sent & 1)			// SYSTIME block (Area 2)
  {
    sample_sent |= 130;			// If SYSTIME has changed, make sure we get a CURRENT and ONEWIRE sample also
    if(!get_sample(settings, systime))
      return;
  }
  if (sample_sent & 2)			// CURRENT block (Area 4)
    if(!get_sample(historical, current))
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
  if (sample_sent & 128)		// ONEWIRE block (Area 8)
    if(!get_sample(status, temperature))
      return;

  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xF0;				// Send 0xF0 to command an end to SAMPLE_DUMP
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);

  // Debug code
  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xA1;				// Load with A1 to request test2 which contains TWI error code
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command
  test4 = rx_buffer;
  tot_samples = tot_samples + test4;
  if (sample_sent != 128)			// Calculate time only between TWI samples
  {
    if (test4 > 0)
      if (secs > 0)
        secs = (secs + (float)diff_i/test4)/2;	// Calculate how many seconds in average between samples
      else
      {
        secs = (float)diff_i/test4;		// Set a baseline value for further calculations
        fprintf(stderr, "ISR: Secs= %.2f\n", secs);
      }
  }
  if (dbg)
    fprintf(stderr, "ISR: Seconds between= %.2f\n", (float)diff_i/test4);

  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xFF;				// Load with FF to PING
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command
  if (rx_buffer == 0xA1)
  {
    usleep(1);					// Short delay to allow AVR to catch up
    tx_buffer = 0xA1;
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    usleep(1);					// Short delay to allow AVR to catch up
    tx_buffer = 0xFF;				// Load with FF to PING
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);	// Buffer should contain answer to previous send command
  }
  if (rx_buffer > 0 && rx_buffer != 64)		// Buffer contains test2, if test2 > 0, then an error has occured
  {
    fprintf(stderr, "ISR: test2 check= %2u\n", rx_buffer);
    debug_msg();
  }
  if (rx_buffer & 64)
  {
    usleep(1);					// Small delay to ensure AVR acknowledges the command
    tx_buffer = 0xA0;				// Load with A0 to request test1 (# of TWI bus error due to illegal START or STOP condition)
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    usleep(1);					// Short delay to allow AVR to catch up
    tx_buffer = 0xFF;
    spiTxRx(spihandle, &tx_buffer, &rx_buffer);
    if (tot_samples > 0)
    {
      tot_error = tot_error + rx_buffer;
      err_ratio = (float)tot_error / (float)tot_samples * 100.0;
      if (err_ratio > ERROR_RATIO)
        fprintf(stderr, "ISR: Err_ratio= %.4f%\ttot_error= %u\ttot_samples= %u\n", err_ratio, tot_error, tot_samples);
    }
  }

  usleep(1);					// Small delay to ensure AVR acknowledges the command
  tx_buffer = 0xAF;				// Send 0xAF to reset the debug vars
  spiTxRx(spihandle, &tx_buffer, &rx_buffer);

  // If needed, write downloaded array to files complete with SQL
  FILE *fp;

  // String to use for building MySQL query, remember to check max size when changing anything
  char sql_string[1023];
  char temp_string[491];
  char tmp[13];

  // SYSTIME table
  sprintf(sql_string, "INSERT INTO TIME VALUES(NULL,NULL,'%02u:%02u',%u,%u);", datalog[settings], datalog[settings+1], datalog[settings+2], datalog[settings+3]);
  // Max string length = (41+12) = 53

  // CURRENT table
  if (sample_sent & 2)
  {
    sprintf(temp_string, "INSERT INTO CURRENT VALUES(LAST_INSERT_ID(),%d,%u.%u,%.1f,%u,%u,%u,%u,%d,%d,%u);",datalog[historical]-40,datalog[historical+1],datalog[historical+2],(float)datalog[historical+3]/2,datalog[historical+4],datalog[historical+5],datalog[historical+6],datalog[historical+7],datalog[historical+8]-40,datalog[historical+9]-40,datalog[historical+10]);
    strcat(sql_string, temp_string);	// Max temp_string length = (56+35) = 91
    if (dbg)
    {
      fp = fopen("/tmp/current.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // HISTORICAL table
  if (sample_sent & 4)
  {
    sprintf(temp_string, "DELETE FROM HISTORICAL;INSERT INTO HISTORICAL VALUES(LAST_INSERT_ID(),%02u%02u%02u,%u,%02u%02u%02u,%u,%u,%u,%u,%u);",datalog[systime],datalog[systime+1],datalog[systime+2],datalog[systime+3],datalog[systime+4],datalog[systime+5],datalog[systime+6],datalog[systime+7],datalog[systime+8],datalog[systime+9],datalog[systime+10],datalog[systime+11]);
    strcat(sql_string, temp_string);	// Max temp_string length = (79+36) = 115
    if (dbg)
    {
      fp = fopen("/tmp/historical.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // SETTINGS table
  if (sample_sent & 8)
  {
    sprintf(temp_string, "DELETE FROM SETTINGS;INSERT INTO SETTINGS VALUES(LAST_INSERT_ID(),%.1f,", (float)datalog[0]/2);
    for (x = 1 ; x < settings-1 ; x++)
    {
      y = sample_template[x];
      if (y ==0x06 || y ==0x07 || y == 0x13 || y == 0x15 || y == 0x1E || y == 0x1F || y == 0x3A || y == 0x41)
      {
        sprintf(tmp, "%d,", datalog[x]-40);
        strcat(temp_string, tmp);
      }
      else
	  if (y == 0x31)
	  {
	    sprintf(tmp, "%u0,%u0,%u0,", datalog[x], datalog[x+1], datalog[x+2]);
        strcat(temp_string, tmp);
	    x = x+2;
	  }
	  else
      if (y == 0x16)
	  {
        sprintf(tmp, "%u%u%u,", datalog[x], datalog[x+1], datalog[x+2]);
        strcat(temp_string, tmp);
 	    x = x+2;
	  }
      else
      {
        sprintf(tmp, "%u,", datalog[x]);
        strcat(temp_string, tmp);
      }
    }
    sprintf(tmp, "%u);", datalog[x]);
    strcat(temp_string, tmp);
    strcat(sql_string, temp_string);	// String length = (66+xx) = 490
    if (dbg)
    {
      fp = fopen("/tmp/settings.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // ALARMS table
  if (sample_sent & 16)
  {
    sprintf(temp_string, "INSERT INTO ALARMS VALUES(LAST_INSERT_ID(),%u,%u,%u,%u);",datalog[current],datalog[current+1],datalog[current+2],datalog[current+3]); 
    strcat(sql_string, temp_string);	// Max temp_string length = (48+12) = 60
    if (dbg)
    {
      fp = fopen("/tmp/alarms.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // LAST_24H table
  if (sample_sent & 32)
  {
    sprintf(temp_string, "INSERT INTO LAST_24H VALUES(LAST_INSERT_ID(),'%02u:%02u',%u);", datalog[alarms], datalog[alarms+1], datalog[alarms+2]);
    strcat(sql_string, temp_string);	// Max temp_string length = (51+9) = 60
    if (dbg)
    {
      fp = fopen("/tmp/last_24h.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // STATUS table
  if (sample_sent & 64)
  {
    sprintf(temp_string, "INSERT INTO STATUS VALUES(LAST_INSERT_ID(),%u,%u,%u,%u);", datalog[last24h],datalog[last24h+1],datalog[last24h+2],datalog[last24h+3]);
    strcat(sql_string, temp_string);	// Max temp_string length = (48+12) = 60
    if (dbg)
    {
      fp = fopen("/tmp/status.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  // ONEWIRE table
  if (sample_sent & 128)
  {
    sprintf(temp_string, "INSERT INTO ONEWIRE VALUES(LAST_INSERT_ID(),%u.%02u,%u.%02u,%u.%02u,%u.%02u,%u.%02u,%u.%02u);",datalog[status],datalog[status+1],datalog[status+2],datalog[status+3],datalog[status+4],datalog[status+5],datalog[status+6],datalog[status+7],datalog[status+8],datalog[status+9],datalog[status+10],datalog[status+11]); 
    strcat(sql_string, temp_string);	// Max temp_string length = (57+36) = 93
    if (dbg)
    {
      fp = fopen("/tmp/onewire.sql", "w");
      fprintf(fp, temp_string);
      fclose(fp);
    }
  }

  if (dbg)
  {
    fp = fopen("/tmp/total.sql", "w");
    fprintf(fp, sql_string);
    fclose(fp);
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

  tick_1 = clock();
}
