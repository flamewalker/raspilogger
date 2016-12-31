all: spi_logger

spi_logger: spi_logger.o
	gcc -o spi_logger spi_logger.o `mysql_config --libs` -lwiringPi
	rm ~/spi-logger
	cp spi_logger ~/spi-logger

spi_logger.o: spi_logger.c
	gcc -c `mysql_config --cflags` spi_logger.c

clean:
	rm spi_logger.o spi_logger ctc_cmd
