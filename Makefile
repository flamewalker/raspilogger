all:
	gcc -c 'mysql_config --cflags' spi_logger.c
	gcc -o spi_logger spi_logger.o 'mysql_config --libs' -lwiringPi
