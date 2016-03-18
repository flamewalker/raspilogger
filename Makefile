all:
	gcc spi_logger.c -o spi_logger -I/usr/include/mysql -DBIG_JOINS=1  -fno-strict-aliasing   -g -L/usr/lib/arm-linux-gnueabihf -lmysqlclient -lpthread -lz -lm -lrt -ldl -lwiringPi
