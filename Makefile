all:
	gcc spi_logger.c -o spi_logger -I/usr/include/mysql -DBIG_JOINS=1  -fno-strict-aliasing   -g -L/usr/lib/arm-linux-gnueabihf -lmysqlclient -lpthread -lz -lm -lrt -ldl -lwiringPi

stable:
	gcc spi_logger_stable.c -o spi_logger -I/usr/include/mysql -DBIG_JOINS=1  -fno-strict-aliasing   -g -L/usr/lib/arm-linux-gnueabihf -lmysqlclient -lpthread -lz -lm -lrt -ldl -lwiringPi

test:
	gcc spi_logger_test.c -o spi_logger -I/usr/include/mysql -DBIG_JOINS=1  -fno-strict-aliasing   -g -L/usr/lib/arm-linux-gnueabihf -lmysqlclient -lpthread -lz -lm -lrt -ldl -lwiringPi
