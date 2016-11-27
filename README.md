# raspilogger
A program for interfacing with an Atmel AVR using SPI to transfer data from a CTC EcoLogic EXT system.

Designed to be used with avrlogger https://github.com/flamewalker/avrlogger


Commands can be entered through named pipe: ctc_cmd

Available commands:
-------------------

A0        : Show corresponding debug variable
 |  
AF

B0        : Show I2C debug message

B1        : Show stats

B2        : Toggle DEBUG mode

B3        : Start DHW heating cycle

DC xx     : Set room_temp to xx/2

DD 14     : Reset all alarms

DE xx     : Set number of SMS to xx

F2 xx     : Set hot water temp to xx (25-99)

F3 xx yy  : Send commmand xx and argument yy to digipot
