# STM32 Based Simple Password Door Lock System Prototype

## System Architecture

Within the scope of the project realized on the STM32F303RE Nucleo-64 development board, 3x4 keypad module, a 2x16 LCD screen and 2 LEDs, one green and one red, are used.

All required outputs are shown on the LCD screen. Password and selection entries are made on the keypad. On the other hand, LEDs provide status information. The green LED blinks when a favorable output is received, and the red LED blinks when an unfavorable output is received.

When the system is started for the first time, the user is asked to define a password for once only. This password remains the same as long as it is not changed by the user. After the user has defined his password, the lock is unlocked and the user is expected to choose to change his password or lock the door.

## Connection

Circuit Diagram will be added as soon as possible, but it is possible to understand connection by review the PasswordDoorLockSystem.ioc or MX_GPIO_Init function located in main.c.

In order to avoid bouncing, using resistors (100KΩ for rows and 10KΩ for columns) is very important.

## Software

First, it is checked whether the system is started for the first time or not, and if it is started for the first time, the user is asked to define a password. If the system is powered up for the first time, the lock is unlocked after password identification and the system enters an endless loop with the main system processes.

This password identification process, which is performed at the beginning, is only for once, and the password and lock status defined after the user has defined a password never changes unless the user changes it.

System operations are controlled by the values found in the first 5 bytes of the FLASH memory on page 127. The first 4 bytes of this section contain the password specified by the user, and the remaining 1 byte contains information on the lock status. The reason for this information to be found in FLASH memory is that when the system is restarted for any reason, it can work again without losing the state it was in.

## Note

This project is implemented as a prototype, so it does not have the equipment to control a lock. I am thinking of adding the necessary equipment for door lock later.
