# STM32 Based Simple Password Door Lock System Prototype



## System Architecture

Within the scope of the project realized on the STM32F303RE Nucleo-64 development board, 3x4 keypad module, a 2x16 LCD screen and 2 LEDs, one green and one red, are used.

All required outputs are shown on the LCD screen. Password and selection entries are made on the keypad. On the other hand, LEDs provide status information. The green LED blinks when a favorable output is received, and the red LED blinks when an unfavorable output is received.

When the system is started, it has a preset password and the door is locked. To unlock or to define a new password, the correct password must be entered in the password. After unlocking, the system offers two different operation choices: defining a new password or locking the door.

## Connection

Circuit Diagram will be added as soon as possible, but it is possible to understand connection by review the PasswordDoorLockSystem.ioc or MX_GPIO_Init function located in main.c.

In order to avoid bouncing, using resistors (100KΩ for rows and 10KΩ for columns) is very important.

## Software

System operations are controlled by the value held by the variable named door_state. If the value of the variable is 0, the system is locked and a password must be entered. If the password is entered correctly, the value of the variable changes to 1. For this operation, the values ​​returned by the keypad_scanner function are compared with the password defined in turn. According to the comparison result, the green or red LED lights up for a while while the information message is printed on the LCD.

In case of unlocking, two different options are offered to the user. The user can change the password by pressing the asterisk (*) key or lock the system again by pressing the pound (#) key. A password is required to lock the system.

The user has to press the pound (#) key to confirm the password he has entered. In addition, the user can clear or cancel the password screen with the asterisk (*) key.

## Note

For now, the system does not store the password defined by user permanently, thus it waits for the predefined password every time the system is restarted. The project will be updated as soon as the necessary adjustments will be made.

This project was implemented as a prototype, so it does not have the equipment to control a lock. I am thinking of adding the necessary equipment later.
