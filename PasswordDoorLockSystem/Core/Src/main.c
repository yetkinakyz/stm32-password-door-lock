/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Memory address that contains password & door lock state data
#define memory_address 0x0801FC00 //STM32303RE - Page 127

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

//8µs Delay Function
void delay_us ();

//LCD Functions
void InitLCD(void);
void lcd_init_write(unsigned char value);
void epulse(void);
void lcd_cmd(unsigned char cmd);
void lcd_write_data (char c);
void lcd_data(unsigned char row, unsigned char col, const char *data);

char keypad_scanner(void);

//Memory Operation Functions
void Flash_Erase(uint32_t address);
void Flash_Write(uint32_t address, uint16_t data);
uint16_t Flash_Read(uint32_t address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* Variables */

  int door_state;
  int counter = 0;
  int pass_check = 0;
  int operation_check = 0;

  char key;
  char password [4];
  char password_temp[4];
  char* key_temp;

  uint16_t memory_data;

  InitLCD(); //LCD Initialization

  /* Starting Messages */

  lcd_data(1, 1, " EEM418 ");
  lcd_data(2, 1, " FINAL PROJECT ");

  HAL_Delay(1000);

  lcd_data(1, 1, " PASSWORD  DOOR ");
  lcd_data(2, 1, " LOCK SYSTEM ");

  HAL_Delay(1000);

  lcd_data(1, 1, "  YETKIN AKYUZ  ");
  lcd_data(2, 1, " BEYZANUR CELIK ");

  HAL_Delay(1000);

  lcd_data(1, 1, "                ");
  lcd_data(2, 1, "                ");

  HAL_Delay(500);

  /* First use check */

  //Get data from 0x0801FC08. This address contains door lock state data.
  memory_data = Flash_Read(memory_address + 8);
  door_state = memory_data;

  //If the memory address is empty (0xFFFF), the system is using for first time.
  //Defining a password is required.
  //This process works only once.
  if (door_state == 0xFFFF)
  {
	  while (1)
	  {
		  lcd_data(1, 1, "DEFINE YOUR ");
		  lcd_data(2, 1, "PASSWORD: ");

		  counter = 0;

		  while(1)
		  {
			  key = keypad_scanner(); //Get char from keypad.

			  //Write keypad inputs to password_temp variable until there are 4 character inputs from keypad.
			  if (counter <= 3 && key != 'n' && key != '#' && key != '*')
			  {
				  key_temp = key;
				  password_temp[counter] = key;

				  lcd_data(2, 11 + counter, &key_temp);
				  HAL_Delay(200);
				  lcd_data(2, 11 + counter, "*");

				  counter++;
			  }

			  //If input length is 4 and pressed the # button
			  else if (counter > 3 && key == '#')
			  {
				  counter = 0;

				  //Erase memory
				  Flash_Erase(memory_address);

				  //Write defined password to memory
				  for (int i = 0; i < 4; i++)
				  {
					  memory_data = (uint16_t *)password_temp[i];

					  Flash_Write(memory_address + i * 2, memory_data);
					  memory_data = Flash_Read(memory_address + i * 2);
					  password [i] = memory_data;
				  }

				  //Write door lock state to memory: (10) Door is open, (20) Door is locked.
				  Flash_Write(memory_address + 8, 10);

				  /* Done! */

				  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 1);

				  lcd_data(1, 1, " DONE ");
				  lcd_data(2, 1, "PASSWORD: ");

				  //Print defined password, wait a second, print **** instead of the password.
				  for (int i = 0; i < 4; i++)
				  {
					  key_temp = password[i];
					  lcd_data(2, 11 + i, &key_temp);
				  }

				  HAL_Delay(1000);

				  for (int i = 0; i < 4; i++)
				  {
					  lcd_data(2, 11 + i, "*");
				  }

				  HAL_Delay(200);

				  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 0);

				  break;
			  }
		  }

		  break;
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_data(1, 1, "                ");
	  lcd_data(2, 1, "                ");

	  /* If door is locked: (10) Door is open, (20) Door is locked. */

	  while (Flash_Read(memory_address + 8) == 20)
	  {
		  lcd_data(1, 1, "STATUS: LOCKED ");
		  lcd_data(2, 1, "PRESS ANY BUTTON");

		  //Wait till when any key on keypad is pressed.
		  while (keypad_scanner() == 'n');

		  //Till when door is opened.
		  while(Flash_Read(memory_address + 8) == 20)
		  {
			  lcd_data(1, 1, "STATUS: LOCKED ");
			  lcd_data(2, 1, "PASSWORD: ");
			  lcd_data(2, 10, " ");

			  while(1)
			  {
				  key = keypad_scanner(); //Get char from keypad.

				  if (counter <= 3 && key != '*' && key != '#' && key != 'n')
				  {
					  key_temp = key;

					  lcd_data(2, 11 + counter, &key_temp);

					  HAL_Delay(200);

					  lcd_data(2, 11 + counter, "*");

					  password_temp[counter] = key;

					  counter++;
				  }

				  //Write keypad inputs to password_temp variable until there are 4 character inputs from keypad.
				  else if (counter > 3 && key == '#')
				  {
					  counter = 0;
					  pass_check = 0;

					  //In turn, compare each digit with the password, if the numbers match, increase pass_check by one.
					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i*2);
						  password[i] = memory_data;

						  if (password[i] == password_temp[i])
						  {
							  pass_check++;
						  }
					  }

					  //Save password data from memory to password_temp due to avoid data loss
					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i*2);
						  password_temp[i] = memory_data;
					  }

					  //Erase memory
					  Flash_Erase(memory_address);

					  //Write password data from password_temp to memory
					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = (uint16_t *)password_temp[i];
						  Flash_Write(memory_address + i*2, memory_data);
					  }

					  //If the passwords match
					  if (pass_check > 3)
					  {
						  //Write door is open info to memory
						  Flash_Write(memory_address + 8, 10);

						  /* Done! */

						  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 1);

						  lcd_data(1, 1, " PASSWORD MATCH ");
						  lcd_data(2, 1, " OPENING DOOR ");

						  HAL_Delay(1500);

						  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 0);

						  break;
					  }

					  //If the passwords do not match
					  else
					  {
						  Flash_Write(memory_address + 8, 20);

						  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, GPIO_PIN_SET);

						  lcd_data(1, 1, " WRONG PASSWORD ");
						  lcd_data(2, 1, " ACCESS DENIED ");

						  HAL_Delay(1000);

						  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, GPIO_PIN_RESET);

						  break;
					  }
				  }

				  //If * key is pressed, clean the password field.
				  else if (key == '*')
				  {
					  counter = 0;

					  break;
				  }
			  }
	  	  }
	  }

	  /* If door is open: (10) Door is open, (20) Door is locked. */

	  while (Flash_Read(memory_address + 8) == 10)
	  {
		  key = keypad_scanner(); //Get char from keypad.

		  //If # button is pressed, Lock the door selection.
		  if (key == '#')
		  {
			  counter = 0;
			  operation_check = 0;

			  lcd_data(1, 1, "               ");
			  lcd_data(2, 1, "               ");

			  while(!operation_check)
			  {
				  lcd_data(1, 1, "LOCK THE DOOR   ");
				  lcd_data(2, 1, "PASSWORD: ");

				  /* Password Check */

				  if (counter <= 3 && key != '*' && key != '#' && key != 'n')
				  {
					  key_temp = key;
					  password_temp[counter] = key;

					  lcd_data(2, 11 + counter, &key_temp);

					  HAL_Delay(200);

					  lcd_data(2, 11 + counter, "*");

					  password_temp[counter] = key;

					  counter++;
				  }

				  else if (counter > 3 && key == '#')
				  {
					  counter = 0;
					  pass_check = 0;

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i*2);
						  password[i] = memory_data;

						  if (password[i] == password_temp[i])
						  {
							  pass_check++;
						  }
					  }

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i*2);
						  password_temp[i] = memory_data;
					  }

					  Flash_Erase(memory_address);

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = (uint16_t *)password_temp[i];
						  Flash_Write(memory_address + i*2, memory_data);
					  }

					  if (pass_check > 3)
					  {
						  Flash_Write(memory_address + 8, 20);

						  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 1);

						  lcd_data(1, 1, " PASSWORD MATCH ");
						  lcd_data(2, 1, "  LOCKING DOOR  ");

						  HAL_Delay(1500);

						  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 0);

						  operation_check = 1;
					  }

					  else
					  {
						  Flash_Write(memory_address + 8, 10);

						  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, 1);

						  lcd_data(1, 1, " WRONG PASSWORD ");
						  lcd_data(2, 1, "                ");

						  HAL_Delay(1000);

						  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, 0);
					  }
				  }

				  //If * button is pressed, clean password filed, or cancel if password files is empty.
				  else if (key == '*')
				  {
					  if (counter == 0)
					  {
						  lcd_data(1, 1, "OPERATION       ");
						  lcd_data(2, 1, "CANCELLED       ");

						  HAL_Delay(1000);

						  operation_check = 1;
					  }

					  else
					  {
						  counter = 0;

						  lcd_data(2, 1, "PASSWORD:       ");
						  lcd_data(2, 10, " ");
					  }
				  }
			  }
		  }

		  //If * button is pressed, change password selection.
		  else if (key == '*')
		  {
			  counter = 0;
			  operation_check = 0;

			  lcd_data(1, 1, "                ");
			  lcd_data(2, 1, "                ");

			  while(!operation_check)
			  {
				  /* Password Check */

				  lcd_data(1, 1, "OLD             ");
				  lcd_data(2, 1, "PASSWORD:       ");
				  lcd_data(2, 10, " ");

				  key = keypad_scanner();

				  if (counter <= 3 && key != '*' && key != '#' && key != 'n')
				  {
					  key_temp = key;
					  password_temp[counter] = key;

					  lcd_data(2, 11 + counter, &key_temp);
					  HAL_Delay(200);
					  lcd_data(2, 11 + counter, "*");

					  counter++;
				  }

				  else if (counter > 3 && key == '#')
				  {
					  counter = 0;
					  pass_check = 0;

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i * 2);
						  password[i] = memory_data;

						  if (password[i] == password_temp[i])
						  {
							  pass_check++;
						  }
					  }

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = Flash_Read(memory_address + i * 2);
						  password_temp[i] = memory_data;
					  }

					  Flash_Erase(memory_address);

					  for (int i = 0; i < 4; i++)
					  {
						  memory_data = (uint16_t *)password_temp[i];
						  Flash_Write(memory_address + i * 2, memory_data);
					  }

					  Flash_Write(memory_address + 8, 10);

					  if (pass_check > 3)
					  {
						  counter = 0;

						  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, 1);

						  lcd_data(1, 1, "               ");
						  lcd_data(2, 1, "               ");

						  HAL_Delay(200);

						  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, 0);

						  /* New password */

						  while (1)
						  {
							  lcd_data(1, 1, "NEW             ");
							  lcd_data(2, 1, "PASSWORD: ");

							  key = keypad_scanner();

							  if (counter<=3 && key!='n' && key!='#' && key!='*')
							  {
								  key_temp = key;
								  password_temp[counter] = key;

								  lcd_data(2, 11 + counter, &key_temp);
								  HAL_Delay(200);
								  lcd_data(2, 11 + counter, "*");

								  counter++;
							  }

							  else if (counter > 3 && key == '#')
							  {
								  counter = 0;

								  //Erase memory
								  Flash_Erase(memory_address);

								  //Write new password to memory
								  for (int i = 0; i < 4; i++)
								  {
									  memory_data = (uint16_t *)password_temp[i];
									  Flash_Write(memory_address+i*2,memory_data);

									  memory_data = Flash_Read(memory_address+i*2);
									  password [i] = memory_data;
								  }

								  //Write door is open info to memory
								  Flash_Write(memory_address + 8, 10);

								  /* Done! */

								  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 1);

								  lcd_data(1, 1, "      DONE      ");
								  lcd_data(2, 1, "PASSWORD:       ");

								  for (int i = 0; i < 4; i++)
								  {
									  key_temp = password[i];

									  lcd_data(2, 11 + i, &key_temp);
								  }

								  HAL_Delay(1000);

								  for (int i = 0; i < 4; i++)
								  {
									  lcd_data(2, 11 + i, "*");
								  }

								  HAL_Delay(200);

								  HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin, 0);

								  operation_check = 1;
								  break;
							  }

							  //If * button is pressed, clean password field or cancel
							  else if (key == '*')
							  {
								  if (counter == 0)
								  {
									  lcd_data(1, 1, "OPERATION       ");
									  lcd_data(2, 1, "CANCELLED       ");

									  HAL_Delay(1000);

									  operation_check = 1;
									  break;
								  }

								  else
								  {
									  counter = 0;

									  lcd_data(2, 1, "PASSWORD:       ");
									  lcd_data(2, 10, " ");
								  }
							  }
						  }
					  }
				  }

				  else
				  {
					  counter = 0;

					  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, 1);

					  lcd_data(1, 1, " WRONG PASSWORD ");
					  lcd_data(2, 1, "                ");

					  HAL_Delay(1000);

					  HAL_GPIO_WritePin(GPIOA, LD_RED_Pin, 0);
				  }
			  }
		  }
		  //If * button is pressed, clean password field or cancel
		  else if (key == '*')
		  {
			  if (counter == 0)
			  {
				  lcd_data(1, 1, "OPERATION       ");
				  lcd_data(2, 1, "CANCELLED       ");

				  HAL_Delay(1000);

				  operation_check = 1;
				  break;
			  }

			  else
			  {
				  counter = 0;

				  lcd_data(2, 1, "PASSWORD:       ");
				  lcd_data(2, 10, " ");
			  }
		  }

		  //Door is open & selection message
		  else
		  {
			  lcd_data(1, 1, "STATUS: OPEN    ");
			  lcd_data(2, 1, "PRESS * OR #    ");
		  }
	  }
    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LD_GREEN_Pin|LD_RED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R2_Pin|R1_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LD_GREEN_Pin LD_RED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD_GREEN_Pin|LD_RED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : C3_Pin */
  GPIO_InitStruct.Pin = C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R1_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R1_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void delay_us ()
{
	unsigned int time=680;
	while (time--);
}

void InitLCD (void)
{
  HAL_Delay (200);
  lcd_init_write(0x30);
  HAL_Delay (10);
  lcd_init_write(0x30);
  HAL_Delay (10);
  lcd_init_write(0x30);
  HAL_Delay (10);
  lcd_init_write(0x20);
  HAL_Delay (10);

  lcd_cmd(0x28);
  lcd_cmd(0x08);
  lcd_cmd(0x01);
  lcd_cmd(0x06);
  lcd_cmd(0x0F);
}

void lcd_init_write(unsigned char value)
{

	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);
	GPIOB->ODR= (GPIOB->ODR & 0x0F) | (value & 0xF0) ;
	epulse();
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);

}

void epulse()
{
	HAL_GPIO_WritePin (LCD_E_GPIO_Port, LCD_E_Pin,1);
	delay_us();
	HAL_GPIO_WritePin (LCD_E_GPIO_Port, LCD_E_Pin,0);
	delay_us();
}

void lcd_cmd(unsigned char cmd)
{
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);
	GPIOB->ODR= (GPIOB->ODR & 0x0F) | (cmd & 0xF0);
	epulse();
	GPIOB->ODR = (GPIOB->ODR & 0x0f) | (cmd<<4);
	epulse();
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);

	HAL_Delay(0.0001);
}
void lcd_write_data (char c)
{
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);
	GPIOB->ODR = (GPIOB->ODR & 0x0f) | (c & 0xf0);
	epulse();
	GPIOB->ODR = (GPIOB->ODR & 0x0f) | (c << 4);
	epulse();
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);
}

void lcd_data(unsigned char row, unsigned char col, const char *data)
{
	char temp;

	switch(row)
	{
		case 1:
			temp = col - 1 + 0x80;
			break;
		case 2:
			temp = col - 1 + 0xC0;
			break;
	}

	lcd_cmd(temp);

	do
	{
		lcd_write_data(*data++);

	}while(*data);
}

char keypad_scanner(void)
{
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);

		return '1';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);

		return '2';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);

		return '3';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);

		return '4';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);

		return '5';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);

		return '6';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);

		return '7';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);

		return '8';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);

		return '9';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0);

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);

		return '*';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);

		return '0';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);

		return '#';
	}

	return 'n';
}

void Flash_Erase(uint32_t address)
{
	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
	FLASH->CR|=0x00000002;
	FLASH->AR=address;
	FLASH->CR|=0x00000040;
	while((FLASH->SR&0x00000001));
	FLASH->CR &= ~0x00000042;
	FLASH->CR=0x00000080;
}

void Flash_Write(uint32_t address, uint16_t data)
{
	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;
	FLASH->CR|=0x00000001;
	*(__IO uint16_t*)address = data;
	while((FLASH->SR&0x00000001));
	FLASH->CR=0x00000080;
}

uint16_t Flash_Read(uint32_t address)
{
	uint16_t * temp = (uint16_t *)address;
	return(*temp);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
