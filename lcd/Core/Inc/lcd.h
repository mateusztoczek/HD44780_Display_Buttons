#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>


#define LCD_8BIT_MODE	// Sets data transfer mode, delete/comment this line to change into 4-bit mode.

#ifdef LCD_8BIT_MODE
#define D0_Pin GPIO_PIN_0
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_2
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_10
#define D3_GPIO_Port GPIOA
#endif

#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOB

#define RS_Pin GPIO_PIN_1
#define RS_GPIO_Port GPIOC
#define RW_Pin GPIO_PIN_2
#define RW_GPIO_Port GPIOC
#define E_Pin GPIO_PIN_3
#define E_GPIO_Port GPIOC

#define LCD_Y 						2
#define LCD_X 						16

#define LCD_LINE1 					0x00
#define LCD_LINE2 					0x40
#define LCD_CLR						0x01
#define LCD_HOME					0x02
#define LCD_MOVE					0x01
#define LCD_ENTRY					0x04
#define LCD_ENTRYRIGHT				0x02
#define LCD_ENTRYLEFT				0x00
#define LCD_ONOFF					0x08
#define LCD_DISPLAYON				0x04
#define LCD_CURSORON				0x02
#define LCD_CURSOROFF				0x00
#define LCD_BLINKON					0x01
#define LCDC_BLINKOFF				0x00
#define LCD_SHIFT					0x10
#define LCD_SHIFTDISP				0x08
#define LCD_SHIFTRIGHT				0x04
#define LCD_SHIFTLEFT				0x00
#define LCD_FUNC					0x20
#define LCD_FUNC8BIT				0x10
#define LCD_FUNC4BIT				0x00
#define LCD_FUNC2LINE				0x08
#define LCD_FUNC1LINE				0x00
#define LCD_FUNC5x10				0x04
#define LCD_FUNC5x7					0x00
#define LCD_SET_CGRAM				0x40
#define LCD_SET_DDRAM				0x80

#define LCD_E_SET 		HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET)
#define LCD_E_RESET 	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET)
#define LCD_RS_SET 		HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET)
#define LCD_RS_RESET 	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET)
#define LCD_RW_SET 		HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_SET)
#define LCD_RW_RESET 	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET)

// Function Prototypes with comments

void LCD_Home(void); // Return the cursor to the home position.
void LCD_Clear(void); // Clear the LCD display.
void LCD_Init(void); // Initialize the LCD.
void LCD_SetLocation(uint8_t x, uint8_t y); // Set the cursor location.
void LCD_Blink(uint8_t on_off); // Turn blinking of the cursor on/off.
void LCD_Cursor(uint8_t on_off); // Turn the cursor on/off.
void LCD_CustomChar(uint8_t number, const uint8_t *def_char); // Define a custom character.
void LCD_Char(char c); // Display a character on the LCD.
void LCD_String(char* str); // Display a string on the LCD.
void LCD_Int(int value); // Display an integer on the LCD.
void LCD_Hex(int value, uint8_t upper_case); // Display an integer in hexadecimal format.
void LCD_Write(uint8_t data,uint8_t is_command); // Write data to the LCD.
void LCD_InitBuffer(void);	// Set empty lines into buffer.
void LCD_UpdateBuffer(uint8_t line, uint8_t position, char* str); // Write string into buffer.
void LCD_FlushBuffer(void);	// Send buffer to display.
void LCD_Print(char* str, uint8_t line); //Write new string on cleared display.
void LCD_Add(char* str, uint8_t line); // Print without clearing the display.
uint8_t LCD_CheckBusyFlag(void); // Check the LCD's busy flag.
uint8_t LCD_Read(void); // Read data from the LCD.

#endif /* LCD_H_ */
