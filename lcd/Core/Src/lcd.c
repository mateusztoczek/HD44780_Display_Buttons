#include "lcd.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include "gpio.h"
#include "tim.h"


// display buffer
char lcdBuffer[LCD_Y][LCD_X];

// timer used to create delay
TIM_HandleTypeDef *timer = &htim3;	// Default timer set to TIM3. Replace '&htim3' with your own timer instance if necessary.


// Configure data pins based on the provided data byte
static inline void LCD_WritePins(uint8_t data) {
	LCD_E_SET;

    #ifdef LCD_8BIT_MODE
		GPIO_TypeDef* ports[] = {D0_GPIO_Port,D1_GPIO_Port,D2_GPIO_Port,D3_GPIO_Port,D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
    	uint16_t pins[] = {D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin, D5_Pin, D6_Pin, D7_Pin};
	#else
		GPIO_TypeDef* ports[] = {D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
    	uint16_t pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
	#endif

    for (int i = 0; i < sizeof(ports) / sizeof(ports[0]); i++) {
        if (data & (1 << i)) {
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        }
    }

    LCD_E_RESET;
}


// Read the status of data pins and return the resulting byte
static inline uint8_t LCD_ReadPins(void) {
    uint8_t result = 0;
    LCD_E_SET;

	#ifdef LCD_8BIT_MODE
		GPIO_TypeDef* ports[] = {D0_GPIO_Port,D1_GPIO_Port,D2_GPIO_Port,D3_GPIO_Port,D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
    	uint16_t pins[] = {D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin, D5_Pin, D6_Pin, D7_Pin};
	#else
		GPIO_TypeDef* ports[] = {D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
    	uint16_t pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
	#endif


    for (int i = 0; i < sizeof(ports) / sizeof(ports[0]); i++) {
        if (HAL_GPIO_ReadPin(ports[i], pins[i]) == GPIO_PIN_SET) {
            result |= (1 << i);
        }
    }

    LCD_E_RESET;
    return result;
}


// Set display in input mode
static void LCD_SetDataInputMode(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D5_Pin;
	HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D6_Pin;
	HAL_GPIO_Init(D6_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D7_Pin;
	HAL_GPIO_Init(D7_GPIO_Port, &GPIO_InitStruct);

	#ifdef LCD_8BIT_MODE
	 	GPIO_InitStruct.Pin = D0_Pin;
	 	HAL_GPIO_Init(D0_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D1_Pin;
	 	HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D2_Pin;
	 	HAL_GPIO_Init(D2_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D3_Pin;
	 	HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);
	#endif

}


// Set display in output mode
static void LCD_SetDataOutputMode(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D5_Pin;
	HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D6_Pin;
	HAL_GPIO_Init(D6_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D7_Pin;
	HAL_GPIO_Init(D7_GPIO_Port, &GPIO_InitStruct);

	#ifdef LCD_8BIT_MODE
	 	GPIO_InitStruct.Pin = D0_Pin;
	 	HAL_GPIO_Init(D0_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D1_Pin;
	 	HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D2_Pin;
	 	HAL_GPIO_Init(D2_GPIO_Port, &GPIO_InitStruct);

	 	GPIO_InitStruct.Pin = D3_Pin;
	 	HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);
	#endif
}


// Read first 4 bites from LCD, then another 4 bites
uint8_t LCD_Read(void) {
	uint8_t result = 0;
	LCD_SetDataInputMode();

	LCD_RW_SET;
	#ifndef LCD_8BIT_MODE
		result = (LCD_ReadPins() << 4);
		result |= LCD_ReadPins();
	#else
		result = LCD_ReadPins();
	#endif

	return result;
}


// Write first 4 bites to LCD, then another 4 bites, wait until display is ready
void LCD_Write(uint8_t data, uint8_t is_command) {
    if (is_command) {
    	LCD_RS_RESET;
    } else {
    	LCD_RS_SET;
    }

	LCD_RW_RESET;
	LCD_SetDataOutputMode();

	#ifndef LCD_8BIT_MODE
		LCD_WritePins(data >> 4);
	#endif
	LCD_WritePins(data);

	while((LCD_CheckBusyFlag() & (1<<7)));
}



//	Print character on display
void LCD_Char(char c) {
	// If the character is in the range of special characters (0x80 - 0x87),
	// keep only the last 3 bits; otherwise, keep the character unchanged
	LCD_Write((((c >= 0x80) && (c <= 0x87)) ? (c & 0x07) : c), 0);
}


//	Print string on display
void LCD_String(char* str) {
	char c;
	while((c = *(str++)))
		LCD_Char(c);
}


// Print integer on display
void LCD_Int(int value) {
	char buf[LCD_X+1];
	sprintf(buf, "%d", value);
	LCD_String(buf);
}


// Print hexadecimal on display
void LCD_Hex(int value, uint8_t upper_case) {
	char buf[LCD_X+1];
	if(upper_case) {
		sprintf(buf, "%X", value);
	} else {
		sprintf(buf, "%x", value);
	}
	LCD_String(buf);
}


// Print custom char on display
void LCD_CustomChar(uint8_t number, const uint8_t *def_char) {
    LCD_Write((64 + ((number & 0x7) * 8)), 1);

    for (uint8_t i = 0; i < 8; i++) {
        LCD_Write((def_char[i]),0);
    }
}


// Set cursor to home position
void LCD_Home(void) {
	LCD_Write(LCD_CLR|LCD_HOME,1);
}


// Read BF value
uint8_t LCD_CheckBusyFlag(void) {
	LCD_RS_RESET;
	return LCD_Read();
}


// Control cursor visibility
void LCD_Cursor(uint8_t on_off) {
	if(on_off == 0)
		LCD_Write(LCD_ONOFF|LCD_DISPLAYON,1);
	else
		LCD_Write(LCD_ONOFF|LCD_DISPLAYON|LCD_CURSORON,1);
}


// Control cursor blinking
void LCD_Blink(uint8_t on_off) {
	if(on_off == 0)
		LCD_Write(LCD_ONOFF|LCD_DISPLAYON,1);
	else
		LCD_Write(LCD_ONOFF|LCD_DISPLAYON|LCD_CURSOROFF|LCD_BLINKON,1);
}


// Set cursor on display: x-row, y-column
void LCD_SetLocation(uint8_t x, uint8_t y) {
    uint8_t line_offset[] = {LCD_LINE1, LCD_LINE2};

    if (y < sizeof(line_offset) / sizeof(line_offset[0])) {
        uint8_t target_line = line_offset[y];
        LCD_Write((0x80 + target_line + x),1);
    }
}


// Set buffer
void LCD_InitBuffer(void) {
    for (int line = 0; line < LCD_Y; line++) {
        for (int position = 0; position < LCD_X; position++) {
            lcdBuffer[line][position] = ' ';
        }
    }
}


// Update text inside buffer
void LCD_UpdateBuffer(uint8_t line, uint8_t position, char* str) {
    uint8_t i = 0;
    while (str[i] != '\0' && (position + i) < LCD_X) {
        lcdBuffer[line][position + i] = str[i];
        i++;
    }
}

// Send bufffer to LCD
void LCD_FlushBuffer(void) {
    for (uint8_t line = 0; line < LCD_Y; line++) {
        for (uint8_t position = 0; position < LCD_X; position++) {
            LCD_SetLocation(position, line);
            LCD_Char(lcdBuffer[line][position]);
        }
    }
}


// Clear LCD display
void LCD_Clear(void) {
	LCD_Write(LCD_CLR,1);
}


//	Clean display and print data
void LCD_Print(char* str, uint8_t line) {
	LCD_Clear();
	LCD_SetLocation(0,line);
	LCD_UpdateBuffer(line, 0, str);
	LCD_String(str);
}


//	Print without cleaning the display
void LCD_Add(char* str, uint8_t line) {
	LCD_SetLocation(0,line);
	LCD_UpdateBuffer(line, 0, str);
	LCD_String(str);
}


// Delay in us
void LCD_Delay_us(TIM_HandleTypeDef *htim, uint16_t us) {
    htim->Instance->CNT = 0;
    while (htim->Instance->CNT <= us);
}


// Delay in ms
void LCD_Delay_ms(TIM_HandleTypeDef *htim, uint16_t ms) {
    htim->Instance->CNT = 0;
    while (htim->Instance->CNT <= (ms * 1000)) {
    }
}


//	Initialize display
void LCD_Init(void) {

	// reset data pins
	LCD_RS_RESET;
	LCD_E_RESET;
	LCD_RW_RESET;

	//set display into output mode and wait
	LCD_SetDataOutputMode();
	HAL_Delay(15);

	// send initialization data
	LCD_WritePins(0x03);
	LCD_Delay_us(timer, 4100);

	LCD_WritePins(0x03);
	LCD_Delay_us(timer, 100);

	// set transfer mode
	#ifdef LCD_8BIT_MODE
		LCD_WritePins(0x03);
		LCD_Delay_us(timer, 100);
		LCD_Write(LCD_FUNC|LCD_FUNC8BIT|LCD_FUNC2LINE|LCD_FUNC5x7,1);
	#else
		LCD_WritePins(0x02);
		LCD_Delay_us(timer, 100);
		LCD_Write(LCD_FUNC|LCD_FUNC4BIT|LCD_FUNC2LINE|LCD_FUNC5x7,1);
	#endif

	LCD_Write(LCD_ONOFF|LCD_CURSOROFF,1);
	LCD_Write(LCD_ONOFF|LCD_DISPLAYON,1);
	LCD_Write(LCD_ENTRY|LCD_ENTRYRIGHT,1);

	// Clear display
	LCD_Clear();
}
