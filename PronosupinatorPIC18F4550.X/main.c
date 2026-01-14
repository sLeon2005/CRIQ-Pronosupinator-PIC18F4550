/*
 * File: main.c
 * Author: Pan de muerto
 * 
 * Team members:
 *  - Kintia Alexa Negrete Osuna
 *  - Yudy Camila Pérez Cervantes
 *  - Francisco Hernández Ramírez
 *  - Jesús Zamora Castelazo
 *  - Sebastián León Medellín
 *
 * Code for the pronosupinator system. Handles ADC readings, stepper motor control, OLED display,
 * UART/Bluetooth comms, I2C OLED screen, EEPROM storage, and an emergency-stop routine.
 */

#pragma config PLLDIV = 1 // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1 // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3 // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)
// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// using a 16MHz crystal
#define _XTAL_FREQ 16000000

// Pin definitions
#define MotorF RD0 // Forward button
#define MotorB RD1 // Backwards button
#define bt_state RD2 // BT state LED
#define DemuxE2 RD4 // Demux input 1
#define DemuxA2 RD5 // Demux input 2
#define DemuxA1 RD6 // Demux input 3
#define DemuxA0 RD7 // Demux input 4

// Global variables
volatile int emergency_stop_active = 0;

// OLED last displayed values to avoid rewriting the same content
char lastBT[20] = "";
char lastAngle[20] = "";
char lastCycles[20] = "";

void Init_ports() {
    TRISC6 = 0; // TX of the BT module as output
    TRISC7 = 1; // RX of the BT module as input
    TRISB0 = 1; // SDA
    TRISB1 = 1; // SCL
    TRISA0 = 1; // AN0 input (joystick)
    TRISA1 = 1; // AN1 input (accelerometer Y axis)
    TRISA2 = 1; // AN2 input (accelerometer C axis)
    TRISA3 = 1; // AN3 input (motor voltage)
    TRISD4 = 0; // Led 1 as output
    TRISD5 = 0; // Led 2 as output
    TRISD6 = 0; // Led 3 as output
    TRISD7 = 0; // Led 4 as output
    TRISB2 = 0; // Stepper inputA
    TRISB3 = 0; // Stepper inputB
    TRISB4 = 0; // Stepper inputC
    TRISB5 = 0; // Stepper inputD
    TRISD0 = 1; // Forward button as input
    TRISD1 = 1; // Backwards button as input
    INTEDG1 = 1; // INT1 rising edge
    INTEDG2 = 1; // INT2 rising edge
}

// EUSART configuartion
void UART_Init(void) {
    SPBRG = 103; // Baud rate value for 9600 bps @ 16 MHz
    BRGH = 1; // High-speed baud rate mode
    SYNC = 0; // Asynchronous mode
    TXEN = 1; // Enable transmitter
    SPEN = 1; // Enable serial port (UART pins active)
    CREN = 1; // Enable continuous reception
}

// Sends one character over UART when the transmitter is ready
void UART_WriteChar(char c) {
    while (!TRMT);
    TXREG = c;
}

// Sends a full string over UART, character by character
void UART_WriteString(const char *s) {
    while (*s) UART_WriteChar(*s++);
}

//////// I2C ////////

// Initializes I2C in Master mode at 400 kHz
void I2C_Init(void) {
    SSPCON1 = 0b00101000; // I2C Master mode
    SSPCON2 = 0;
    SSPADD = (_XTAL_FREQ / (4 * 400000UL)) - 1; // 400 kHz
    SSPSTAT = 0;
}

// Waits until I2C bus and module are ready
void I2C_Wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTATbits.R_nW));
}

// Sends an I2C START condition
void I2C_Start(void) {
    I2C_Wait();
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN);
}

// Sends an I2C STOP condition
void I2C_Stop(void) {
    I2C_Wait();
    SSPCON2bits.PEN = 1;
    while (SSPCON2bits.PEN);
}
// Sends one byte over I2C

void I2C_Write(uint8_t data) {
    I2C_Wait();
    SSPBUF = data;
    while (SSPSTATbits.BF);
}
// OLED
#define OLED_ADDR 0x3C

// Sends a command byte to the OLED via I2C
void OLED_Command(uint8_t cmd) {
    I2C_Start();
    I2C_Write(OLED_ADDR << 1);
    I2C_Write(0x00);
    I2C_Write(cmd);
    I2C_Stop();
}

// Initializes the OLED display by sending the standard configuration command sequence
void OLED_Init(void) {
    __delay_ms(100);
    OLED_Command(0xAE);
    OLED_Command(0xD5);
    OLED_Command(0x80);
    OLED_Command(0xA8);
    OLED_Command(0x3F);
    OLED_Command(0xD3);
    OLED_Command(0x00);
    OLED_Command(0x40);
    OLED_Command(0x8D);
    OLED_Command(0x14);
    OLED_Command(0x20);
    OLED_Command(0x00);
    OLED_Command(0xA1);
    OLED_Command(0xC8);
    OLED_Command(0xDA);
    OLED_Command(0x12);
    OLED_Command(0x81);
    OLED_Command(0xCF);
    OLED_Command(0xD9);
    OLED_Command(0xF1);
    OLED_Command(0xDB);
    OLED_Command(0x40);
    OLED_Command(0xA4);
    OLED_Command(0xA6);
    OLED_Command(0xAF);
}

// Clears a rectangular area of the OLED by writing zeros to the specified pages and columns
void OLED_ClearArea(uint8_t startPage, uint8_t endPage, uint8_t startCol, uint8_t endCol) {
    for (uint8_t page = startPage; page <= endPage; page++) {
        OLED_Command(0xB0 + page);
        OLED_Command(0x00 + (startCol & 0x0F));
        OLED_Command(0x10 + ((startCol >> 4) & 0x0F));
        I2C_Start();
        I2C_Write(OLED_ADDR << 1);
        I2C_Write(0x40);
        for (uint8_t col = startCol; col <= endCol; col++) I2C_Write(0x00);
        I2C_Stop();
    }
}

// Fonts and digits
const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space (0)
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0' (1)
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1' (2)
    {0x62, 0x51, 0x49, 0x49, 0x46}, // '2' (3)
    {0x22, 0x41, 0x49, 0x49, 0x36}, // '3' (4)
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4' (5)
    {0x2F, 0x49, 0x49, 0x49, 0x31}, // '5' (6)
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6' (7)
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7' (8)
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8' (9)
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9' (10)
    {0x06, 0x09, 0x09, 0x06, 0x00}, // '°' (11)
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-' (12)
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 13: 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 14: 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 15: 'C'
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // 16: 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 17: 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 18: 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 19: 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 20: 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 21: 'I'
    {0x30, 0x40, 0x40, 0x40, 0x3F}, // 22: 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 23: 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 24: 'L'
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // 25: 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 26: 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 27: 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 28: 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 29: 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 30: 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 31: 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 32: 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 33: 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 34: 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 35: 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 36: 'X'
    {0x03, 0x04, 0x78, 0x04, 0x03}, // 37: 'Y'
    {0x47, 0x49, 0x51, 0x61, 0x41}, // 38: 'Z'
};

// Sets the OLED cursor to the specified page and column
void OLED_SetCursor(uint8_t page, uint8_t col) {
    OLED_Command(0xB0 + page); // Select page (row of 8 pixels)
    OLED_Command(0x00 + (col & 0x0F)); // Set lower 4 bits of column
    OLED_Command(0x10 + ((col >> 4) & 0x0F)); // Set upper 4 bits of column
}

// Draw a character on the OLED, scaled by a factor
void OLED_DrawScaledChar(char c, uint8_t scale, uint8_t start_page, uint8_t start_col) {
    uint8_t idx;
    // Map the character to its font index
    if (c >= '0' && c <= '9') {
        idx = c - '0' + 1; // Numbers
    } else if (c >= 'A' && c <= 'Z') {
        idx = c - 'A' + 13; // Letters
    } else if ((uint8_t) c == 248) {
        idx = 11; // Degree symbol
    } else if (c == '-') {
        idx = 12; // Minus
    } else if (c == ' ') {
        idx = 0; // Space
    } else {
        return; // Ignore unsupported characters
    }
    // Loop through each column of the character
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t colData = font5x7[idx][col]; // Get pixel pattern
        uint8_t scaled_col[3] = {0}; // For vertical scaling
        uint8_t rowOut = 0;
        // Scale pixels vertically
        for (uint8_t bit = 0; bit < 7; bit++) {
            uint8_t pixel = (colData >> bit) & 0x01;
            for (uint8_t v = 0; v < scale; v++) {
                uint8_t outPage = rowOut / 8;
                uint8_t outBit = rowOut % 8;
                if (pixel) scaled_col[outPage] |= (1 << outBit);
                rowOut++;
            }
        }
        // Draw horizontally and vertically scaled pixels
        for (uint8_t x = 0; x < scale; x++) {
            for (uint8_t p = 0; p < scale; p++) {
                OLED_SetCursor(start_page + p, start_col + (col * scale) + x);
                I2C_Start();
                I2C_Write(OLED_ADDR << 1);
                I2C_Write(0x40);
                I2C_Write(scaled_col[p]);
                I2C_Stop();
            }
        }
    }
}

// Each character is scaled 2× in size, and all characters are drawn on the specified page (row).
void OLED_DrawCentered2x_AtPage(const char* str, uint8_t page) {
    uint8_t len = 0;
    const char* tmp = str;
    while (*tmp++) len++; // Count characters
    uint8_t charWidth = 5 * 2; // Width of each character
    uint8_t totalWidth = len * (charWidth + 2); // Total string width
    uint8_t startCol = (128 - totalWidth) / 2; // Calculate starting column to center
    // Draw each character at correct position
    for (uint8_t i = 0; i < len; i++) {
        OLED_DrawScaledChar(str[i], 2, page, startCol + i * (charWidth + 2));
    }
}
// Draw a single character on the OLED, unscaled

void OLED_DrawChar(const char c, uint8_t start_page, uint8_t start_col) {
    uint8_t idx;
    // Map the character to its font index
    if (c >= '0' && c <= '9') {
        idx = c - '0' + 1;
    } else if (c >= 'A' && c <= 'Z') {
        idx = c - 'A' + 13;
    } else if ((uint8_t) c == 248) {
        idx = 11;
    } else if (c == '-') {
        idx = 12;
    } else if (c == ' ') {
        idx = 0;
    } else return;
    // Draw each column of the character
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t colData = font5x7[idx][col];
        OLED_SetCursor(start_page, start_col + col);
        I2C_Start();
        I2C_Write(OLED_ADDR << 1);
        I2C_Write(0x40);
        I2C_Write(colData);
        I2C_Stop();
    }
    // Leave a small gap after the character
    OLED_SetCursor(start_page, start_col + 5);
    I2C_Start();
    I2C_Write(OLED_ADDR << 1);
    I2C_Write(0x40);
    I2C_Write(0x00);
    I2C_Stop();
}

// Print a string on the OLED starting at a specific page and column
void OLED_Print(const char* str, uint8_t page, uint8_t col) {
    while (*str) {
        OLED_DrawChar(*str, page, col); // Draw each character
        col += 6; // Move cursor to next character
        str++;
    }
}

// ADC Configuration
void Config_ADC() {
    // ADCON1 - Configure which pins are analog and which are digital
    PCFG3 = 1; // AN0 - AN3 analog, rest digital
    PCFG2 = 0;
    PCFG1 = 1;
    PCFG0 = 1;
    ADFM = 1; // Right-justified
    ADCS2 = 0; // ADC clock Fosc/32
    ADCS1 = 1;
    ADCS0 = 0;
    ADON = 1; // Enable ADC
    __delay_ms(100);
}

// Read Joystick value (AN0)
float Joystick_ADC() {
    // Select AN0 channel
    CHS3 = 0;
    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 0;
    __delay_us(20);
    GO_DONE = 1; // Start ADC conversion
    while (GO_DONE); // Wait until conversion finishes
    // Combine high and low result bytes (0-1023)
    float value = ADRESH * 256.0f + ADRESL;
    return value;
}

// Read Gyroscope value (AN1) and convert to angle
float GY_ADC() {
    // Select AN1
    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 1;
    GO_DONE = 1; // Start ADC conversion
    while (GO_DONE); // Wait until conversion finishes
    float rawy = ADRESH * 256.0f + ADRESL; // Combine high and low result bytes (0-1023)
    float voltage = (5.0f * rawy) / 1023.0f; // Convert ADC reading to voltage (0-5V)
    float gAcc = (voltage - 2.35f) / 0.76f; // Convert voltage to g-force
    if (gAcc > 1.0f) gAcc = 1.0f; // Set gravity between 1 and -1 to be a valid parameter for the arcsin
    if (gAcc < -1.0f) gAcc = -1.0f;
    float angle = asin(gAcc) * (180.0f / 3.1415926f); // Convert g-force to angle in degrees using arcsine
    return angle;
}

// Interrupts
void Enable_Interrupts() {
    GIE = 1; // Enable global interrupts
    INT1IE = 1; // Enable INT1 interrupt
    INT1IF = 0; // Clear INT1 flag
    INT2IE = 1; // Enable INT2 interrupt
    INT2IF = 0; // Clear INT2 flag
}

void __interrupt() ISR(void) {
    if (INT1IF && INT2IF) {
        // Both buttons pressed simultaneously - EMERGENCY STOP
        emergency_stop_active = 1;
        // Turn off all demux inputs
        DemuxA2 = 0;
        DemuxA1 = 0;
        DemuxA0 = 0;
        DemuxE2 = 0;
        // Send emergency stop message via UART BEFORE OLED
        UART_WriteString("\r\n!!! EMERGENCY STOP ACTIVATED !!!\r\n");
        // Small delay to ensure system is stable
        __delay_ms(100);
        // Clear OLED and display emergency message
        OLED_ClearArea(0, 7, 0, 127);
        __delay_ms(100); // Extra delay after clearing
        OLED_DrawCentered2x_AtPage("EMERGENCY", 2);
        __delay_ms(50);
        OLED_DrawCentered2x_AtPage("STOP", 4);
        // Clear both interrupt flags
        INT1IF = 0;
        INT2IF = 0;
        // Wait for buttons to be released
        while (MotorF == 1 || MotorB == 1) {
            __delay_ms(10);
        }
        // Enter monitoring loop - wait for both buttons to be pressed again to exit
        while (1) {
            // Check if both buttons are pressed again to exit emergency stop
            if (MotorF == 1 && MotorB == 1) {
                // Wait for confirmation (debounce)
                __delay_ms(100);
                if (MotorF == 1 && MotorB == 1) {
                    // Exit emergency stop - perform system reboot
                    UART_WriteString("\r\n!!! SYSTEM REBOOTING !!!\r\n");
                    OLED_ClearArea(0, 7, 0, 127);
                    __delay_ms(100);
                    OLED_DrawCentered2x_AtPage("REBOOTING", 3);
                    __delay_ms(1000);
                    // Software reset by jumping to reset vector
                    asm("RESET");
                }
            }
            __delay_ms(50);
        }
    }
}

// Demux LEDs control
void Demux_control() {
    int angleCondition = GY_ADC();
    DemuxE2 = 1;
    if (angleCondition < 10 && angleCondition > -10) {
        DemuxE2 = 0;
    } else {
        DemuxE2 = 1;
        if (angleCondition <= -70) {
            DemuxA2 = 0;
            DemuxA1 = 0;
            DemuxA0 = 0;
        } else if (angleCondition > -70 && angleCondition <= -50) {
            DemuxA2 = 0;
            DemuxA1 = 0;
            DemuxA0 = 1;
        } else if (angleCondition > -50 && angleCondition <= -30) {
            DemuxA2 = 0;
            DemuxA1 = 1;
            DemuxA0 = 0;
        } else if (angleCondition > -30 && angleCondition <= -10) {
            DemuxA2 = 0;
            DemuxA1 = 1;
            DemuxA0 = 1;
        } else if (angleCondition > 10 && angleCondition <= 30) {
            DemuxA2 = 1;
            DemuxA1 = 0;
            DemuxA0 = 0;
        } else if (angleCondition > 30 && angleCondition <= 50) {
            DemuxA2 = 1;
            DemuxA1 = 0;
            DemuxA0 = 1;
        } else if (angleCondition > 50 && angleCondition <= 70) {
            DemuxA2 = 1;
            DemuxA1 = 1;
            DemuxA0 = 0;
        } else {
            DemuxA2 = 1;
            DemuxA1 = 1;
            DemuxA0 = 1;
        }
    }
}

// Main
void main(void) {
    Init_ports();
    UART_Init();
    I2C_Init();
    OLED_Init();
    Config_ADC();
    Enable_Interrupts();
    OLED_ClearArea(0, 7, 0, 127);
    // WELCOME MESSAGE FOR DAFNE
    __delay_ms(200); // Give OLED time to be ready
    OLED_DrawCentered2x_AtPage("WELCOME", 2);
    __delay_ms(50);
    OLED_DrawCentered2x_AtPage("DAFNE", 4);
    UART_WriteString("Welcome Dafne!\r\n");
    __delay_ms(2000); // Display welcome message for 2 seconds
    OLED_ClearArea(0, 7, 0, 127); // Clear welcome message
    OLED_Print("-", 0, 114); // direction display
    char buffer[30];
    int joystick;
    int angle;
    UART_WriteString("System started\r\n");
    while (1) {
        // Emergency stop
        if (MotorF == 1 && MotorB == 1 && emergency_stop_active == 0) {
            // Trigger software emergency stop
            INT1IF = 1;
            INT2IF = 1;
        }
        Demux_control();
        joystick = (int) Joystick_ADC();
        angle = GY_ADC();
        sprintf(buffer, "%d,%d\r\n", joystick, angle);
        UART_WriteString(buffer);
        // Verify if bt status has changed
        char btMsg[20];
        if (bt_state == 1) sprintf(btMsg, "BT ACTIVE");
        else sprintf(btMsg, "");
        if (strcmp(btMsg, lastBT) != 0) {
            strcpy(lastBT, btMsg);
            OLED_ClearArea(0, 1, 0, 127);
            if (bt_state == 1) OLED_Print("BT", 0, 0);
        }
        // Verify if angle has changed
        char angleStr[20];
        sprintf(angleStr, "%d%c", angle, 248);
        if (strcmp(angleStr, lastAngle) != 0) {
            strcpy(lastAngle, angleStr);
            OLED_ClearArea(1, 2, 0, 127);
            OLED_DrawCentered2x_AtPage(angleStr, 3);
        }
        if (joystick < 495) OLED_Print("F", 0, 106);
        else if (joystick > 535) OLED_Print("B", 0, 106);
        else OLED_Print("X", 0, 106);
        if (angle > 8) OLED_Print("R", 0, 122);
        else if (angle < -8) OLED_Print("L", 0, 122);
        else OLED_Print("X", 0, 122);
        __delay_ms(70);
    }
}