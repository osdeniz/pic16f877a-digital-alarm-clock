#include <xc.h>        // Microcontroller definitions for PIC16F877A
#include <stdint.h>    // Standard integer types (uint8_t, etc.)
#include <stdio.h>     // Standard I/O library for sprintf function

// ============================================================================
// CONFIGURATION BITS - Hardware setup for PIC16F877A
// ============================================================================
#pragma config FOSC = HS    // High-speed crystal oscillator (20MHz)
#pragma config WDTE = OFF   // Watchdog timer disabled
#pragma config PWRTE = OFF  // Power-up timer disabled
#pragma config BOREN = ON   // Brown-out reset enabled (resets when voltage drops)
#pragma config LVP = OFF    // Low voltage programming disabled
#pragma config CPD = OFF    // EEPROM code protection disabled
#pragma config WRT = OFF    // Flash memory write protection disabled
#pragma config CP = OFF     // Code protection disabled

#define _XTAL_FREQ 20000000  // 20MHz oscillator frequency for delay calculations

// ============================================================================
// DS3232 RTC MODULE DEFINITIONS
// ============================================================================
#define DS3232_ADDRESS 0xD0  // DS3232 I2C slave address (write address)
#define DS3232_READ    0xD1  // DS3232 I2C read address

// DS3232 Register addresses
#define DS3232_SECONDS    0x00
#define DS3232_MINUTES    0x01
#define DS3232_HOURS      0x02
#define DS3232_DAY        0x03
#define DS3232_DATE       0x04
#define DS3232_MONTH      0x05
#define DS3232_YEAR       0x06
#define DS3232_ALARM1_SEC 0x07
#define DS3232_ALARM1_MIN 0x08
#define DS3232_ALARM1_HOUR 0x09
#define DS3232_CONTROL    0x0E
#define DS3232_STATUS     0x0F

// I2C pins for DS3232 communication
#define SDA_PIN         TRISCbits.TRISC4
#define SCL_PIN         TRISCbits.TRISC3
#define SDA_DATA        PORTCbits.RC4
#define SCL_DATA        PORTCbits.RC3

// ============================================================================
// LCD DISPLAY CONFIGURATION (16x4 Character LCD in 4-bit mode)
// ============================================================================
// LCD control pins connected to PORTD
#define LCD_RS PORTDbits.RD0    // Register Select: 0=Command, 1=Data
#define LCD_RW PORTDbits.RD1    // Read/Write: 0=Write, 1=Read (always write)
#define LCD_EN PORTDbits.RD2    // Enable: Triggers LCD operation on falling edge

// LCD data pins (4-bit mode uses only D4-D7)
#define LCD_D4 PORTDbits.RD3    // Data bit 4
#define LCD_D5 PORTDbits.RD4    // Data bit 5
#define LCD_D6 PORTDbits.RD5    // Data bit 6
#define LCD_D7 PORTDbits.RD6    // Data bit 7

// ============================================================================
// BUTTON AND OUTPUT DEFINITIONS
// ============================================================================
// Input buttons connected to PORTB (with internal pull-ups enabled)
#define BTN_NEXT    PORTBbits.RB1    // Next/Select button (active low)
#define BTN_INC     PORTBbits.RB2    // Increment button (active low)
#define BTN_MODE    PORTBbits.RB3    // Mode switch: 0=Clock setting, 1=Alarm setting

// Output devices
#define ALARM_OUT   PORTBbits.RB4    // Buzzer output for alarm sound
#define ALARM_LED   PORTCbits.RC7    // LED indicator for alarm status

// ============================================================================
// GLOBAL VARIABLES - System state and time keeping
// ============================================================================
// Current time variables (read from DS3232)
uint8_t hour = 0, minute = 0, second = 0;

// Current date variables (read from DS3232)
uint8_t date = 1, month = 1, year = 0;  // Year is 2-digit (20xx)

// Alarm time settings (stored in DS3232 alarm registers)
uint8_t alarm_hour = 0, alarm_minute = 0;  // Default alarm at 00:00

// User interface state variables
uint8_t field = 0;          // Currently selected field for editing (0-5)
uint8_t mode = 0;           // Current mode: 0=Clock setting, 1=Alarm setting
uint8_t alarm_triggered = 0; // Flag indicating if alarm is currently active
uint8_t alarm_enabled = 1;   // Flag to enable/disable alarm functionality

// ============================================================================
// LCD DISPLAY BUFFERS - Formatted strings for display
// ============================================================================
char time_str[17];    // Buffer for time display string
char date_str[17];    // Buffer for date display string  
char alarm_str[17];   // Buffer for alarm status string

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
// I2C communication functions for DS3232
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_read(uint8_t ack);
void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t data);
uint8_t i2c_read_byte(uint8_t address, uint8_t reg);

// DS3232 RTC functions
void ds3232_init(void);
void ds3232_set_time(uint8_t hour, uint8_t minute, uint8_t second);
void ds3232_set_date(uint8_t date, uint8_t month, uint8_t year);
void ds3232_read_time(void);
void ds3232_read_date(void);
void ds3232_set_alarm(uint8_t hour, uint8_t minute);
uint8_t ds3232_check_alarm(void);
uint8_t bcd_to_dec(uint8_t bcd);
uint8_t dec_to_bcd(uint8_t dec);

// LCD low-level functions
void lcd_send_nibble(uint8_t nibble);  // Send 4 bits to LCD data pins
void lcd_cmd(uint8_t cmd);             // Send command to LCD
void lcd_data(char data);              // Send data character to LCD
void lcd_init(void);                   // Initialize LCD in 4-bit mode
void lcd_puts(const char *s);          // Display string on LCD
void lcd_gotoxy(uint8_t x, uint8_t y); // Set cursor position
void lcd_clear(void);                  // Clear LCD display

// ============================================================================
// I2C COMMUNICATION FUNCTIONS FOR DS3232
// ============================================================================

// Initialize I2C hardware for DS3232 communication
void i2c_init(void) {
    SDA_PIN = 1;    // Set SDA as input (open-drain)
    SCL_PIN = 1;    // Set SCL as input (open-drain)
    SDA_DATA = 1;   // SDA high
    SCL_DATA = 1;   // SCL high
}

// Generate I2C start condition
void i2c_start(void) {
    SDA_PIN = 0; SDA_DATA = 1;  // SDA high
    SCL_PIN = 0; SCL_DATA = 1;  // SCL high
    __delay_us(5);
    SDA_PIN = 0; SDA_DATA = 0;  // SDA low while SCL high
    __delay_us(5);
    SCL_PIN = 0; SCL_DATA = 0;  // SCL low
    __delay_us(5);
}

// Generate I2C stop condition
void i2c_stop(void) {
    SDA_PIN = 0; SDA_DATA = 0;  // SDA low
    SCL_PIN = 0; SCL_DATA = 1;  // SCL high
    __delay_us(5);
    SDA_PIN = 0; SDA_DATA = 1;  // SDA high while SCL high
    __delay_us(5);
}

// Write 8 bits to I2C bus
void i2c_write(uint8_t data) {
    for(uint8_t i = 0; i < 8; i++) {
        if(data & 0x80) {
            SDA_PIN = 1;  // SDA high
        } else {
            SDA_PIN = 0; SDA_DATA = 0;  // SDA low
        }
        __delay_us(2);
        SCL_PIN = 0; SCL_DATA = 1;  // SCL high
        __delay_us(5);
        SCL_PIN = 0; SCL_DATA = 0;  // SCL low
        __delay_us(2);
        data <<= 1;
    }
    
    // Read ACK
    SDA_PIN = 1;  // Release SDA
    __delay_us(2);
    SCL_PIN = 0; SCL_DATA = 1;  // SCL high
    __delay_us(5);
    SCL_PIN = 0; SCL_DATA = 0;  // SCL low
    __delay_us(2);
}

// Read 8 bits from I2C bus
uint8_t i2c_read(uint8_t ack) {
    uint8_t data = 0;
    SDA_PIN = 1;  // Release SDA for reading
    
    for(uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCL_PIN = 0; SCL_DATA = 1;  // SCL high
        __delay_us(5);
        if(SDA_DATA) data |= 1;
        SCL_PIN = 0; SCL_DATA = 0;  // SCL low
        __delay_us(5);
    }
    
    // Send ACK/NACK
    if(ack) {
        SDA_PIN = 0; SDA_DATA = 0;  // ACK (SDA low)
    } else {
        SDA_PIN = 0; SDA_DATA = 1;  // NACK (SDA high)
    }
    __delay_us(2);
    SCL_PIN = 0; SCL_DATA = 1;  // SCL high
    __delay_us(5);
    SCL_PIN = 0; SCL_DATA = 0;  // SCL low
    __delay_us(2);
    
    return data;
}

// Write single byte to DS3232 register
void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t data) {
    i2c_start();
    i2c_write(address);     // Device address
    i2c_write(reg);         // Register address
    i2c_write(data);        // Data
    i2c_stop();
}

// Read single byte from DS3232 register
uint8_t i2c_read_byte(uint8_t address, uint8_t reg) {
    uint8_t data;
    
    i2c_start();
    i2c_write(address);     // Device address (write)
    i2c_write(reg);         // Register address
    
    i2c_start();            // Repeated start
    i2c_write(address | 1); // Device address (read)
    data = i2c_read(0);     // Read data with NACK
    i2c_stop();
    
    return data;
}

// ============================================================================
// DS3232 RTC FUNCTIONS
// ============================================================================

// Convert BCD to decimal
uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Convert decimal to BCD
uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) + (dec % 10);
}

// Initialize DS3232 RTC module
void ds3232_init(void) {
    // Enable oscillator and disable alarms initially
    i2c_write_byte(DS3232_ADDRESS, DS3232_CONTROL, 0x04);  // Enable oscillator, disable alarms
    i2c_write_byte(DS3232_ADDRESS, DS3232_STATUS, 0x00);   // Clear status register
}

// Set time in DS3232
void ds3232_set_time(uint8_t hour, uint8_t minute, uint8_t second) {
    i2c_write_byte(DS3232_ADDRESS, DS3232_SECONDS, dec_to_bcd(second));
    i2c_write_byte(DS3232_ADDRESS, DS3232_MINUTES, dec_to_bcd(minute));
    i2c_write_byte(DS3232_ADDRESS, DS3232_HOURS, dec_to_bcd(hour));
}

// Set date in DS3232
void ds3232_set_date(uint8_t date, uint8_t month, uint8_t year) {
    i2c_write_byte(DS3232_ADDRESS, DS3232_DATE, dec_to_bcd(date));
    i2c_write_byte(DS3232_ADDRESS, DS3232_MONTH, dec_to_bcd(month));
    i2c_write_byte(DS3232_ADDRESS, DS3232_YEAR, dec_to_bcd(year));
}

// Read current time from DS3232
void ds3232_read_time(void) {
    second = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_SECONDS) & 0x7F);
    minute = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_MINUTES) & 0x7F);
    hour = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_HOURS) & 0x3F);
}

// Read current date from DS3232
void ds3232_read_date(void) {
    date = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_DATE) & 0x3F);
    month = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_MONTH) & 0x1F);
    year = bcd_to_dec(i2c_read_byte(DS3232_ADDRESS, DS3232_YEAR));
}

// Set alarm time in DS3232
void ds3232_set_alarm(uint8_t hour, uint8_t minute) {
    i2c_write_byte(DS3232_ADDRESS, DS3232_ALARM1_SEC, 0x00);  // 00 seconds
    i2c_write_byte(DS3232_ADDRESS, DS3232_ALARM1_MIN, dec_to_bcd(minute));
    i2c_write_byte(DS3232_ADDRESS, DS3232_ALARM1_HOUR, dec_to_bcd(hour));
    
    // Enable alarm 1 interrupt
    if(alarm_enabled) {
        i2c_write_byte(DS3232_ADDRESS, DS3232_CONTROL, 0x05);  // Enable alarm 1 interrupt
    } else {
        i2c_write_byte(DS3232_ADDRESS, DS3232_CONTROL, 0x04);  // Disable alarm 1 interrupt
    }
}

// Check if alarm is triggered
uint8_t ds3232_check_alarm(void) {
    uint8_t status = i2c_read_byte(DS3232_ADDRESS, DS3232_STATUS);
    if(status & 0x01) {  // Alarm 1 flag set
        // Clear alarm flag
        i2c_write_byte(DS3232_ADDRESS, DS3232_STATUS, status & 0xFE);
        return 1;
    }
    return 0;
}

// ============================================================================
// DISPLAY UPDATE FUNCTION
// ============================================================================
// Updates all information on the 16x4 LCD display
// Line 0: Current time (HH:MM:SS)
// Line 1: Current date (DD/MM/YYYY) 
// Line 2: Alarm time and status
// Line 3: Current setting mode
void lcd_display_all(void) {
    // Format and display current time on line 0
    sprintf(time_str, "TIME: %02u:%02u:%02u", hour, minute, second);
    lcd_gotoxy(0, 0); 
    lcd_puts(time_str);

    // Format and display current date on line 1
    sprintf(date_str, "DATE: %02u/%02u/20%02u", date, month, year);
    lcd_gotoxy(0, 1); 
    lcd_puts(date_str);

    // Format and display alarm settings on line 2
    sprintf(alarm_str, "ALARM: %02u:%02u [%s]", alarm_hour, alarm_minute, alarm_enabled ? "ON" : "OFF");
    lcd_gotoxy(0, 2); 
    lcd_puts(alarm_str);

    // Display current setting mode on line 3
    lcd_gotoxy(0, 3);
    lcd_puts(mode == 0 ? "SETTING: CLOCK" : "SETTING: ALARM");
}

// ============================================================================
// LCD IMPLEMENTATION FUNCTIONS
// ============================================================================

// Send 4-bit nibble to LCD data pins D4-D7
// Each LCD operation requires sending 8 bits as two 4-bit nibbles
void lcd_send_nibble(uint8_t nibble) {
    // Set data pins according to nibble bits
    LCD_D4 = (nibble >> 0) & 1;
    LCD_D5 = (nibble >> 1) & 1;
    LCD_D6 = (nibble >> 2) & 1;
    LCD_D7 = (nibble >> 3) & 1;
    
    // Generate enable pulse (high-to-low transition triggers LCD)
    LCD_EN = 1; 
    __delay_us(1);      // Enable pulse width
    LCD_EN = 0; 
    __delay_us(100);    // Processing time
}

// Send command to LCD (RS=0 for command mode)
void lcd_cmd(uint8_t cmd) {
    LCD_RS = 0;                     // Command mode
    LCD_RW = 0;                     // Write mode
    lcd_send_nibble(cmd >> 4);      // Send upper nibble first
    lcd_send_nibble(cmd & 0x0F);    // Send lower nibble
    __delay_ms(2);                  // Command processing delay
}

// Send data character to LCD (RS=1 for data mode)
void lcd_data(char data) {
    LCD_RS = 1;                     // Data mode
    LCD_RW = 0;                     // Write mode
    lcd_send_nibble(data >> 4);     // Send upper nibble first
    lcd_send_nibble(data & 0x0F);   // Send lower nibble
    __delay_us(100);                // Character processing delay
}

// Initialize LCD in 4-bit mode according to HD44780 datasheet
void lcd_init(void) {
    __delay_ms(20);                 // Wait for LCD power-up
    
    // Initialization sequence for 4-bit mode
    lcd_send_nibble(0x03); __delay_ms(5);    // Function set (8-bit mode)
    lcd_send_nibble(0x03); __delay_us(150);  // Function set (8-bit mode)
    lcd_send_nibble(0x03);                   // Function set (8-bit mode)
    lcd_send_nibble(0x02);                   // Function set (4-bit mode)
    
    // Configure LCD parameters
    lcd_cmd(0x28);    // 4-bit mode, 2 lines, 5x7 font
    lcd_cmd(0x0C);    // Display on, cursor off, blink off
    lcd_cmd(0x06);    // Entry mode: increment cursor, no shift
    lcd_cmd(0x01);    // Clear display
}

// Display null-terminated string on LCD
void lcd_puts(const char *s) {
    while(*s) lcd_data(*s++);
}

// Set cursor position on LCD (x=column, y=row)
// LCD memory addresses: Line0=0x00, Line1=0x40, Line2=0x14, Line3=0x54
void lcd_gotoxy(uint8_t x, uint8_t y) {
    uint8_t address[] = {0x00, 0x40, 0x14, 0x54};  // DDRAM addresses for each line
    lcd_cmd(0x80 | (address[y] + x));               // Set DDRAM address command
}

// Clear LCD display and reset cursor to home position
void lcd_clear(void) {
    lcd_cmd(0x01);      // Clear display command
    __delay_ms(2);      // Clear command processing delay
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================
void main(void) {
    // ========================================================================
    // HARDWARE INITIALIZATION
    // ========================================================================
    
    // Configure port directions
    TRISD = 0x00;              // PORTD all outputs (LCD control and data)
    TRISB = 0x0F;              // RB0-RB3 inputs (buttons), RB4+ outputs
    TRISBbits.TRISB4 = 0;      // RB4 output (buzzer)
    TRISCbits.TRISC7 = 0;      // RC7 output (alarm LED)
    
    // Initialize port values
    PORTD = 0x00;              // Clear LCD port
    PORTB = 0x00;              // Clear button port  
    PORTC = 0x00;              // Clear LED port
    ALARM_OUT = 0;             // Buzzer off
    ALARM_LED = 0;             // Alarm LED off

    // Enable internal pull-up resistors for button inputs
    OPTION_REGbits.nRBPU = 0;  // Enable PORTB pull-ups

    // ========================================================================
    // I2C AND DS3232 INITIALIZATION
    // ========================================================================
    i2c_init();                // Initialize I2C communication
    ds3232_init();             // Initialize DS3232 RTC module
    
    // Set initial alarm time in DS3232
    ds3232_set_alarm(alarm_hour, alarm_minute);

    // ========================================================================
    // LCD INITIALIZATION AND STARTUP
    // ========================================================================
    lcd_init();                // Initialize LCD display
    lcd_clear();               // Clear display

    // ========================================================================
    // MAIN PROGRAM LOOP
    // ========================================================================
    while(1) {
        // ====================================================================
        // TIME READING FROM DS3232
        // ====================================================================
        ds3232_read_time();     // Read current time from DS3232
        ds3232_read_date();     // Read current date from DS3232

        // Update LCD display with current values
        lcd_display_all();

        // ====================================================================
        // USER INPUT PROCESSING (Button handling with debouncing)
        // ====================================================================
        
        // MODE button: Switch between clock setting and alarm setting
        if (!BTN_MODE) {
            mode = !mode;           // Toggle mode
            alarm_triggered = 0;    // Clear alarm trigger when changing modes
            while (!BTN_MODE);      // Wait for button release (debounce)
            __delay_ms(200);        // Additional debounce delay
        }

        // INC button: Move to next field for editing
        if (!BTN_INC) {
            if (mode == 0)          // Clock setting mode: 6 fields
                field = (field + 1) % 6;
            else                    // Alarm setting mode: 3 fields  
                field = (field + 1) % 3;
            while (!BTN_INC);       // Wait for button release
            __delay_ms(200);        // Debounce delay
        }

        // NEXT button: Increment currently selected field
        if (!BTN_NEXT) {
            if (mode == 0) {        // Clock setting mode
                if (field == 0) {
                    hour = (hour + 1) % 24;           // Hours (0-23)
                    ds3232_set_time(hour, minute, second);  // Update DS3232
                }
                if (field == 1) {
                    minute = (minute + 1) % 60;       // Minutes (0-59)
                    ds3232_set_time(hour, minute, second);  // Update DS3232
                }
                if (field == 2) {
                    second = (second + 1) % 60;       // Seconds (0-59)
                    ds3232_set_time(hour, minute, second);  // Update DS3232
                }
                if (field == 3) {
                    date = (date % 31) + 1;           // Date (1-31)
                    ds3232_set_date(date, month, year);     // Update DS3232
                }
                if (field == 4) {
                    month = (month % 12) + 1;         // Month (1-12)
                    ds3232_set_date(date, month, year);     // Update DS3232
                }
                if (field == 5) {
                    year = (year + 1) % 100;          // Year (0-99)
                    ds3232_set_date(date, month, year);     // Update DS3232
                }
            } else {                // Alarm setting mode
                if (field == 0) {
                    alarm_hour = (alarm_hour + 1) % 24;      // Alarm hours
                    ds3232_set_alarm(alarm_hour, alarm_minute); // Update DS3232 alarm
                }
                else if (field == 1) {
                    alarm_minute = (alarm_minute + 1) % 60; // Alarm minutes
                    ds3232_set_alarm(alarm_hour, alarm_minute); // Update DS3232 alarm
                }
                else if (field == 2) {
                    alarm_enabled = !alarm_enabled;     // Alarm on/off
                    ds3232_set_alarm(alarm_hour, alarm_minute); // Update DS3232 alarm
                }
            }
            while (!BTN_NEXT);      // Wait for button release
            __delay_ms(200);        // Debounce delay
        }

        // ====================================================================
        // ALARM LOGIC USING DS3232
        // ====================================================================
        // Check if alarm is triggered by DS3232
        if (alarm_enabled && ds3232_check_alarm()) {
            alarm_triggered = 1;    // Set alarm active flag
            ALARM_OUT = 1;          // Turn on buzzer
            ALARM_LED = 1;          // Turn on alarm LED
        } else if (!alarm_enabled) {
            alarm_triggered = 0;
            ALARM_OUT = 0;          // Turn off buzzer
            ALARM_LED = 0;          // Turn off alarm LED
        }

        __delay_ms(100);            // Main loop delay for stable operation
    }
}