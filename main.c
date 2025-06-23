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
// Current time variables (volatile because modified in interrupt)
volatile uint8_t hour = 0, minute = 0, second = 0;

// Current date variables
uint8_t date = 1, month = 1, year = 0;  // Year is 2-digit (20xx)

// Alarm time settings
uint8_t alarm_hour = 0, alarm_minute = 0;  // Default alarm at 00:00

// User interface state variables
uint8_t field = 0;          // Currently selected field for editing (0-5)
uint8_t mode = 0;           // Current mode: 0=Clock setting, 1=Alarm setting
uint8_t alarm_triggered = 0; // Flag indicating if alarm is currently active
uint8_t alarm_enabled = 1;   // Flag to enable/disable alarm functionality

// Timer variables
volatile uint8_t tick_1s = 0; // Flag set every second by Timer0 interrupt

// ============================================================================
// LCD DISPLAY BUFFERS - Formatted strings for display
// ============================================================================
char time_str[17];    // Buffer for time display string
char date_str[17];    // Buffer for date display string  
char alarm_str[17];   // Buffer for alarm status string

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
// LCD low-level functions
void lcd_send_nibble(uint8_t nibble);  // Send 4 bits to LCD data pins
void lcd_cmd(uint8_t cmd);             // Send command to LCD
void lcd_data(char data);              // Send data character to LCD
void lcd_init(void);                   // Initialize LCD in 4-bit mode
void lcd_puts(const char *s);          // Display string on LCD
void lcd_gotoxy(uint8_t x, uint8_t y); // Set cursor position
void lcd_clear(void);                  // Clear LCD display

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
// TIMER0 INTERRUPT SERVICE ROUTINE
// ============================================================================
// Timer0 generates interrupts every ~13.1ms (with 256 prescaler)
// 76 interrupts = approximately 1 second
// This provides the timebase for the digital clock
void __interrupt() isr(void) {
    static uint8_t count = 0;  // Counter for 1-second timing
    
    if (TMR0IF) {              // Timer0 overflow interrupt flag
        TMR0IF = 0;            // Clear interrupt flag
        TMR0 = 6;              // Reload timer for precise timing
        count++;
        
        // Generate 1-second tick
        if (count >= 76) {
            count = 0;
            tick_1s = 1;       // Set flag for main loop
        }
    }
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
    // TIMER0 CONFIGURATION (1-second timebase generation)
    // ========================================================================
    OPTION_REGbits.T0CS = 0;   // Timer0 source = internal clock (Fosc/4)
    OPTION_REGbits.PSA = 0;    // Prescaler assigned to Timer0
    OPTION_REGbits.PS = 0b111; // Prescaler 1:256
    // Timer0 frequency = 20MHz/4/256 = 19.53kHz
    // Overflow period = 256/19.53kHz = 13.1ms
    
    TMR0 = 6;                  // Preload timer for precise 1-second timing
    TMR0IE = 1;                // Enable Timer0 interrupt
    PEIE = 1;                  // Enable peripheral interrupts
    GIE = 1;                   // Enable global interrupts

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
        // TIME KEEPING (1-second tick processing)
        // ====================================================================
        if (tick_1s) {
            tick_1s = 0;            // Clear 1-second flag
            second++;               // Increment seconds
            
            // Handle time rollover (seconds -> minutes -> hours -> days)
            if (second >= 60) {
                second = 0;
                minute++;
                if (minute >= 60) {
                    minute = 0;
                    hour++;
                    if (hour >= 24) {
                        hour = 0;
                        date++;
                        // Simple date handling (no leap year or month-specific days)
                        if (date > 31) {
                            date = 1;
                            month++;
                            if (month > 12) {
                                month = 1;
                                year = (year + 1) % 100;  // 2-digit year wrap
                            }
                        }
                    }
                }
            }
        }

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
                if (field == 0) hour = (hour + 1) % 24;           // Hours (0-23)
                if (field == 1) minute = (minute + 1) % 60;       // Minutes (0-59)
                if (field == 2) second = (second + 1) % 60;       // Seconds (0-59)
                if (field == 3) date = (date % 31) + 1;           // Date (1-31)
                if (field == 4) month = (month % 12) + 1;         // Month (1-12)
                if (field == 5) year = (year + 1) % 100;          // Year (0-99)
            } else {                // Alarm setting mode
                if (field == 0) alarm_hour = (alarm_hour + 1) % 24;      // Alarm hours
                else if (field == 1) alarm_minute = (alarm_minute + 1) % 60; // Alarm minutes
                else if (field == 2) alarm_enabled = !alarm_enabled;     // Alarm on/off
            }
            while (!BTN_NEXT);      // Wait for button release
            __delay_ms(200);        // Debounce delay
        }

        // ====================================================================
        // ALARM LOGIC
        // ====================================================================
        // Check if alarm should trigger (with protection against 00:00 default)
        if (alarm_enabled &&
            !(alarm_hour == 0 && alarm_minute == 0) &&  // Prevent trigger at default 00:00
            hour == alarm_hour && minute == alarm_minute) {
            alarm_triggered = 1;    // Set alarm active flag
            ALARM_OUT = 1;          // Turn on buzzer
            ALARM_LED = 1;          // Turn on alarm LED
        } else {
            ALARM_OUT = 0;          // Turn off buzzer
            ALARM_LED = 0;          // Turn off alarm LED
        }

        __delay_ms(50);             // Main loop delay for stable operation
    }
}