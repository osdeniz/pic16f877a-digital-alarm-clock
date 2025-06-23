# PIC16F877A Digital Alarm Clock

A feature-rich digital alarm clock project implemented on the PIC16F877A microcontroller with LCD display and button controls.

## 🎯 Project Overview

This project implements a complete digital alarm clock system using the PIC16F877A microcontroller. The system features real-time clock functionality, alarm capability, and a user-friendly interface with LCD display and tactile buttons.

## ✨ Features

- **Real-time Clock**: Displays current time (HH:MM:SS) with automatic rollover
- **Date Display**: Shows current date (DD/MM/YYYY) with basic date tracking
- **Programmable Alarm**: Set custom alarm time with enable/disable functionality
- **Dual Mode Interface**: Switch between clock setting and alarm setting modes
- **Visual & Audio Feedback**: LED indicator and buzzer for alarm activation
- **Button-based Navigation**: Intuitive button interface for all operations
- **16x4 LCD Display**: Clear, multi-line information display

## 📋 Hardware Requirements

### Components
- **Microcontroller**: PIC16F877A
- **Display**: 16x4 Character LCD (HD44780 compatible)
- **Crystal Oscillator**: 20MHz
- **Buttons**: 4 tactile push buttons
- **Output Devices**: 
  - Buzzer for alarm sound
  - LED for visual alarm indication
- **Power Supply**: 5V DC regulated

### Pin Configuration

#### LCD Connections (PORTD)
```
RD0 → LCD RS (Register Select)
RD1 → LCD RW (Read/Write)
RD2 → LCD EN (Enable)
RD3 → LCD D4 (Data bit 4)
RD4 → LCD D5 (Data bit 5)
RD5 → LCD D6 (Data bit 6)
RD6 → LCD D7 (Data bit 7)
```

#### Button Connections (PORTB)
```
RB0 → Decrement Button (with pull-up)
RB1 → Next/Select Button (with pull-up)
RB2 → Increment Button (with pull-up)
RB3 → Mode Toggle Button (with pull-up)
```

#### Output Connections
```
RB4 → Buzzer (Alarm Audio Output)
RC7 → LED (Alarm Visual Indicator)
```

## 🔧 Circuit Description

### Power and Clock
- The PIC16F877A operates at 5V with a 20MHz crystal oscillator
- Configuration bits are set for high-speed oscillator mode
- Brown-out reset is enabled for reliable operation

### LCD Interface
- 4-bit mode operation to save I/O pins
- Standard HD44780 control protocol
- Displays 4 lines of 16 characters each

### Input Interface
- Four active-low buttons with internal pull-up resistors enabled
- Software debouncing implemented for reliable button detection
- 200ms debounce delay prevents false triggering

### Timer System
- Timer0 configured with 1:256 prescaler
- Generates precise 1-second timebase for clock operation
- Interrupt-driven timekeeping for accurate operation

## 🎮 User Interface

### Button Functions

| Button | Function |
|--------|----------|
| **MODE** | Toggle between Clock Setting and Alarm Setting modes |
| **INC** | Move to next field for editing |
| **NEXT** | Increment current field value |
| **DEC** | Decrement current field value |

### Display Layout
```
Line 0: TIME: HH:MM:SS
Line 1: DATE: DD/MM/YYYY
Line 2: ALARM: HH:MM [ON/OFF]
Line 3: SETTING: CLOCK/ALARM
```

### Operating Modes

#### Clock Setting Mode
Navigate through 6 fields:
1. **Hours** (00-23)
2. **Minutes** (00-59)
3. **Seconds** (00-59)
4. **Date** (01-31)
5. **Month** (01-12)
6. **Year** (00-99, representing 20xx)

#### Alarm Setting Mode
Navigate through 3 fields:
1. **Alarm Hours** (00-23)
2. **Alarm Minutes** (00-59)
3. **Alarm Enable/Disable** (ON/OFF)

## ⚙️ Software Architecture

### Main Components

1. **Interrupt Service Routine (ISR)**
   - Timer0 overflow handler
   - Generates 1-second timing tick
   - Non-blocking timekeeping

2. **LCD Driver Functions**
   - 4-bit mode communication
   - Command and data transmission
   - Cursor positioning and display control

3. **User Interface Handler**
   - Button debouncing and state detection
   - Mode switching and field navigation
   - Value increment/decrement with rollover

4. **Alarm Logic**
   - Time comparison for alarm triggering
   - Audio and visual feedback control
   - Protection against false triggers

### Key Features

- **Modular Design**: Separate functions for each major component
- **Interrupt-Driven**: Non-blocking timekeeping operation
- **Robust Input Handling**: Comprehensive button debouncing
- **Safe Operation**: Protection against invalid states

## 📖 Programming Guide

### Development Environment
- **IDE**: MPLAB X IDE
- **Compiler**: XC8 C Compiler
- **Programmer**: PICkit 3/4 or compatible

### Configuration Settings
```c
#pragma config FOSC = HS     // High-speed crystal
#pragma config WDTE = OFF    // Watchdog disabled
#pragma config PWRTE = OFF   // Power-up timer disabled
#pragma config BOREN = ON    // Brown-out reset enabled
#pragma config LVP = OFF     // Low voltage programming disabled
```

### Compilation Notes
- Set `_XTAL_FREQ` to match your crystal frequency (20MHz default)
- Ensure XC8 compiler optimizations are enabled
- Use appropriate device selection (PIC16F877A)

## 🚀 Getting Started

### 1. Hardware Setup
1. Assemble the circuit according to the pin configuration
2. Connect 20MHz crystal with appropriate capacitors
3. Ensure proper power supply and decoupling capacitors
4. Connect LCD with proper contrast adjustment

### 2. Software Setup
1. Install MPLAB X IDE and XC8 compiler
2. Create new project for PIC16F877A
3. Import the `main.c` source file
4. Configure project settings and programmer

### 3. Programming
1. Compile the project (check for errors)
2. Connect your programmer to the target circuit
3. Program the PIC16F877A with the generated hex file
4. Verify programming success

### 4. Operation
1. Power on the system
2. LCD should display current time and date
3. Use buttons to navigate and set time/alarm
4. Test alarm functionality

## 🔍 Troubleshooting

### Common Issues

| Problem | Possible Causes | Solutions |
|---------|----------------|-----------|
| LCD not displaying | Power, connections, contrast | Check wiring, adjust contrast pot |
| Incorrect time | Crystal frequency, Timer0 config | Verify 20MHz crystal, check config bits |
| Buttons not responding | Pull-ups, debouncing | Enable internal pull-ups, check connections |
| Alarm not working | Logic error, output connections | Verify alarm time setting, check buzzer/LED |

### Debug Tips
- Use debugger/simulator in MPLAB X
- Add temporary LED indicators for testing
- Verify configuration bits are properly set
- Check crystal oscillator startup

## 📚 Technical Specifications

### Timing Accuracy
- **Crystal**: 20MHz ±20ppm
- **Timer Resolution**: ~13.1ms per Timer0 overflow
- **Time Accuracy**: ±1 second per day (crystal dependent)

### Power Consumption
- **Active Mode**: ~20mA @ 5V (LCD backlight dependent)
- **Sleep Mode**: Not implemented (continuous operation)

### Environmental Conditions
- **Operating Temperature**: 0°C to +70°C
- **Storage Temperature**: -40°C to +125°C
- **Humidity**: <85% non-condensing

## 🔮 Future Enhancements

Potential improvements for the project:
- **DS3231 RTC Integration**: Add external RTC for better accuracy
- **Battery Backup**: Maintain time during power outages
- **Multiple Alarms**: Support for several independent alarms
- **Snooze Function**: Temporary alarm disable with auto-reactivation
- **Temperature Display**: Add DS18B20 temperature sensor
- **Remote Control**: IR remote for convenient operation

## 📄 License

This project is open-source and available under the MIT License. Feel free to modify and distribute according to your needs.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests to improve this project.

## 📞 Support

For questions or support regarding this project, please create an issue in the repository or contact the project maintainer.

---

**Project Status**: ✅ Stable and fully functional  
**Last Updated**: 2024  
**Version**: 1.0 