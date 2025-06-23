# PIC16F877A Digital Alarm Clock with DS3232 RTC

A feature-rich digital alarm clock project implemented on the PIC16F877A microcontroller with external DS3232 RTC module, LCD display and button controls.

## 🎯 Project Overview

This project implements a complete digital alarm clock system using the PIC16F877A microcontroller and DS3232 external RTC module. The system features real-time clock functionality with high accuracy, alarm capability, and a user-friendly interface with LCD display and tactile buttons.

## ✨ Features

- **High-Precision Real-time Clock**: Uses DS3232 RTC module for accurate timekeeping
- **I2C Communication**: Digital communication with DS3232 RTC module
- **Persistent Timekeeping**: DS3232 maintains time even when main system is powered off
- **Date Display**: Shows current date (DD/MM/YYYY) with automatic calendar management
- **Programmable Alarm**: Set custom alarm time with enable/disable functionality stored in DS3232
- **Dual Mode Interface**: Switch between clock setting and alarm setting modes
- **Visual & Audio Feedback**: LED indicator and buzzer for alarm activation
- **Button-based Navigation**: Intuitive 3-button interface for all operations
- **16x4 LCD Display**: Clear, multi-line information display
- **Hardware Alarm**: Alarm functionality handled by DS3232 hardware

## 📋 Hardware Requirements

### Components
- **Microcontroller**: PIC16F877A
- **RTC Module**: DS3232 (I2C interface)
- **Display**: 16x4 Character LCD (HD44780 compatible)
- **Crystal Oscillator**: 20MHz
- **Buttons**: 3 tactile push buttons
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

#### I2C Connections for DS3232 (PORTC)
```
RC3 → DS3232 SCL (I2C Clock)
RC4 → DS3232 SDA (I2C Data)
```

#### Button Connections (PORTB)
```
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
- DS3232 RTC module operates at 3.3V or 5V
- Configuration bits are set for high-speed oscillator mode
- Brown-out reset is enabled for reliable operation

### DS3232 RTC Module
- High-precision temperature-compensated RTC
- I2C interface for communication
- Built-in alarm functionality
- Battery backup capability for continuous operation
- ±2ppm accuracy from 0°C to +40°C

### LCD Interface
- 4-bit mode operation to save I/O pins
- Standard HD44780 control protocol
- Displays 4 lines of 16 characters each

### Input Interface
- Three active-low buttons with internal pull-up resistors enabled
- Software debouncing implemented for reliable button detection
- 200ms debounce delay prevents false triggering

### I2C Communication System
- Software I2C implementation
- Communicates with DS3232 RTC module
- Handles BCD to decimal conversion for time/date values

## 🎮 User Interface

### Button Functions

| Button | Function |
|--------|----------|
| **MODE** | Toggle between Clock Setting and Alarm Setting modes |
| **INC** | Move to next field for editing |
| **NEXT** | Increment current field value |

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

1. **I2C Communication Module**
   - Software I2C implementation
   - Start/stop condition generation
   - Read/write byte functions
   - Communication with DS3232 RTC

2. **DS3232 RTC Driver Functions**
   - Time and date reading/writing
   - Alarm configuration
   - BCD to decimal conversion
   - Status register management

3. **LCD Driver Functions**
   - 4-bit mode communication
   - Command and data transmission
   - Cursor positioning and display control

4. **User Interface Handler**
   - Button debouncing and state detection
   - Mode switching and field navigation
   - Value increment/decrement with rollover

5. **Alarm Logic**
   - Hardware alarm detection from DS3232
   - Audio and visual feedback control
   - Alarm enable/disable management

### Key Features

- **Hardware RTC**: Dedicated DS3232 for accurate timekeeping
- **Modular Design**: Separate functions for each major component
- **I2C Communication**: Reliable digital communication protocol
- **Persistent Storage**: Time and alarm settings stored in DS3232
- **Robust Input Handling**: Comprehensive button debouncing

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
#pragma config CPD = OFF     // EEPROM code protection disabled
#pragma config WRT = OFF     // Flash memory write protection disabled
#pragma config CP = OFF      // Code protection disabled
```

### Compilation Notes
- Set `_XTAL_FREQ` to match your crystal frequency (20MHz default)
- Ensure XC8 compiler optimizations are enabled
- Use appropriate device selection (PIC16F877A)

## 🚀 Getting Started

### 1. Hardware Setup
1. Assemble the circuit according to the pin configuration
2. Connect 20MHz crystal with appropriate capacitors
3. Connect DS3232 RTC module to I2C pins (RC3/RC4)
4. Ensure proper power supply and decoupling capacitors
5. Connect LCD with proper contrast adjustment

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
2. LCD should display current time and date from DS3232
3. Use buttons to navigate and set time/alarm
4. Test alarm functionality

## 🔍 Troubleshooting

### Common Issues

| Problem | Possible Causes | Solutions |
|---------|----------------|-----------|
| LCD not displaying | Power, connections, contrast | Check wiring, adjust contrast pot |
| Time not updating | DS3232 connection, I2C issues | Verify I2C wiring, check DS3232 power |
| Buttons not responding | Pull-ups, debouncing | Enable internal pull-ups, check connections |
| Alarm not working | DS3232 alarm config, output connections | Verify alarm settings, check buzzer/LED |
| I2C communication error | SDA/SCL connections | Check RC3/RC4 connections, verify DS3232 address |

### Debug Tips
- Use debugger/simulator in MPLAB X
- Add temporary LED indicators for testing
- Verify configuration bits are properly set
- Check I2C communication with oscilloscope
- Verify DS3232 power supply and connections

## 📚 Technical Specifications

### Timing Accuracy
- **DS3232 RTC**: ±2ppm accuracy (0°C to +40°C)
- **Crystal**: 20MHz ±20ppm for microcontroller
- **Time Accuracy**: ±1 minute per year (DS3232 dependent)
- **Temperature Compensation**: Automatic in DS3232

### Power Consumption
- **Active Mode**: ~25mA @ 5V (LCD backlight dependent)
- **DS3232**: ~2µA in battery backup mode
- **Sleep Mode**: Not implemented (continuous operation)

### Communication
- **I2C Speed**: ~100kHz (software implementation)
- **DS3232 Address**: 0xD0 (write) / 0xD1 (read)

### Environmental Conditions
- **Operating Temperature**: 0°C to +70°C
- **Storage Temperature**: -40°C to +125°C
- **Humidity**: <85% non-condensing

## 🔮 Future Enhancements

Potential improvements for the project:
- **Battery Backup**: Add coin cell battery to DS3232 for power-off timekeeping
- **Multiple Alarms**: Support for DS3232's second alarm functionality
- **Snooze Function**: Temporary alarm disable with auto-reactivation
- **Temperature Display**: Utilize DS3232's built-in temperature sensor
- **Remote Control**: IR remote for convenient operation
- **Data Logging**: Store alarm history in EEPROM

## 📄 License

This project is open-source and available under the MIT License. Feel free to modify and distribute according to your needs.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests to improve this project.

## 📞 Support

For questions or support regarding this project, please create an issue in the repository or contact the project maintainer.

---

**Project Status**: ✅ Stable and fully functional  
**Last Updated**: 2024  
**Version**: 2.0 (DS3232 Implementation) 