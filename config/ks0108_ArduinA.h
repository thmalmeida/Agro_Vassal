/*
 * ks0108_Arduino.h - User specific configuration for Arduino GLCD library
 *
 * Use this file to set io pins
 * This version is for a standard ks0108 display
 * connected using the default Arduino wiring
 *
*/

#ifndef GLCD_PIN_CONFIG_H
#define GLCD_PIN_CONFIG_H

/*
 * define name for pin configuration
 */
#define glcd_PinConfigName "ks0108-Arduino"

/*********************************************************/
/*  Configuration for assigning LCD bits to Arduino Pins */
/*********************************************************/
/* Data pin definitions
 */
#define glcdData0Pin        46
#define glcdData1Pin        47
#define glcdData2Pin        48
#define glcdData3Pin        49
#define glcdData4Pin        50
#define glcdData5Pin        51
#define glcdData6Pin        52
#define glcdData7Pin        53

/* Arduino pins used for Commands
 * default assignment uses the first five analog pins
 */

#define glcdCSEL1        31
#define glcdCSEL2        32

#if NBR_CHIP_SELECT_PINS > 2
#define glcdCSEL3         3   // third chip select if needed
#endif

#if NBR_CHIP_SELECT_PINS > 3
#define glcdCSEL4         2   // fourth chip select if needed
#endif

#define glcdEN           35
#define glcdRW           34
#define glcdDI           33
// Reset Bit  - uncomment the next line if reset is connected to an output pin
#define glcdRES			44    // Reset Bit

#endif //GLCD_PIN_CONFIG_H
