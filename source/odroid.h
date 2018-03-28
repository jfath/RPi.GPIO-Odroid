/*
*  Odroid specific code borrowed from Hardkernel's wiringPi port
*/

// License and info from Hardkernel's original file:
/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef ODROID_H_INCLUDED
#define ODROID_H_INCLUDED

/* start wiringPi.h code */

#define	PI_MODEL_UNKNOWN  0
#define	PI_MODEL_A        1
#define	PI_MODEL_B        2
#define	PI_MODEL_BP       3
#define	PI_MODEL_CM       4
#define	PI_MODEL_AP       5
#define	PI_MODEL_ODROIDC  6
#define PI_MODEL_ODROIDXU_34    7
#define	PI_MODEL_ODROIDC2	8

// Failure modes

#define	WPI_FATAL	(1==1)
#define	WPI_ALMOST	(1==2)

/* end wiringPi.h code */


/* start wiringPi.c code */

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

//
// For ODROID-C Board
//
#define ODROIDC_GPIO_MASK (0xFFFFFF80)

#define ODROIDC_PERI_BASE 0xC1100000
#define GPIO_REG_OFFSET   0x8000
#define ODROID_GPIO_BASE  (ODROIDC_PERI_BASE + GPIO_REG_OFFSET)

#define GPIO_PIN_BASE           80
#define GPIOY_PIN_START         80
#define GPIOY_PIN_END           96
#define GPIOX_PIN_START         97
#define GPIOX_PIN_END           118

#define GPIOX_FSEL_REG_OFFSET   0x0C
#define GPIOX_OUTP_REG_OFFSET   0x0D
#define GPIOX_INP_REG_OFFSET    0x0E
#define GPIOX_PUPD_REG_OFFSET   0x3E
#define GPIOX_PUEN_REG_OFFSET   0x4C

#define GPIOY_FSEL_REG_OFFSET   0x0F
#define GPIOY_OUTP_REG_OFFSET   0x10
#define GPIOY_INP_REG_OFFSET    0x11
#define GPIOY_PUPD_REG_OFFSET   0x3D
#define GPIOY_PUEN_REG_OFFSET   0x4B

#define piAinNode0   "/sys/class/saradc/saradc_ch0"
#define piAinNode1   "/sys/class/saradc/saradc_ch1"

static int adcFds [2] = {
    -1, -1,
} ;

//
// For ODROID-C2 Board
//
#define ODROIDC2_GPIO_MASK		(0xFFFFFF00)
#define ODROIDC2_GPIO_BASE		0xC8834000

#define C2_GPIO_PIN_BASE           136
#define C2_GPIOY_PIN_START         (C2_GPIO_PIN_BASE + 75)
#define C2_GPIOY_PIN_END           (C2_GPIO_PIN_BASE + 91)
#define C2_GPIOX_PIN_START         (C2_GPIO_PIN_BASE + 92)
#define C2_GPIOX_PIN_END           (C2_GPIO_PIN_BASE + 114)

#define C2_GPIOX_FSEL_REG_OFFSET   0x118
#define C2_GPIOX_OUTP_REG_OFFSET   0x119
#define C2_GPIOX_INP_REG_OFFSET    0x11A
#define C2_GPIOX_PUPD_REG_OFFSET   0x13E
#define C2_GPIOX_PUEN_REG_OFFSET   0x14C

#define C2_GPIOY_FSEL_REG_OFFSET   0x10F
#define C2_GPIOY_OUTP_REG_OFFSET   0x110
#define C2_GPIOY_INP_REG_OFFSET    0x111
#define C2_GPIOY_PUPD_REG_OFFSET   0x13B
#define C2_GPIOY_PUEN_REG_OFFSET   0x149

#define C2_piAinNode0   "/sys/class/saradc/ch0"
#define C2_piAinNode1   "/sys/class/saradc/ch1"

//
// For ODROID-XU3/4 Board
//
#define ODROIDXU_GPIO_MASK  (0xFFFFFF00)

#define ODROIDXU_GPX_BASE   0x13400000  // GPX0,1,2,3
#define ODROIDXU_GPA_BASE   0x14010000  // GPA0,1,2, GPB0,1,2,3,4

#define GPIO_X1_START       16
#define GPIO_X1_CON_OFFSET  0x0C20
#define GPIO_X1_DAT_OFFSET  0x0C24
#define GPIO_X1_PUD_OFFSET  0x0C28
#define GPIO_X1_END         23

#define GPIO_X2_START       24
#define GPIO_X2_CON_OFFSET  0x0C40
#define GPIO_X2_DAT_OFFSET  0x0C44
#define GPIO_X2_PUD_OFFSET  0x0C48
#define GPIO_X2_END         31

#define GPIO_X3_START       32
#define GPIO_X3_CON_OFFSET  0x0C60
#define GPIO_X3_DAT_OFFSET  0x0C64
#define GPIO_X3_PUD_OFFSET  0x0C68
#define GPIO_X3_END         39

#define GPIO_A0_START       171
#define GPIO_A0_CON_OFFSET  0x0000
#define GPIO_A0_DAT_OFFSET  0x0004
#define GPIO_A0_PUD_OFFSET  0x0008
#define GPIO_A0_END         178

#define GPIO_A2_START       185
#define GPIO_A2_CON_OFFSET  0x0040
#define GPIO_A2_DAT_OFFSET  0x0044
#define GPIO_A2_PUD_OFFSET  0x0048
#define GPIO_A2_END         192

#define GPIO_B3_START       207
#define GPIO_B3_CON_OFFSET  0x00C0
#define GPIO_B3_DAT_OFFSET  0x00C4
#define GPIO_B3_PUD_OFFSET  0x00C8
#define GPIO_B3_END         214


#ifdef DEFINE_ODROID_VARS

//From c_gpio.c and c_gpio.h
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)

#define INPUT  1 // is really 0 for control register!
#define OUTPUT 0 // is really 1 for control register!
#define ALT0   4

#define HIGH 1
#define LOW  0

#define PUD_OFF  0
#define PUD_DOWN 1
#define PUD_UP   2
//End from c_gpio.c and c_gpio.h

//From common.h
#define MAXPINCOUNT 40
extern const int (*pin_to_gpio)[MAXPINCOUNT+1];
//End from common.h

int wiringPiReturnCodes = FALSE ;

static volatile uint32_t *gpio, *gpio1;

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.
//static int *pinToGpio ;
//static int pin_array_count;

// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

//static int *physToGpio ;

static char *piAinNode0_xu;
static char *piAinNode1_xu;

static int sysFdData [64] = {
    -1, -1, -1, -1, -1, -1, -1, -1, // 0...7
    -1, -1, -1, -1, -1, -1, -1, -1, // 8...15
    -1, -1, -1, -1, -1, -1, -1, -1, // 16...23
    -1, -1, -1, -1, -1, -1, -1, -1, // 24...31
    -1, -1, -1, -1, -1, -1, -1, -1, // 32...39
    -1, -1, -1, -1, -1, -1, -1, -1, // 40...47
    -1, -1, -1, -1, -1, -1, -1, -1, // 48...55
    -1, -1, -1, -1, -1, -1, -1, -1, // 56...63
};

static int sysFdIrqType [64] = {
    -1, -1, -1, -1, -1, -1, -1, -1, // 0...7
    -1, -1, -1, -1, -1, -1, -1, -1, // 8...15
    -1, -1, -1, -1, -1, -1, -1, -1, // 16...23
    -1, -1, -1, -1, -1, -1, -1, -1, // 24...31
    -1, -1, -1, -1, -1, -1, -1, -1, // 32...39
    -1, -1, -1, -1, -1, -1, -1, -1, // 40...47
    -1, -1, -1, -1, -1, -1, -1, -1, // 48...55
    -1, -1, -1, -1, -1, -1, -1, -1, // 56...63
};


//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROID_GPIO pin
//
static int pinToGpioOdroidC [64] = {
    88,  87, 116, 115, 104, 102, 103,  83, // 0..7
    -1,  -1, 117, 118, 107, 106, 105,  -1, // 8..16
    -1,  -1,  -1,  -1,  -1, 101, 100, 108, // 16..23
    97,  -1,  99,  98,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROID_GPIO pin
//
//!!!Odroid - don't know why wiringPi code uses array size of 64 instead of 41.
//Also need to access from other files, so changed static int to const int
/*odroid static int */const int physToGpioOdroidC [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
  83,  -1,
  -1,  -1,
  88,  87,
 116,  -1,
 115, 104,
  -1, 102,
 107,  -1,
 106, 103,
 105, 117,
  -1, 118,	// 25, 26

  -1,  -1,
 101,  -1,
 100,  99,
 108,  -1,
  97,  98,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;

//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROIDC2_GPIO pin
//
static int pinToGpioOdroidC2_Rev1_1 [64] = {
   247, 238, 239, 237, 236, 233, 231, 249, // 0..7
    -1,  -1, 229, 225, 235, 232, 230,  -1, // 8..15
    -1,  -1,  -1,  -1,  -1, 228, 219, 234, // 16..23
   214,  -1, 224, 218,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};


static int pinToGpioOdroidC2_Rev1_0 [64] = {
   219, 218, 247,  -1, 235, 233, 234, 214, // 0..7
    -1,  -1, 248, 249, 238, 237, 236,  -1, // 8..15
    -1,  -1,  -1,  -1,  -1, 232, 231, 239, // 16..23
   228,  -1, 230, 229,  -1,  -1,  -1,  -1, // 24..31
// Padding:
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROIDC2_GPIO pin
//
/*odroid static int*/const int physToGpioOdroidC2_Rev1_1 [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
 249,  -1,
  -1,  -1,
 247, 238,
 239,  -1,
 237, 236,
  -1, 233,
 235,  -1,
 232, 231,
 230, 229,
  -1, 225,	// 25, 26

  -1,  -1,
 228,  -1,
 219, 224,
 234,  -1,
 214, 218,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;


/*odroid static int*/const int physToGpioOdroidC2_Rev1_0 [64] =
{
  -1,       // 0
  -1,  -1,	// 1, 2
  -1,  -1,
  -1,  -1,
 214,  -1,
  -1,  -1,
 219, 218,
 247,  -1,
  -1, 235,
  -1, 233,
 238,  -1,
 237, 234,
 236, 248,
  -1, 249,	// 25, 26

  -1,  -1,
 232,  -1,
 231, 230,
 239,  -1,
 228, 229,
  -1,  -1,
  -1,  -1, // 39, 40

// Not used
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1,
} ;

//
// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the ODROIDXU_GPIO pin
//
static int pinToGpioOdroidXU [64] = {
   174, 173,    //  0 |  1 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  22,    //  2 |  3 : GPX1.5, GPX1.6
    19,  23,    //  4 |  5 : GPX1.3, GPX1.7
    24,  18,    //  6 |  7 : GPX2.0, GPX1.2

   209, 210,    //  8 |  9 : GPB3.2(I2C_1.SDA), GPB3.3(I2C_1.SCL)
   190,  25,    // 10 | 11 : GPA2.5(SPI_1.CSN), GPX2.1
   192, 191,    // 12 | 13 : GPA2.7(SPI_1.MOSI), GPA2.6(SPI_1.MISO)
   189, 172,    // 14 | 15 : GPA2.4(SPI_1.SCLK), GPA0.1(UART_0.TXD)
   171,  -1,    // 16 | 17 : GPA0.0(UART_0.RXD),
    -1,  -1,    // 18 | 19
    -1,  28,    // 20 | 21 :  , GPX2.4
    30,  31,    // 22 | 23 : GPX2.6, GPX2.7
    -1,  -1,    // 24 | 25   PWR_ON(INPUT), ADC_0.AIN0
    29,  33,    // 26 | 27 : GPX2.5, GPX3.1
    -1,  -1,    // 28 | 29 : REF1.8V OUT, ADC_0.AIN3
   187, 188,    // 30 | 31 : GPA2.2(I2C_5.SDA), GPA2.3(I2C_5.SCL)

    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

//
// physToGpio:
//	Take a physical pin (1 through 40) and re-map it to the ODROIDXU_GPIO pin
//
/*odroid static int*/const int physToGpioOdroidXU [64] =
{
    -1,         //  0
    -1,  -1,	//  1 |  2 : 3.3V, 5.0V
   209,  -1,    //  3 |  4 : GPB3.2(I2C_1.SDA), 5.0V
   210,  -1,    //  5 |  6 : GPB3.3(I2C_1.SCL), GND
    18, 172,    //  7 |  8 : GPX1.2, GPA0.1(UART_0.TXD)
    -1, 171,    //  9 | 10 : GND, GPA0.0(UART_0.RXD)
   174, 173,    // 11 | 12 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
    21,  -1,    // 13 | 14 : GPX1.5, GND
    22,  19,    // 15 | 16 : GPX1.6, GPX1.3
    -1,  23,    // 17 | 18 : 3.3V, GPX1.7
   192,  -1,    // 19 | 20 : GPA2.7(SPI_1.MOSI), GND
   191,  24,    // 21 | 22 : GPA2.6(SPI_1.MISO), GPX2.0
   189, 190,    // 23 | 24 : GPA2.4(SPI_1.SCLK), GPA2.5(SPI_1.CSN)
    -1,  25,    // 25 | 26 : GND, GPX2.1
   187, 188,    // 27 | 28 : GPA2.2(I2C_5.SDA), GPA2.4(I2C_5.SCL)
    28,  -1,    // 29 | 30 : GPX2.4, GND
    30,  29,    // 31 | 32 : GPX2.6, GPX2.5
    31,  -1,    // 33 | 34 : GPX2.7, GND
    -1,  33,    // 35 | 36 : PWR_ON(INPUT), GPX3.1
    -1,  -1,    // 37 | 38 : ADC_0.AIN0, 1.8V REF OUT
    -1,  -1,    // 39 | 40 : GND, AADC_0.AIN3

    // Not used
    -1, -1, -1, -1, -1, -1, -1, -1, // 41...48
    -1, -1, -1, -1, -1, -1, -1, -1, // 49...56
    -1, -1, -1, -1, -1, -1, -1      // 57...63
} ;

/* end wiringPi.c code */


/* Non-static add extern definition below */
int odroid_found;
int  piModel;

const int bcmToOGpioOdroidC[64] = {	// BCM ModE
     -1,  -1,  -1,  -1,  83, 101, 100, 118, // 0..7
    117, 106, 107, 105,  99, 108,  -1,  -1, // 8..15
     98,  88,  87,  97,  -1,  -1, 115, 104, // 16..23
    102, 103,  -1, 116,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidC2[64] = {	// BCM ModE
     -1,  -1,  -1,  -1, 249, 228, 219, 225, // 0..7
    229, 232, 235, 230, 224, 234,  -1,  -1, // 8..15
    218, 247, 238, 214,  -1,  -1, 237, 236, // 16..23
    233, 231,  -1, 239,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioOdroidXU[64] = {	// BCM ModE
     -1,  -1, 209, 210,  18,  28,  30,  25, // 0..7
    190, 191, 192, 189,  29,  31, 172, 171, // 8..15
     33, 174, 173,  -1,  -1,  -1,  22,  19, // 16..23
     23,  24,  -1,  21,  -1,  -1,  -1,  -1, // 24..31
// Padding:
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 32..39
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 40..47
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, // 48..55
     -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1  // 56..63
};

const int bcmToOGpioRPi[64] = {	// BCM ModE
      0,   1,   2,   3,   4,   5,   6,   7, // 0..7
      8,   9,  10,  11,  12,  13,  14,  15, // 8..15
     16,  17,  16,  19,  20,  21,  22,  23, // 16..23
     24,  25,  26,  27,  28,  29,  30,  31, // 24..31
// Padding:
     32,  33,  34,  35,  36,  37,  38,  39, // 32..39
     40,  41,  42,  43,  44,  45,  46,  47, // 40..47
     48,  49,  50,  51,  52,  53,  54,  55, // 48..55
     56,  57,  58,  59,  60,  61,  62,  63  // 56..63
};

const int (*bcm_to_odroidgpio)[64];


#else /* DEFINE_ODROID_VARS */

extern int odroid_found;
extern int  piModel;
extern const int physToGpioOdroidC[64];
extern const int physToGpioOdroidC2_Rev1_1[64];
extern const int physToGpioOdroidXU[64];
extern const int bcmToOGpioOdroidC[64];
extern const int bcmToOGpioOdroidC2[64];
extern const int bcmToOGpioOdroidXU[64];
extern const int bcmToOGpioRPi[64];
extern const int (*bcm_to_odroidgpio)[64];

int wiringPiSetupOdroid (void);
void wiringPiCleanupOdroid (void);
void pinModeOdroid (int pin, int mode);
void pullUpDnControlOdroid (int pin, int pud);
int digitalReadOdroid (int pin);
void digitalWriteOdroid (int pin, int value);
int analogReadOdroid (int pin);
void analogWriteOdroid (int pin, int value);
int pinGetModeOdroid (int pin);
void setInfoOdroid(char *hardware, void *vinfo);
void setMappingPtrsOdroid(void);

#endif /* DEFINE_ODROID_VARS */


#endif /* ODROID_H_INCLUDED */