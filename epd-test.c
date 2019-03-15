/* 
 *  This example code is for PDi 3.7" BWR EPD on EXT2 board which is verified by Arduino M0 Pro.
 *  And it sould be able to execute on Arduino IDE supported Board.
 *  Like Arduino Due or Arduino Uno(Need a level shifter 5V -> 3V for EXT2 board)  
 *  For more information about PDi EPD and EXT2 board, please visit 
 *  http://www.pervasivedisplays.com/
 *  http://www.pervasivedisplays.com/kits/ext2_kit
 */

/*

color    rpi      function    epd
-------------------------------------
black    pin 6    GND         pin 20
brown    pin 1    3.3V        pin 1
red      pin 3    bcm2/SDA    pin 15
orange   pin 5    bcm3/SCL    pin 7
yellow   pin 11   bcm17/CS    pin 19
green    pin 13   bcm27/DC    pin 9
blue     pin 15   bcm22/reset pin 10
violet   pin 16   bcm23/busy  pin 8
gray     pin 18   bcm24/pnlon pin 11
white    pin 22   bcm25/css   pin 2
black    pin 39   GND/4wire   pin 17  select 4-wire SPI

*/

#define SDA_PIN 2
#define SCL_PIN 3
#define CS_PIN 17
#define DC_PIN 27
#define RESET_PIN 22
#define BUSY_PIN 23
#define PNLON_PIN 24
#define CSS_PIN 25

#define LOW 0
#define HIGH 1

#define OUTPUT 0
#define INPUT 1

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#include <sys/mman.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <pthread.h>

#include "epd-test.h"

#define MAP_SIZE      0x10000

#define ERROR_JTAG_INIT_FAILED -1
#define ERROR_OK 1

uint32_t bcm2835_peri_base = 0x3F000000;
#define BCM2835_GPIO_BASE	(bcm2835_peri_base + 0x200000) /* GPIO controller */

#define BCM2835_PADS_GPIO_0_27		(bcm2835_peri_base + 0x100000)
#define BCM2835_PADS_GPIO_0_27_OFFSET	(0x2c / 4)

/* GPIO setup macros */
#define MODE_GPIO(g) (*(pio_base+((g)/10))>>(((g)%10)*3) & 7)
#define INP_GPIO(g) do { *(pio_base+((g)/10)) &= ~(7<<(((g)%10)*3)); } while (0)
#define SET_MODE_GPIO(g, m) do { /* clear the mode bits first, then set as necessary */ \
		INP_GPIO(g);						\
		*(pio_base+((g)/10)) |=  ((m)<<(((g)%10)*3)); } while (0)
#define OUT_GPIO(g) SET_MODE_GPIO(g, 1)

#define GPIO_SET (*(pio_base+7))  /* sets   bits which are 1, ignores bits which are 0 */
#define GPIO_CLR (*(pio_base+10)) /* clears bits which are 1, ignores bits which are 0 */
#define GPIO_LEV (*(pio_base+13)) /* current level of the pin */

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static int bcm2835gpio_init(void);
static int bcm2835gpio_quit(void);

/* Transition delay coefficients */
static unsigned int jtag_delay = 1;

void delay( int msecs ) {
  usleep( 1000 * msecs );
}

static void digitalWrite(int pin, int value)
{
  //  uint32_t curval = GPIO_LEV & (1 << pin) ? HIGH : LOW;
  volatile uint32_t lev;

  lev = GPIO_LEV;
  GPIO_SET = 0;
  
  if( value == HIGH ) {
    GPIO_SET = 1 << pin;
  } else {
    GPIO_CLR = 1 << pin;
  }
  
  for (unsigned int i = 0; i < jtag_delay; i++)
    asm volatile ("");
}

static int digitalRead(int pin) {
  // the first read back is unreliable, so do a dummy ready before returning data
  // lev = GPIO_LEV;
  GPIO_SET = 0;
  GPIO_SET = 0;
  GPIO_SET = 0;

  if( GPIO_LEV & 1 << pin )
    return HIGH;
  else
    return LOW;
}

static int bcm2835gpio_init(void)
{
	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		perror("open");
		return ERROR_JTAG_INIT_FAILED;
	}

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	static volatile uint32_t *pads_base;
	pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_PADS_GPIO_0_27);

	if (pads_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* set 4mA drive strength, slew rate limited, hysteresis on */
	pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] = 0x5a000002 + 4;
	
	return ERROR_OK;
}

void pinMode( int pin, int mode ) {

  if( mode == OUTPUT ) {
    OUT_GPIO(pin);
  } else if ( mode == INPUT ) {
    INP_GPIO(pin);
  } else {
    printf( "attempt to set unknown pin mode\n" );
  }
}

extern uint8_t blackBuffer [];
extern uint8_t redBuffer [];
extern uint8_t WBuffer [];

void dummy() {
  int i;
  for (i = 0; i < 120; i++ )  // just manually tweaked
    GPIO_SET = 0;
}
// Software SPI setup
void softwareSpi( uint8_t data ) {
  for ( int i = 0; i < 8; i++ ) {
    if ((( data >> (7 - i) ) & 0x01 ) == 1 ) digitalWrite( SDA_PIN, HIGH );
    else digitalWrite( SDA_PIN, LOW );
    dummy();
    digitalWrite( SCL_PIN, HIGH );
    dummy();
    digitalWrite( SCL_PIN, LOW );
  }
}

// Software SPI protocl setup
void sendIndexData( uint8_t index, const uint8_t *data, uint32_t len ) {
  digitalWrite( DC_PIN, LOW );      //DC Low
  digitalWrite( CS_PIN, LOW );      //CS Low
  softwareSpi( index );
  digitalWrite( CS_PIN, HIGH );     //CS High
  digitalWrite( DC_PIN, HIGH );     //DC High
  digitalWrite( CS_PIN, LOW );      //CS Low
  for ( int i = 0; i < len; i++ ) softwareSpi( data[ i ] );
  digitalWrite( CS_PIN, HIGH );     //CS High
}

//setup function runs once on startup
int main(int argc, char *argv[]) {
  if(argc != 2) {
    printf( "Usage: epd-test <png file>, PNG should be 240x416, black, white, and red\n" );
    return 0;
  }

  read_png_file(argv[1]);
  process_black_plane();
  process_red_plane();

  bcm2835gpio_init();
  
  pinMode( PNLON_PIN, OUTPUT);
  pinMode( SCL_PIN, OUTPUT );
  pinMode( SDA_PIN, OUTPUT );
  pinMode( CS_PIN, OUTPUT );
  pinMode( DC_PIN, OUTPUT );
  pinMode( RESET_PIN, OUTPUT );
  
  digitalWrite( PNLON_PIN, HIGH );    //PANEL_ON# = 1
  pinMode( BUSY_PIN, INPUT );         //All Pins 0
  delay( 5 );                         //Delay 5ms
  digitalWrite( RESET_PIN, HIGH );    //RES# = 1
  delay( 5 );                         //Delay 5ms
  digitalWrite( RESET_PIN, LOW );
  delay( 10 );
  digitalWrite( RESET_PIN, HIGH );
  delay( 5 );
  digitalWrite( CS_PIN, HIGH );       //CS# = 1

  uint8_t data9[] = { 0x0e };         
  sendIndexData( 0x00, data9, 1 );    //Soft-reset
  delay( 5 );
  
  uint8_t data7[] = { 0x19 };   
  sendIndexData( 0xe5, data7, 1 );    //Input Temperature: 25C
  uint8_t data6[] = { 0x02 };         
  sendIndexData( 0xe0, data6, 1 );    //Active Temperature

/*
  uint8_t data1[] = { 0x0f };   
  sendIndexData( 0x00, data1, 1 );    //Panel Setting
  uint8_t data2[] = { 0x17, 0x17, 0x27 };         
  sendIndexData( 0x06, data2, 3 );    //Booster soft start setting
  uint8_t data3[] = { 0x01, 0x90, 0x01, 0x2c };   
  sendIndexData( 0x61, data3, 4 );    //Resolution setting
  uint8_t data4[] = { 0x87 };         
  sendIndexData( 0x50, data4, 1 );    //Vcom and data interval setting
  uint8_t data5[] = { 0x88 };         
  sendIndexData( 0xe3, data5, 1 );    //Power Saving
*/

// Send image data
  printf("2\n");
  sendIndexData( 0x10, blackBuffer, 12480 ); //First frame
  sendIndexData( 0x13, redBuffer, 12480 );   //Second frame

  printf("3\n");
  delay( 50 );
  uint8_t data8[] = { 0x00 };
  sendIndexData( 0x04, data8, 1 );    //Power on
  delay( 5 );
  while( digitalRead( BUSY_PIN ) != HIGH );
  sendIndexData( 0x12, data8, 1 );    //Display Refresh
  delay( 5 );
  while( digitalRead( BUSY_PIN ) != HIGH );
  sendIndexData( 0x02, data8, 1 );    //Turn off DC/DC
  while( digitalRead( BUSY_PIN ) != HIGH );
  digitalWrite( DC_PIN, LOW );
  digitalWrite( CS_PIN, LOW );
  digitalWrite( SDA_PIN, LOW );
  digitalWrite( SCL_PIN, LOW );
  digitalWrite( RESET_PIN, LOW );
  printf("4\n");
}


