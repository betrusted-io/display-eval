#define SCLK_PIN 11
#define SI_PIN   10
#define SCS_PIN  8
#define EXTCOM_PIN 24
#define DISP_PIN 23

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
#include <termios.h>

#include <sys/mman.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <pthread.h>

#include "memlcd-test.h"

uint8_t blackBuffer [COL_SIZE][ROW_SIZE];

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

void dummy() {
  int i;
  for (i = 0; i < 120; i++ )  // just manually tweaked
    GPIO_SET = 0;
}

static inline pulseClk(void) {
  dummy();
  digitalWrite( SCLK_PIN, HIGH );
  dummy();
  digitalWrite( SCLK_PIN, LOW );
}

static int dbglimit = 0;

void sendRowData( uint32_t gateline ) {
  int i, j;
  int g;
  uint8_t d;
  
  digitalWrite( SCS_PIN, HIGH );

  // mode selection, 3ck + 3ck DMY
  digitalWrite( SI_PIN, HIGH ); // M0 = update mode
  pulseClk();
  digitalWrite( SI_PIN, LOW );  // M1 = X
  pulseClk();
  digitalWrite( SI_PIN, LOW );  // M2 = not all clear
  pulseClk();

  pulseClk();
  pulseClk();
  pulseClk();

  // gate address period, 10 clocks
  g = gateline + 1;
  for( i = 0; i < 10; i ++ ) {
    if( g & 1 )
      digitalWrite( SI_PIN, HIGH );
    else
      digitalWrite( SI_PIN, LOW );
    pulseClk();
    g >>= 1;
  }
    
  // data write period, 366 clocks
  for( i = 0; i < COL_SIZE; i++ ) {
    d = blackBuffer[i][gateline];
    if( d == 1 ) {
      digitalWrite( SI_PIN, HIGH );
    } else {
      digitalWrite( SI_PIN, LOW );
    }
    pulseClk();
  }

  // data transfer period, 16 clocks
  digitalWrite( SI_PIN, LOW );
  for( i = 0; i < 16; i ++ ) {
    pulseClk();
  }
  
  digitalWrite( SCS_PIN, LOW );
}

//setup function runs once on startup
int main(int argc, char *argv[]) {
  int i;
  int time;
  struct termios old_tio, new_tio;
  unsigned char c;

  
  if(argc != 2) {
    printf( "Usage: memlcd-test <png file>, PNG should be 240x416, black, white, and red\n" );
    return 0;
  }

  read_png_file(argv[1]);
  process_black_plane();

  bcm2835gpio_init();
  
  pinMode( SCLK_PIN, OUTPUT);
  pinMode( SI_PIN, OUTPUT );
  pinMode( SCS_PIN, OUTPUT );
  pinMode( EXTCOM_PIN, OUTPUT );
  pinMode( DISP_PIN, OUTPUT );

  // init all pins to low
  digitalWrite( SCS_PIN, LOW );
  digitalWrite( SCLK_PIN, LOW );
  digitalWrite( SI_PIN, LOW );
  digitalWrite( EXTCOM_PIN, LOW );
  digitalWrite( DISP_PIN, LOW );

  /* get the terminal settings for stdin */
  tcgetattr(STDIN_FILENO,&old_tio);

  /* we want to keep the old setting to restore them a the end */
  new_tio=old_tio;

  /* disable canonical mode (buffered i/o) and local echo */
  new_tio.c_lflag &=(~ICANON & ~ECHO);

  /* set the new settings immediately */
  tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

  printf( "hit q to exit.\n" );
  time = 0;
  do {
    printf( "sending data %d ", time );
    for( i = 0; i < ROW_SIZE; i++ ) {
      sendRowData(i);
    }
    digitalWrite( DISP_PIN, HIGH );

    printf( " done. wait 1s\n" );
    usleep(400000);
    digitalWrite( EXTCOM_PIN, HIGH );
    usleep(500000);
    digitalWrite( EXTCOM_PIN, LOW );
    time++;
    c=getchar();
  } while( c != 'q' );

  /* restore the former settings */
  tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
  
  pinMode( SCLK_PIN, OUTPUT);
  pinMode( SI_PIN, OUTPUT );
  digitalWrite( SCS_PIN, LOW );
  digitalWrite( EXTCOM_PIN, LOW );
  digitalWrite( DISP_PIN, LOW );

  return 0;
}


