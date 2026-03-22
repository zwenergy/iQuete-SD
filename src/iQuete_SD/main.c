
#include <stdio.h>
#include "pico/stdlib.h"
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"

#include "hardware/vreg.h"

// LED.
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// Snoop commands.
#include "get_input.pio.h"


// File system stuff.
#define FILETABLE_START_BLOCK 0xFF0
#define FILETABLE_START_OFFSET 0x2000
#define FILETABLE_LEN 8180
#define FILESTRUCT_SIZE 20
#define FILETABLE_SEQ_OFFSET (0x3FF4 + 0x4)
#define FILETABLE_MAX_FILES 409
#define FILETABLE_MAGIC_OFFSET 0x3FF4
#define FILETABLE_CHECKSUM_OFFSET (0x3FF4 + 0xA)

#define FILE_TARGET_BLOCK 1000


#define LED_INT 16
#define LED_RED 50
#define LED_GREEN 50
#define LED_BLUE 150
#define LED_PIO pio1

/*

This file should be tailored to match the hardware design.

See
https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration

*/

#include "hw_config.h"

/* SDIO Interface */
static sd_sdio_if_t sdio_if = {
    /*
    Pins CLK_gpio, D1_gpio, D2_gpio, and D3_gpio are at offsets from pin D0_gpio.
    The offsets are determined by sd_driver\SDIO\rp2040_sdio.pio.
        CLK_gpio = (D0_gpio + SDIO_CLK_PIN_D0_OFFSET) % 32;
        As of this writing, SDIO_CLK_PIN_D0_OFFSET is 30,
            which is -2 in mod32 arithmetic, so:
        CLK_gpio = D0_gpio +9.
        D1_gpio = D0_gpio + 1;
        D2_gpio = D0_gpio + 2;
        D3_gpio = D0_gpio + 3;
    */
    .CMD_gpio = 12,
    .D0_gpio = 8,
    .baud_rate = 125 * 1000 * 1000 / 6  // 20833333 Hz
};

#define BUS_SDA 18
#define BUS_SCL 19
#define DET 20

#define IO7 0
#define IO6 1
#define IO5 2
#define IO4 3
#define IO3 4
#define IO2 5
#define IO1 6
#define IO0 7
#define nWP 13
#define nWE 14
#define ALE 15
#define CLE 26
#define nCE 27
#define nRE 28
#define RB 29

#define CMD_MASK (1<<CLE)
#define ADDR_MASK (1<<ALE)

#define BTN0 23
#define BTN1 24


#define SWITCH0_ADDR 0b1001000
#define SWITCH1_ADDR 0b1001010

#define SLEEPUS 2

#define IOMASK 0b11111111
#define READ1 0x00
#define BLOCK_ERASE_SETUP_CMD 0x60
#define ERASE_CONFIRM_CMD 0xD0
#define MULTIPLANE_STATUS_CMD 0x71
#define STATUS_CMD 0x70
#define SEQ_DATA_INPUT_CMD 0x80
#define DUMMY_PROGRAM_CMD 0x11
#define PROGRAM_CONFIRM_CMD 0x10

#define BLOCK_ERASE_TIME_MAX_US 3000
#define DUMMY_BUSY_TIME_US 11
#define PROG_TIME_MAX_US 500

#define NOP __asm volatile ("nop")
// Should be at least 50 ns
#define PAUSE NOP;\
              NOP;\
              NOP;\
              NOP;\
              NOP;\
              NOP;\
              NOP;\
              NOP
#define PAGEREAD_US 12
#define BYTES_PER_PAGE 512
#define SPARE_BYTES_PER_PAGE 16
#define PAGES_PER_BLOCK 32
#define MAX_BLOCKS 4096
#define NR_PLANES 4

#define FAT_ENTRIES 4096
// 16b nodes.
#define FAT_LEN ( FAT_ENTRIES * 2 )
#define FAT_BLOCK_OFFSET 0xFF0
#define NR_FAT_BLOCKS 16

// SD card more stuff.
#define MAX_SD_ROMS 100

// File struct.
typedef struct {
  char fileName[8];
  char fileExt[3];
  uint8_t valid;
  uint16_t startBlock;
  uint8_t padding[2];
  uint32_t fileSize;
} bbfsFile;

// Tmp. compare struct.
typedef struct {
  char fileName[8];
  char fileExt[3];
  uint32_t startBlock;
  uint32_t fileSize;
} dummyFile;

// Debug manual.
dummyFile romList[ MAX_SD_ROMS ];
unsigned romListLen = 0;


unsigned char reverse_byte(unsigned char x)
{
  static const unsigned char table[] = {
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
  };
  return table[x];
}


// ECC stuff.

// Taken from iQue stuff: https://github.com/emoose/iQueTool/blob/master/iQueTool/Structs/iQueBlockSpare.cs
// ECC algo ported from https://github.com/TheBlueMatt/u-boot/blob/master/drivers/mtd/nand/nand_ecc.c
const uint8_t EccPrecalcTable[] =
{
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
    0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
    0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
    0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
    0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
    0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
    0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
    0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00
};

// Taken from iQue stuff: https://github.com/emoose/iQueTool/blob/master/iQueTool/Structs/iQueBlockSpare.cs
// Calcs ECC for a 256-byte block
uint32_t Calculate256Ecc( uint8_t* pageData, int offset )
{
    uint8_t idx, reg1, reg2, reg3, tmp1, tmp2;
    int i;

    /* Initialize variables */
    reg1 = 0;
    reg2 = 0;
    reg3 = 0;

    /* Build up column parity */
    for (i = 0; i < 256; i++)
    {
        /* Get CP0 - CP5 from table */
        idx = EccPrecalcTable[ pageData[ i + offset ] ];
        reg1 ^= (idx & 0x3f);

        /* All bit XOR = 1 ? */
        if ((idx & 0x40) == 0x40)
        {
            reg3 ^= (uint8_t) i;
            reg2 ^= (uint8_t)~((uint8_t)i);
        }
    }

    /* Create non-inverted ECC code from line parity */
    tmp1 = ((reg3 & 0x80) >> 0); /* B7 -> B7 */
    tmp1 |= ((reg2 & 0x80) >> 1); /* B7 -> B6 */
    tmp1 |= ((reg3 & 0x40) >> 1); /* B6 -> B5 */
    tmp1 |= ((reg2 & 0x40) >> 2); /* B6 -> B4 */
    tmp1 |= ((reg3 & 0x20) >> 2); /* B5 -> B3 */
    tmp1 |= ((reg2 & 0x20) >> 3); /* B5 -> B2 */
    tmp1 |= ((reg3 & 0x10) >> 3); /* B4 -> B1 */
    tmp1 |= ((reg2 & 0x10) >> 4); /* B4 -> B0 */

    tmp2 = ((reg3 & 0x08) << 4); /* B3 -> B7 */
    tmp2 |= ((reg2 & 0x08) << 3); /* B3 -> B6 */
    tmp2 |= ((reg3 & 0x04) << 3); /* B2 -> B5 */
    tmp2 |= ((reg2 & 0x04) << 2); /* B2 -> B4 */
    tmp2 |= ((reg3 & 0x02) << 2); /* B1 -> B3 */
    tmp2 |= ((reg2 & 0x02) << 1); /* B1 -> B2 */
    tmp2 |= ((reg3 & 0x01) << 1); /* B0 -> B1 */
    tmp2 |= ((reg2 & 0x01) << 0); /* B7 -> B0 */

    /* Calculate final ECC code */
    uint8_t ecc0 = ~tmp2;
    uint8_t ecc1 = ~tmp1;
    uint8_t ecc2 = (((~reg1) << 2) | 0x03);
    
    uint32_t ecc = ecc0 | ( ecc1 << 8 ) | ( ecc2 << 16 );


    return ecc;
}

// Taken from iQue stuff: https://github.com/emoose/iQueTool/blob/master/iQueTool/Structs/iQueBlockSpare.cs
// calcs ECC for a 512-byte block, formatted like the iQue ([3-byte ECC(0x100:0x200)] 0xFF 0xFF [3-byte ECC(0:0x100)])
void Calculate512Ecc( uint8_t* pageData, uint8_t* dst )
{
    dst[3] = 0xFF;
    dst[4] = 0xFF;

    uint32_t ecc1 = Calculate256Ecc( pageData, 0 );
    uint32_t ecc2 = Calculate256Ecc( pageData, 0x100 );
    
    dst[0] = ecc2 & 0xFF;
    dst[1] = ( ecc2 >> 8 ) & 0xFF;
    dst[2] = ( ecc2 >> 16 ) & 0xFF;
    
    dst[5] = ecc1 & 0xFF;
    dst[6] = ( ecc1 >> 8 ) & 0xFF;
    dst[7] = ( ecc1 >> 16 ) & 0xFF;
}
        
// ECC stuff end.

// BBFS Footer checksum.
uint16_t calcBBFScheckSum( uint8_t* dat ) {
  // Dat is assumed to be the beginning of the BBFS area.
  uint16_t* dat16 = (uint16_t*) dat;
  uint16_t sum = 0;
  
  for ( int i = 0; i < 0x1FFF; ++i ) {
    uint16_t tmp = dat16[ i ];
    tmp = (( tmp << 8 ) | (tmp >> 8 )) & 0xFFFF;
    
    sum += tmp;
  }
  
  sum = (uint16_t) 0xCAD7 - sum;
  
  return sum;
}

// Swap endianness functions. The file table stuff is big endian.
// RP2040 is little endian.
uint32_t swapEndianWord( uint32_t d ) {
  return ( ( d & 0x000000FF ) << 24 ) | ( ( d & 0x0000FF00 ) << 8 ) | 
         ( ( d & 0x00FF0000 ) >> 8 )  | ( ( d & 0xFF000000 ) >> 24 );
}

uint16_t swapEndian2Byte( uint16_t d ) {
  return ( ( d & 0x00FF ) << 8 ) |  ( ( d & 0xFF00 ) >> 8 ); 
}

// LED stuff.
unsigned ledState = 0;
void put_pixel( uint32_t pixel_grb )
{
  pio_sm_put_blocking( LED_PIO, 0, pixel_grb << 8u );
}

void put_rgb( uint8_t red, uint8_t green, uint8_t blue )
{
  uint32_t mask = (green << 16) | (red << 8) | (blue << 0);
  put_pixel(mask);
}

void flipLED() {
  if ( ledState ) {
    put_rgb( 0, 0, 0 );
  } else {
    put_rgb( LED_RED, LED_GREEN, LED_BLUE );
  }
  
  ledState = !ledState;
}

int parseSDGames( const char* romDir ) {
  
  printf( "Parsing ROMs from SD card\n" );
  FATFS fs;
  FRESULT fr = f_mount(&fs, "", 1);
  if (FR_OK != fr) {
    panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    return -1;
  }
  
  FRESULT res;
  DIR dir;
  FILINFO fno;

  unsigned romCnt = 0;

  res = f_opendir( &dir, romDir );
  if ( res == FR_OK ) {
    for (;;) {
      // Get file.
      res = f_readdir( &dir, &fno );
      // All files read?
      if ( fno.fname[0] == 0 ) {
        break;
      }
      
      printf( "Parse file %s\n", fno.fname );
      
      // It's a directory.
      if ( fno.fattrib & AM_DIR ) {
        continue;
      }
      
      // Get extension.
      char* dot = strrchr( fno.fname, '.' );
      
      // Is a ROM?
      if ( dot && 
           ( ( strcmp( dot + 1, "app" ) == 0 ) ||
           ( strcmp( dot + 1, "rec" ) == 0 ) ) ) {
        
        // Store it.
        unsigned name_len = dot - fno.fname;
        strncpy( romList[ romCnt ].fileName, fno.fname, name_len );
        strncpy( romList[ romCnt ].fileExt, dot + 1, 3 );
        romList[ romCnt ].fileSize = fno.fsize;

        printf( "Added rom file: %s \n", fno.fname );

        ++romCnt;
        romListLen = romCnt;
      }
    }
    
    f_closedir( &dir );

  } else {
    printf("Failed to open \"%s\". (%u)\n", romDir, res);
  }
  
  f_unmount( "" );
  
  return 0;
}

// Reads a single NAND block. Assumes main signals are set to initial values.
// This means:
// gpio_put( nCE, 1 );
// gpio_put( CLE, 0 );
// gpio_put( nWE, 1 );
// gpio_put( ALE, 0 );
// gpio_put( nRE, 1 );
// gpio_set_dir_out_masked( IOMASK );

void readSingleNANDBlock( unsigned curBlock, uint8_t* dst, unsigned skipSpare ) {
  printf( "->readSingleNANDBlock: %u\n", curBlock );
  
  uint32_t curAddr = curBlock * PAGES_PER_BLOCK * BYTES_PER_PAGE;
    
  // Enable NAND.
  gpio_put( nCE, 0 );
  
  // Set CLE.
  gpio_put( CLE, 1 );
  
  // Set write.
  gpio_put( nWE, 0 );
  
  // Set Read 1 command.
  gpio_put_masked( IOMASK, READ1 );
  
  PAUSE;
  
  // Finish CMD latch.
  gpio_put( nWE, 1 );
  gpio_put( CLE, 0 );
  
  // Start address latch.
  gpio_put( ALE, 1 );
  
  // First byte.
  gpio_put_masked( IOMASK, reverse_byte( curAddr & IOMASK ) );
  PAUSE;
  gpio_put( nWE, 0 );
  PAUSE;
  gpio_put( nWE, 1 );
  PAUSE;
  
  // Second byte. Skip address bit A8 due to page size.
  gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 9 ) & IOMASK ) );
  PAUSE;
  gpio_put( nWE, 0 );
  PAUSE;
  gpio_put( nWE, 1 );
  PAUSE;
  
  // Third byte.
  gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 17 ) & IOMASK ) );
  PAUSE;
  gpio_put( nWE, 0 );
  PAUSE;
  gpio_put( nWE, 1 );
  PAUSE;
  
  // 4th byte.
  gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 25 ) & IOMASK ) );
  PAUSE;
  gpio_put( nWE, 0 );
  PAUSE;
  gpio_put( nWE, 1 );
  PAUSE;
  
  // End address latch.
  gpio_put( ALE, 0 );
  
  // Switch direction.
  gpio_set_dir_in_masked( IOMASK );
    
  unsigned dumpSpares = !skipSpare;
  // Start read out the current block.
  // All bytes per block.
  for ( unsigned curPage = 0; curPage < PAGES_PER_BLOCK; ++curPage ) {
    // Wait for page read.
    sleep_us( PAGEREAD_US );
    
    // All bytes per page.
    for ( unsigned curByte = 0; curByte < BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE; ++curByte ) {
      gpio_put( nRE, 0 );
      PAUSE;
      
      // Read out byte.
      uint8_t b = gpio_get_all() & IOMASK;
      
      // Reverse and store. Potentially skip spare bits.
      if ( curByte < BYTES_PER_PAGE || dumpSpares ) {
        dst[ curByte + ( curPage * ( BYTES_PER_PAGE + ( dumpSpares * SPARE_BYTES_PER_PAGE ) ) ) ] = reverse_byte( b );
      }
              
      gpio_put( nRE, 1 );
      PAUSE;
    }
  }
  
  // Stop read out since we reached the end of a block.
  gpio_put( nCE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  PAUSE;
}

// Reads out a number of NAND blocks.
void readBlocks( unsigned startBlock, unsigned nrBlocks, uint8_t* dst, unsigned skipSpare ) {
  printf( "read blocks\n" );
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  unsigned dumpSpares = !skipSpare;
  
  // Go through each block.
  for ( unsigned curBlock = startBlock; curBlock < nrBlocks + startBlock; ++curBlock ) {
    
    unsigned currentOffset = ( curBlock - startBlock ) * PAGES_PER_BLOCK * ( BYTES_PER_PAGE + ( dumpSpares * SPARE_BYTES_PER_PAGE ) );
    
    readSingleNANDBlock( curBlock, dst + currentOffset, skipSpare );     
  }
  
  printf( "Done reading.\n" );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

void dumpMainNAND( FIL* f ) {
  printf( "DUMPING NAND\n" );
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Go through each block.
  for ( unsigned curBlock = 0; curBlock < MAX_BLOCKS; ++curBlock ) {
    uint8_t blockBuffer[ PAGES_PER_BLOCK * ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) ];
    
    readSingleNANDBlock( curBlock, blockBuffer, 0 );
    
    // Write out to file.
    unsigned bw;
    f_write( f, blockBuffer, ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) * PAGES_PER_BLOCK, &bw );
      
    printf( "Dumped block %u / 4096\n", curBlock );
    
    // Show led.
    flipLED();
  }
  
  printf( "Done dumping.\n" );
  
  // Turn off LED.
  put_rgb( 0, 0, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

uint8_t latestBBFS[ BYTES_PER_PAGE * PAGES_PER_BLOCK ];
unsigned latestBBFSindex = -1;

void loadLatestBBFS() {
  printf( "Load latest BBFS\n" );
  
  // Read every possible BBFS.
  latestBBFSindex = 0;
  unsigned largestSeq = 0;
  for ( unsigned i = 0; i < NR_FAT_BLOCKS; ++i ) {
    // Read the file system block.
    readBlocks( FILETABLE_START_BLOCK + i, 1, latestBBFS, 1 );
    
    // Check magic.
    if ( strncmp( (const char*) ( latestBBFS + FILETABLE_MAGIC_OFFSET ), "BBFS", 4 ) ) {
      continue;
    }
    
    // Check sequence number.
    uint32_t seq = ((uint32_t*) latestBBFS)[ FILETABLE_SEQ_OFFSET / 4 ];
    
    // Swap endian.
    seq = swapEndianWord( seq );
    
    printf( "Seq. at BBFS %u: %lu\n", i, seq );
    
    if ( seq > largestSeq ) {
      latestBBFSindex = i;
      largestSeq = seq;
    }
  }
  
  printf( "Current BBFS is at offset %u\n", latestBBFSindex );
  
  if ( latestBBFSindex != NR_FAT_BLOCKS - 1 ) {
    // Reload the latest one.
    readBlocks( FILETABLE_START_BLOCK + latestBBFSindex, 1, latestBBFS, 1 );
  }
}

bbfsFile* getFile( unsigned fileNr ) {
  //printf( "Get File\n" );
  
  bbfsFile* curFile = ( bbfsFile* ) (latestBBFS + FILETABLE_START_OFFSET + ( fileNr * FILESTRUCT_SIZE ) );

  return curFile;  
}

void createLinkChain( unsigned startBlock, unsigned nrBlocks ) {
  // Right now we always place them at the same spot.
  for ( unsigned curBlock = 0; curBlock < nrBlocks; ++curBlock ) {
    if ( curBlock == nrBlocks - 1 ) {
      ( (uint16_t*) latestBBFS )[ curBlock + startBlock ] = 0xFFFF;
    } else {
      ( (uint16_t*) latestBBFS )[ curBlock + startBlock ] = swapEndian2Byte( (uint16_t) curBlock + startBlock + 1 );
    }
  }
}

// Erases the whole NAND. Uses multiplane erase.
void eraseNAND() {
  printf( "Erasing NAND\n" );
  // Erases the complete NAND.
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Disable write protect.
  gpio_put( nWP, 1 );
  
  // We will do a multiplane erase.
  for ( unsigned curBlock = 0; curBlock < MAX_BLOCKS; curBlock += NR_PLANES ) {
    
    // Enable NAND.
    gpio_put( nCE, 0 );
    
    // Go over each plane.
    for ( unsigned curPlane = 0; curPlane < NR_PLANES; ++curPlane ) {
      uint32_t curAddr = ( curBlock + curPlane ) * PAGES_PER_BLOCK * BYTES_PER_PAGE;
      //printf( "Setting address to %lu\n", curAddr );
      
      // Set CLE.
      gpio_put( CLE, 1 );
      
      // Set write.
      gpio_put( nWE, 0 );
      
      // Set Block erase setup command.
      gpio_put_masked( IOMASK, reverse_byte( BLOCK_ERASE_SETUP_CMD ) );
      
      PAUSE;
      
      // Finish CMD latch.
      gpio_put( nWE, 1 );
      PAUSE;
      gpio_put( CLE, 0 );
      PAUSE;
      
      // Start address latch.
      gpio_put( ALE, 1 );
      
      // First byte A9 to A16
      gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 9 ) & IOMASK ) );
      PAUSE;
      gpio_put( nWE, 0 );
      PAUSE;
      gpio_put( nWE, 1 );
      PAUSE;
      
      // Second byte A17 to A24
      gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 17 ) & IOMASK ) );
      PAUSE;
      gpio_put( nWE, 0 );
      PAUSE;
      gpio_put( nWE, 1 );
      PAUSE;
      
      // Third byte. A25.
      gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 25 ) & IOMASK ) );
      PAUSE;
      gpio_put( nWE, 0 );
      PAUSE;
      gpio_put( nWE, 1 );
      PAUSE;
      
      // End address latch.
      gpio_put( ALE, 0 );
      
      PAUSE;
    }
    
    // Send the erase confirm command.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    // Set Block erase confirm command.
    gpio_put_masked( IOMASK, reverse_byte( ERASE_CONFIRM_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    
    // Wait.
    sleep_us( BLOCK_ERASE_TIME_MAX_US );
    
    // Check status. First write multiplane status command.
    // Set CLE.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    gpio_put_masked( IOMASK, reverse_byte( MULTIPLANE_STATUS_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    PAUSE;
    
    // Switch direction.
    gpio_set_dir_in_masked( IOMASK );
    
    // Set to read mode.
    gpio_put( nRE, 0 );
    PAUSE;
    
    // Read out status. Currently unused.
    //uint8_t curStat = reverse_byte( gpio_get_all() & IOMASK );
    
    // Currently no error handling.
    
    // Switch back to write mode.
    gpio_put( nRE, 1 );
    PAUSE;
    
    // Switch direction.
    gpio_set_dir_out_masked( IOMASK );
    
    // Fnish the planes.
    gpio_put( nCE, 0 );
    PAUSE;
  }
  
  // Re-enable write protect.
  gpio_put( nWP, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

// Erases a certain number of NAND blocks.
void eraseNANDBlocks( unsigned startBlock, unsigned nrBlocks ) {
  printf( "Erasing blocks: %u (len: %u)\n", startBlock, nrBlocks );
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Disable write protect.
  gpio_put( nWP, 1 );
  
  // Do block by block.
  for ( unsigned curBlock = startBlock; curBlock < startBlock + nrBlocks; ++curBlock ) {
    
    // Enable NAND.
    gpio_put( nCE, 0 );
    
    uint32_t curAddr = curBlock * PAGES_PER_BLOCK * BYTES_PER_PAGE;
    //printf( "Setting address to %lu\n", curAddr );
    
    // Set CLE.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    // Set Block erase setup command.
    gpio_put_masked( IOMASK, reverse_byte( BLOCK_ERASE_SETUP_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    PAUSE;
    
    // Start address latch.
    gpio_put( ALE, 1 );
    
    // First byte A9 to A16
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 9 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // Second byte A17 to A24
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 17 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // Third byte. A25.
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 25 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // End address latch.
    gpio_put( ALE, 0 );
    
    PAUSE;
    
    // Send the erase confirm command.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    // Set Block erase confirm command.
    gpio_put_masked( IOMASK, reverse_byte( ERASE_CONFIRM_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    
    // Wait.
    sleep_us( BLOCK_ERASE_TIME_MAX_US );
    
    // Check status. First write status command.
    // Set CLE.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    gpio_put_masked( IOMASK, reverse_byte( STATUS_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    PAUSE;
    
    // Switch direction.
    gpio_set_dir_in_masked( IOMASK );
    
    // Set to read mode.
    gpio_put( nRE, 0 );
    PAUSE;
    
    // Read out status. Currently unused.
    // uint8_t curStat = reverse_byte( gpio_get_all() & IOMASK );
    
    // Currently no error handling.
    
    // Switch back to write mode.
    gpio_put( nRE, 1 );
    PAUSE;
    
    // Switch direction.
    gpio_set_dir_out_masked( IOMASK );
    
    // Fnish blocks.
    gpio_put( nCE, 0 );
    PAUSE;
  }
  
  // Re-enable write protect.
  gpio_put( nWP, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

// Programs the whole NAND with a file content (the file is assumed
// to have the spares included).
// To speed things up, it does a multiplane program.
// Note: DOES NOT ERASE. You need to erase beforehand.
void programNAND( FIL* f ) {
  printf( "Program NAND\n" );
  // Programs the complete NAND.
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Disable write protect.
  gpio_put( nWP, 1 );
  
  // We will do a multiplane program.
  for ( unsigned curBlock = 0; curBlock < MAX_BLOCKS; curBlock += NR_PLANES ) {
    
    printf( "Reading block %u\n", curBlock );
    // Read in the 4 parallel blocks from file.
    uint8_t dataBuffer[ ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) * PAGES_PER_BLOCK * NR_PLANES ];
    unsigned br;
    f_read( f, dataBuffer, ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) * PAGES_PER_BLOCK * NR_PLANES, &br );
    
    // Go over pages.
    for ( unsigned curPage = 0; curPage < PAGES_PER_BLOCK; ++curPage ) {
      
      // Go over each plane.
      for ( unsigned curPlane = 0; curPlane < NR_PLANES; ++curPlane ) {
        // Enable NAND.
        gpio_put( nCE, 0 );
        
        uint32_t curAddr = ( ( ( curBlock + curPlane ) * PAGES_PER_BLOCK ) + curPage ) * BYTES_PER_PAGE;
        
        // Set CLE.
        gpio_put( CLE, 1 );
        
        // Set write.
        gpio_put( nWE, 0 );
        
        // Set seq. data input command
        gpio_put_masked( IOMASK, reverse_byte( SEQ_DATA_INPUT_CMD ) );
        
        PAUSE;
        
        // Finish CMD latch.
        gpio_put( nWE, 1 );
        gpio_put( CLE, 0 );
        
        // Start address latch.
        gpio_put( ALE, 1 );
        
        // First byte A0 to A7
        gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 0 ) & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
        
        // Second byte A9 to A16
        gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 9 ) & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
        
        // Third byte. A17 to A24.
        gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 17 ) & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
        
        // 4th byte. A24.
        gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 25 ) & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
        
        // End address latch.
        gpio_put( ALE, 0 );
        PAUSE;
        
        // Read in data.
        for ( unsigned curByte = 0; curByte < ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ); ++curByte ) {
          uint8_t b = dataBuffer[ curByte + ( ( ( curPlane * PAGES_PER_BLOCK ) + curPage ) * ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) ) ];
          
          gpio_put_masked( IOMASK, reverse_byte( b & IOMASK ) );
          PAUSE;
          gpio_put( nWE, 0 );
          PAUSE;
          gpio_put( nWE, 1 );
          PAUSE;
        }
        
        // Set CLE.
        gpio_put( CLE, 1 );
        
        // Set write.
        gpio_put( nWE, 0 );
        
        if ( curPlane < NR_PLANES - 1 ) {
          // Set program dummy command
          gpio_put_masked( IOMASK, reverse_byte( DUMMY_PROGRAM_CMD ) );
        } else {
          // Program confirm command.
          gpio_put_masked( IOMASK, reverse_byte( PROGRAM_CONFIRM_CMD ) );
        }
        
        PAUSE;
        
        // Finish CMD latch.
        gpio_put( nWE, 1 );
        PAUSE;
        gpio_put( CLE, 0 );
        
        if ( curPlane < NR_PLANES - 1 ) {
          sleep_us( DUMMY_BUSY_TIME_US );
        } else {
          sleep_us( PROG_TIME_MAX_US );
        }
      }
      
      // Check status. First write multiplane status command.
      // Set CLE.
      gpio_put( CLE, 1 );
      
      // Set write.
      gpio_put( nWE, 0 );
      
      gpio_put_masked( IOMASK, reverse_byte( MULTIPLANE_STATUS_CMD ) );
      
      PAUSE;
      
      // Finish CMD latch.
      gpio_put( nWE, 1 );
      PAUSE;
      gpio_put( CLE, 0 );
      PAUSE;
      
      // Switch direction.
      gpio_set_dir_in_masked( IOMASK );
      
      // Set to read mode.
      gpio_put( nRE, 0 );
      PAUSE;
      
      // Read out status. Currently unused.
      // uint8_t curStat = reverse_byte( gpio_get_all() & IOMASK );
      
      // Currently no error handling.
      
      // Switch back to write mode.
      gpio_put( nRE, 1 );
      PAUSE;
      
      // Switch direction.
      gpio_set_dir_out_masked( IOMASK );
      
      // Fnish.
      gpio_put( nCE, 1 );
      PAUSE;
    }
  }

  // Re-enable write protect.
  gpio_put( nWP, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

// Programs a single block to NAND. Does NOT erase beforehand.
// Assumes signals to be initialized.
// This means:
//  gpio_put( nCE, 1 );
//  gpio_put( CLE, 0 );
//  gpio_put( nWE, 1 );
//  gpio_put( ALE, 0 );
//  gpio_put( nRE, 1 );
//  gpio_set_dir_out_masked( IOMASK );
//  gpio_put( nWP, 1 );
void programSingleBlock( uint8_t* src, unsigned curBlock, unsigned hasSpares ) {
  
  // Go over each page.
  for ( unsigned curPage = 0; curPage < PAGES_PER_BLOCK; ++curPage ) {    
    // Enable NAND.
    gpio_put( nCE, 0 );
    
    uint32_t curAddr = ( ( curBlock * PAGES_PER_BLOCK ) + curPage ) * BYTES_PER_PAGE;
    
    // Set CLE.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    // Set seq. data input command
    gpio_put_masked( IOMASK, reverse_byte( SEQ_DATA_INPUT_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    gpio_put( CLE, 0 );
    
    // Start address latch.
    gpio_put( ALE, 1 );
    
    // First byte A0 to A7
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 0 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // Second byte A9 to A16
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 9 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // Third byte. A17 to A24.
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 17 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // 4th byte. A24.
    gpio_put_masked( IOMASK, reverse_byte( ( curAddr >> 25 ) & IOMASK ) );
    PAUSE;
    gpio_put( nWE, 0 );
    PAUSE;
    gpio_put( nWE, 1 );
    PAUSE;
    
    // End address latch.
    gpio_put( ALE, 0 );
    PAUSE;
    
    // Read in data.
    for ( unsigned curByte = 0; curByte < BYTES_PER_PAGE; ++curByte ) {
      uint8_t b = src[ curByte + ( curPage * ( BYTES_PER_PAGE + ( hasSpares ? SPARE_BYTES_PER_PAGE : 0 ) ) ) ];
      
      gpio_put_masked( IOMASK, reverse_byte( b & IOMASK ) );
      PAUSE;
      gpio_put( nWE, 0 );
      PAUSE;
      gpio_put( nWE, 1 );
      PAUSE;
    }
    
    if ( !hasSpares ) {
      // Calc spare.
      uint8_t ecc[ 8 ];
      Calculate512Ecc( src + ( curPage * BYTES_PER_PAGE ), ecc );
      
      uint8_t dummySpares[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF };
      
      // Do spare bytes. First 8 times 0xFF (no SA, skip bad blocks for now).
      for ( unsigned curByte = 0; curByte < SPARE_BYTES_PER_PAGE / 2; ++curByte ) {
        uint8_t b = dummySpares[ curByte ];
        
        gpio_put_masked( IOMASK, reverse_byte( b & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
        
      }
    
      // And now the actual spare bytes.      
      for ( unsigned curByte = 0; curByte < SPARE_BYTES_PER_PAGE / 2; ++curByte ) {
        uint8_t b = ecc[ curByte ];
        
        gpio_put_masked( IOMASK, reverse_byte( b & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
      }
      
    } else {
      // Just read out the spare bytes.
      for ( unsigned curByte = BYTES_PER_PAGE; curByte < ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ); ++curByte ) {
        uint8_t b = src[ curByte + ( curPage * ( BYTES_PER_PAGE + SPARE_BYTES_PER_PAGE ) ) ];
        
        gpio_put_masked( IOMASK, reverse_byte( b & IOMASK ) );
        PAUSE;
        gpio_put( nWE, 0 );
        PAUSE;
        gpio_put( nWE, 1 );
        PAUSE;
      }
    }
    
    // Set CLE.
    gpio_put( CLE, 1 );
    
    // Set write.
    gpio_put( nWE, 0 );
    
    // Program confirm command.
    gpio_put_masked( IOMASK, reverse_byte( PROGRAM_CONFIRM_CMD ) );
    
    PAUSE;
    
    // Finish CMD latch.
    gpio_put( nWE, 1 );
    PAUSE;
    gpio_put( CLE, 0 );
    
    sleep_us( PROG_TIME_MAX_US );
  }
  
  // Check status. First write status command.
  // Set CLE.
  gpio_put( CLE, 1 );
  
  // Set write.
  gpio_put( nWE, 0 );
  
  gpio_put_masked( IOMASK, reverse_byte( STATUS_CMD ) );
  
  PAUSE;
  
  // Finish CMD latch.
  gpio_put( nWE, 1 );
  PAUSE;
  gpio_put( CLE, 0 );
  PAUSE;
  
  // Switch direction.
  gpio_set_dir_in_masked( IOMASK );
  
  // Set to read mode.
  gpio_put( nRE, 0 );
  PAUSE;
  
  // Read out status. Currently unused.
  // uint8_t curStat = reverse_byte( gpio_get_all() & IOMASK );
  
  // Currently no error handling.
  
  // Switch back to write mode.
  gpio_put( nRE, 1 );
  PAUSE;
  
  // Switch direction.
  gpio_set_dir_out_masked( IOMASK );
  
  // Fnish.
  gpio_put( nCE, 1 );
  PAUSE;
}

// Programs the content of a single file to the NAND.
// NOTES:
// * DOES NOT ERASE. You need to erase first.
void programNANDFile( FIL* f, unsigned startBlock, unsigned sizeInBlocks, unsigned fileHasSpares ) {
  printf( "Program NAND Net File\n" );
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Disable write protect.
  gpio_put( nWP, 1 );

  const unsigned blockSize = ( BYTES_PER_PAGE + ( fileHasSpares ? SPARE_BYTES_PER_PAGE : 0 ) ) * PAGES_PER_BLOCK;
  // Go over blocks.
  for ( unsigned curBlock = 0; curBlock < sizeInBlocks; ++curBlock ) {
    printf( "Reading block %u\n", curBlock );
    uint8_t dataBuffer[ blockSize ];
    unsigned br;
    
    // Load buffer.
    f_read( f, dataBuffer, blockSize, &br );
    
    // Program the current block.
    programSingleBlock( dataBuffer, curBlock + startBlock, fileHasSpares );
  }

  // Re-enable write protect.
  gpio_put( nWP, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

// Writes an array to NAND.
// NOTES:
// * DOES NOT ERASE. You need to erase first.
void writeArrayToNand( uint8_t* dat, unsigned startBlock, unsigned sizeInBlocks, unsigned hasSpares ) {
  printf( "Write array to NAND\n" );
  printf( "startBlock: %u, size in blocks: %u\n", startBlock, sizeInBlocks );
  
  // Make sure initial signals are right.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
  gpio_set_dir_out_masked( IOMASK );
  
  // Disable write protect.
  gpio_put( nWP, 1 );

  const unsigned blockSize = ( BYTES_PER_PAGE + ( hasSpares ? SPARE_BYTES_PER_PAGE : 0 ) ) * PAGES_PER_BLOCK;
  // Go over blocks.
  for ( unsigned curBlock = 0; curBlock < sizeInBlocks; ++curBlock ) {
    // Program the current block.
    programSingleBlock( dat + ( curBlock * blockSize ), curBlock + startBlock, hasSpares );
    
  }

  // Re-enable write protect.
  gpio_put( nWP, 0 );
  
  // Set defaut signal state.
  gpio_put( nCE, 1 );
  gpio_put( CLE, 0 );
  gpio_put( nWE, 1 );
  gpio_put( ALE, 0 );
  gpio_put( nRE, 1 );
}

// Copies a file into the data area and creates spare on the fly.
// Does an erase over that area first.
// IMPORTANT: Assumes a continous data area, it's not splitting currently. 
void copyFileToNAND( FIL* f, unsigned startBlock, unsigned sizeInBlocks ) {
  printf( "Copy file to block: %u (size: %u)\n", startBlock, sizeInBlocks );
  // Start with erasing.
  eraseNANDBlocks( startBlock, sizeInBlocks );
  
  // Copy file.
  programNANDFile( f, startBlock, sizeInBlocks, 0 );
}

// Replace dummy by real file.
void doDummyReplace( const char* appName, const char* ext, unsigned appSizeBytes, FIL* f ) {
  printf( "doDummyReplace\n" );
  // Do the actual copy.
  unsigned sizeInBlocks = appSizeBytes / ( BYTES_PER_PAGE * PAGES_PER_BLOCK );
  copyFileToNAND( f, FILE_TARGET_BLOCK, sizeInBlocks );
  
  // Adjust the link chain.
  createLinkChain( FILE_TARGET_BLOCK, sizeInBlocks );
  
  // Get the dummy file.
  bbfsFile* bbfile;
  
  // Search for the dummy file.
  for ( unsigned curFile = 0; curFile < FILETABLE_MAX_FILES; ++curFile ) {
    bbfile = getFile( curFile );
    if ( bbfile->valid && 
         !strncmp( bbfile->fileExt, "rec", 3 ) && 
         !strncmp( bbfile->fileName, appName, 8 ) ) {
           
      printf( "FOUND REC FILE.\n" );
      break;
    }
  }
  
  // Clear the previous FAT info.
  *( ( (uint16_t*) latestBBFS ) + swapEndian2Byte( bbfile->startBlock ) ) = 0;
  
  // Set the new link chain start.
  bbfile->startBlock = swapEndian2Byte( FILE_TARGET_BLOCK );
  
  // Set the new file size.
  bbfile->fileSize = swapEndianWord( appSizeBytes );
  
  // Set new extension.
  memcpy( bbfile->fileExt, ext, 3 );
  
  // Calculate new checksum.
  *( (uint16_t*) (latestBBFS + FILETABLE_CHECKSUM_OFFSET) ) = 
    swapEndian2Byte( calcBBFScheckSum( latestBBFS ) );
    
  // Erase FAT.
  eraseNANDBlocks( 0xFF0 + latestBBFSindex, 1 );
  
  // And write the new BBFS to the NAND.
  writeArrayToNand( latestBBFS, 0xFF0 + latestBBFSindex, 1, 0 );
}

// Delete a given file.
void deleteFile( bbfsFile* f ) {
  // Set it to invalid.
  f->valid = 0;
  
  // Free the FAT.
  unsigned sizeInBlocks = swapEndianWord( f->fileSize ) / ( BYTES_PER_PAGE * PAGES_PER_BLOCK );
  unsigned startBlock = swapEndian2Byte( f->startBlock );
  for ( unsigned curBlock = startBlock; curBlock < startBlock + sizeInBlocks; ++curBlock ) {
    ((uint16_t*) latestBBFS)[ curBlock ] = 0;
  };
};

// Create random dummy file. Returns the block where the dummy was written
// to. Writes content to NAND, but filesystem only local.
unsigned addDummyFile( const uint8_t* filename, const uint8_t* ext ) {
  
  // Get a free block.
  unsigned targetBlock = -1;
  
  for ( unsigned curBlock = 0; curBlock < FAT_ENTRIES; ++curBlock ) {
    uint16_t curEntry = swapEndian2Byte( ( (uint16_t*) latestBBFS )[ curBlock ] );
    if ( curEntry == 0 ) {
      targetBlock = curBlock;
      break;
    }
  }
  
  // Create dummy.
  uint8_t dummyContent[ BYTES_PER_PAGE * PAGES_PER_BLOCK ];
  memset( dummyContent, 0, BYTES_PER_PAGE * PAGES_PER_BLOCK );
  
  // Copy it to NAND.
  eraseNANDBlocks( targetBlock, 1 );
  writeArrayToNand( dummyContent, targetBlock, 1, 0 );
  
  // Get a file slot.
  bbfsFile* bbfile = 0;
  for ( unsigned fileSlot = 0; fileSlot < FILETABLE_MAX_FILES; ++fileSlot ) {
    bbfile = getFile( fileSlot );
    
    if ( !bbfile->valid  ) {
      break;
    }
  }
  
  // Add file.
  bbfile->valid = 1;
  memcpy( bbfile->fileName, filename, 8 );
  memcpy( bbfile->fileExt, ext, 3 );
  bbfile->fileSize = swapEndianWord( BYTES_PER_PAGE * PAGES_PER_BLOCK );
  bbfile->startBlock = swapEndian2Byte( targetBlock );
  
  // Set FAT.
  ((uint16_t*) latestBBFS)[ targetBlock ] = 0xFFFF;
  
  return targetBlock;
}

void writeCurrentFATtoNAND() {
  // Calculate new checksum.
  *( (uint16_t*) (latestBBFS + FILETABLE_CHECKSUM_OFFSET) ) = 
    swapEndian2Byte( calcBBFScheckSum( latestBBFS ) );
  
  // Erase FAT.
  eraseNANDBlocks( 0xFF0 + latestBBFSindex, 1 );
  
  // And write the new BBFS to the NAND.
  writeArrayToNand( latestBBFS, 0xFF0 + latestBBFSindex, 1, 0 );
}

// Init with available dummy files.
void initAvailableDummyFiles() {
  printf( "Init NAND with dummy files\n" );
  // Disable existing ROM files.
  bbfsFile* bbfile = 0;
  for ( unsigned fileSlot = 0; fileSlot < FILETABLE_MAX_FILES; ++fileSlot ) {
    bbfile = getFile( fileSlot );
    
    if ( bbfile->valid ) {
      printf( "Found valid file in NAND:\n" );
      for ( unsigned fCnt = 0; fCnt < 8; ++fCnt ) {
        printf( "%c", bbfile->fileName[ fCnt ] );
      }
      printf( "\n" );
      // Compare to previously parsed ROMs on SD card.
      for ( unsigned sdCnt = 0; sdCnt < romListLen; ++sdCnt ) {
        printf( "Comparing against: %s\n", romList[ sdCnt ].fileName );
        
        if ( !strncmp( bbfile->fileName, romList[ sdCnt ].fileName, 8 ) ) {
          // Found file in the iQue FAT which is also on the SD card. Disable it.
          deleteFile( bbfile );
          
          printf( "Also found that file on SD card. Deleting it.\n" );
          
          break;
        }
      }
    }
  }
  
  // Add dummies.
  for ( unsigned curRom = 0; curRom < romListLen; ++curRom ) {
    romList[ curRom ].startBlock = addDummyFile( (uint8_t*) romList[ curRom ].fileName, (const uint8_t*) "rec" );
  }
  
  // Write new FAT to NAND.
  writeCurrentFATtoNAND();
}


/* Hardware Configuration of the SD Card socket "object" */
static sd_card_t sd_card = {.type = SD_IF_SDIO, .sdio_if_p = &sdio_if};

/**
 * @brief Get the number of SD cards.
 *
 * @return The number of SD cards, which is 1 in this case.
 */
size_t sd_get_num() { return 1; }

/**
 * @brief Get a pointer to an SD card object by its number.
 *
 * @param[in] num The number of the SD card to get.
 *
 * @return A pointer to the SD card object, or @c NULL if the number is invalid.
 */
sd_card_t* sd_get_by_num(size_t num) {
    if (0 == num) {
        // The number 0 is a valid SD card number.
        // Return a pointer to the sd_card object.
        return &sd_card;
    } else {
        // The number is invalid. Return @c NULL.
        return NULL;
    }
}


void adjustSwitch( uint32_t address, uint32_t val ) {
  // Start condition.
  gpio_put( BUS_SCL, 1 );
  gpio_put( BUS_SDA, 1 );
  sleep_us( SLEEPUS );
  
  gpio_put( BUS_SDA, 0 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 0 );
  sleep_us( SLEEPUS );
  
  for ( unsigned i = 0; i < 7; ++i ) {
    gpio_put( BUS_SDA, ( address >> ( 6 - i ) ) & 1 );
    sleep_us( SLEEPUS );
    gpio_put( BUS_SCL, 1 );
    sleep_us( SLEEPUS );
    gpio_put( BUS_SCL, 0 );
    sleep_us( SLEEPUS );
  }
  
  // Write bit.
  gpio_put( BUS_SDA, 0 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 1 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 0 );
  
  // Skip ACK bit.
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 1 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 0 );
  
  // Switch data.
  for ( unsigned i = 0; i < 8; ++i ) {
    gpio_put( BUS_SDA, val );
    sleep_us( SLEEPUS );
    gpio_put( BUS_SCL, 1 );
    sleep_us( SLEEPUS );
    gpio_put( BUS_SCL, 0 );
    sleep_us( SLEEPUS );
  }
  
  gpio_put( BUS_SDA, 0 );
  
  // Skip ACK bit.
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 1 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 0 );
  
  // STOP condition.
  gpio_put( BUS_SDA, 0 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SCL, 1 );
  sleep_us( SLEEPUS );
  gpio_put( BUS_SDA, 1 );
}

void enableRP() {
  // Disable DET.
  gpio_set_dir( DET, GPIO_IN );
  
  sleep_ms( 1 );
  
  // Disable switches.
  adjustSwitch( SWITCH0_ADDR, 0 );
  
  sleep_ms( 1 );  

  gpio_put( nCE, 1 );
  gpio_set_dir( nCE, GPIO_OUT );
  gpio_put( CLE, 0 );
  gpio_set_dir( CLE, GPIO_OUT );
  gpio_put( nWE, 1 );
  gpio_set_dir( nWE, GPIO_OUT );
  gpio_put( nWP, 0 );
  gpio_set_dir( nWP, GPIO_OUT );
  gpio_put( ALE, 0 );
  gpio_set_dir( ALE, GPIO_OUT );
  gpio_put( nRE, 1 );
  gpio_set_dir( nRE, GPIO_OUT );
}

void disableRP() {
  gpio_set_dir( IO7, GPIO_IN );
  gpio_set_dir( IO6, GPIO_IN );
  gpio_set_dir( IO5, GPIO_IN );
  gpio_set_dir( IO4, GPIO_IN );
  gpio_set_dir( IO3, GPIO_IN );
  gpio_set_dir( IO2, GPIO_IN );
  gpio_set_dir( IO1, GPIO_IN );
  gpio_set_dir( IO0, GPIO_IN );
  gpio_set_dir( nCE, GPIO_IN );
  gpio_set_dir( CLE, GPIO_IN );
  gpio_set_dir( nWE, GPIO_IN );
  gpio_set_dir( nWP, GPIO_IN );
  gpio_set_dir( ALE, GPIO_IN );
  gpio_set_dir( nRE, GPIO_IN );
  
  // Turn on switches.
  adjustSwitch( SWITCH0_ADDR, 1 );
  
  // Enable DET.
  gpio_put( DET, 0 );
  gpio_set_dir( DET, GPIO_OUT );
}

int __not_in_flash_func( main )() {
  stdio_init_all();
  
  gpio_init( BUS_SDA );
  gpio_put( BUS_SDA, 1 );
  
  gpio_init( BUS_SCL );
  gpio_put( BUS_SCL, 1 );
  
  gpio_set_dir( BUS_SDA, GPIO_OUT );
  gpio_set_dir( BUS_SCL, GPIO_OUT );
  
  gpio_init( DET );
  
  gpio_init( IO7 );
  gpio_set_dir( IO7, GPIO_IN );
  gpio_init( IO6 );
  gpio_set_dir( IO6, GPIO_IN );
  gpio_init( IO5 );
  gpio_set_dir( IO5, GPIO_IN );
  gpio_init( IO4 );
  gpio_set_dir( IO4, GPIO_IN );
  gpio_init( IO3 );
  gpio_set_dir( IO3, GPIO_IN );
  gpio_init( IO2 );
  gpio_set_dir( IO2, GPIO_IN );
  gpio_init( IO1 );
  gpio_set_dir( IO1, GPIO_IN );
  gpio_init( IO0 );
  gpio_set_dir( IO0, GPIO_IN );
  
  gpio_init( nCE );
  gpio_init( CLE );
  gpio_init( nWE );
  gpio_set_dir( nWE, GPIO_IN );
  
  gpio_init( nWP );    
  gpio_init( ALE );    
  gpio_init( nRE );
      
  gpio_init( LED_INT );
  gpio_set_dir( LED_INT, GPIO_OUT );
  
  gpio_init( BTN0 );
  gpio_set_dir( BTN0, GPIO_IN );
  gpio_pull_up( BTN0 );
  
  gpio_init( BTN1 );
  gpio_set_dir( BTN1, GPIO_IN );
  gpio_pull_up( BTN1 );
  
  
  // Regular card usage.
  disableRP();
  
  
  // DEBUG TEST.
  sleep_ms( 5000 );
  
  // Parse ROMs.
  parseSDGames( "/" );
  
  enableRP();
  // Load the file system.
  loadLatestBBFS();
  
  // Replace the recrypt.sys.
  FATFS fs;
  FRESULT fr = f_mount(&fs, "", 1);
  if (FR_OK != fr) {
      panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
      return -1;
  }

  FIL fil;
  fr = f_open( &fil, "recrypt.sys", FA_READ );
  printf( "Opened recrypt.sys from SD card.\n" );
  
  if (FR_OK != fr && FR_EXIST != fr) {
    panic("f_open(%s) error: %s (%d)\n", fil, FRESULT_str(fr), fr);
    return -1;
  }
  
  // Search for the recrypt.sys in NAND.
  for ( unsigned curFile = 0; curFile < FILETABLE_MAX_FILES; ++curFile ) {
    bbfsFile* f = getFile( curFile );
    
    if ( f->valid ) {
      //printf( "searching for recrypt.sys... valid file\n" );
      if ( !strncmp( f->fileName, "recrypt", 7 ) &&
           !strncmp( f->fileExt, "sys", 3 ) ) {
        printf( "Found current recrypt.sys, replacing it.\n" );
      
        copyFileToNAND( &fil, swapEndian2Byte( f->startBlock ), 1 );
        break;
      }
    }
  }
  
  f_unmount("");
  
  // Add dummies.
  initAvailableDummyFiles();
  
  disableRP();
  
  // Go over files.
  for ( unsigned curFile = 0; curFile < FILETABLE_MAX_FILES; ++curFile ) {
    bbfsFile* f = getFile( curFile );
    
    if ( f->valid ) {
      printf( "File %u is valid\n", curFile );
      
      printf( "Filename:\n" );
      for ( unsigned fCnt = 0; fCnt < 8; ++fCnt ) {
        printf( "%c", f->fileName[ fCnt ] );
      }
      
      printf( "." );
      
      for ( unsigned fCnt = 0; fCnt < 3; ++fCnt ) {
        printf( "%c", f->fileExt[ fCnt ] );
      }
      
      printf( "\n" );
      
      printf( "File size: %lu\n", swapEndianWord( f->fileSize ) );
      
      printf( "Start block: %u\n", swapEndian2Byte( f->startBlock ) );
      
    }
  } 
  
  
  // LED
  PIO pio = LED_PIO;
  int32_t sm = pio_claim_unused_sm( pio, false );
  uint32_t offset = pio_add_program( pio, &ws2812_program );
  ws2812_program_init( pio, sm, offset, LED_INT, 800000, true );
  put_rgb( 0, 0, 0 );
  
  printf( "starting snoop pio\n" );
  // Snoop input data.
  pio = LED_PIO;
  sm = pio_claim_unused_sm( pio, false );
  offset = pio_add_program( pio, &get_input_program );
  get_input_program_init( pio, sm, offset );
  
  printf( "started snoop pio\n" );
  
  //put_rgb( LED_RED, LED_GREEN, LED_BLUE );
  //sleep_ms( 500 );

  

  unsigned curReadCmd = 0;
  unsigned curAddr = 0;
  unsigned addrCnt = 0;
  
  
  unsigned doneDummyCopy = 0;
  while ( 1 ) {
    
    //flipLED();
    //sleep_ms( 500 );
    
    // Check for commands.
    if ( !pio_sm_is_rx_fifo_empty( pio, sm ) ) {
      
      unsigned curData = pio_sm_get( pio, sm );
      if ( curData & CMD_MASK ) {
        curAddr = 0;
        addrCnt = 0;
        
        if ( ( curData & 0b11111111 ) == 0 ) {
          curReadCmd = 1;
        } else {
          curReadCmd = 0;
        }
        
      //} else if ( curReadCmd && ( curData & ADDR_MASK ) ) {
      } else if ( curReadCmd && ( curData & ADDR_MASK ) && !doneDummyCopy ) {
        // Latch address.
        uint8_t tmpByte = reverse_byte(curData & 0b11111111); 
        //curAddr = (curAddr << 8) | tmpByte;
        curAddr = curAddr | ( tmpByte << ( addrCnt * 8 ) );
        ++addrCnt;
        
        
        // Check and copy.
        if ( addrCnt == 4 ) {
          printf( "%u\n", curAddr );
          for ( unsigned curDummy = 0; curDummy < romListLen; ++curDummy ) {
            uint32_t trigAddr = ( romList[ curDummy ].startBlock * BYTES_PER_PAGE * PAGES_PER_BLOCK ) >> 1;
            if ( ( curAddr >= trigAddr && curAddr < ( trigAddr + ( BYTES_PER_PAGE * PAGES_PER_BLOCK ) / 2 ) ) ) {
              
              enableRP();
              
              put_rgb( LED_RED, LED_GREEN, LED_BLUE );
              
              
              // Copy over stuff.
              FATFS fs;
              FRESULT fr = f_mount(&fs, "", 1);
              if (FR_OK != fr) {
                  panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
                  return -1;
              }

              FIL fil;
              
              char romName[ 15 ];
              memcpy( romName, romList[ curDummy ].fileName, 8 );
              romName[ 8 ] = '.';
              memcpy( romName + 9, romList[ curDummy ].fileExt, 3 );
              romName[ 12 ] = '\0';
        
              fr = f_open( &fil, romName, FA_READ );
              
              loadLatestBBFS();
              
              doDummyReplace( romList[ curDummy ].fileName, romList[ curDummy ].fileExt, romList[ curDummy ].fileSize, &fil );
              
              
              fr = f_close(&fil);
              if (FR_OK != fr) {
                  printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
              }

              f_unmount("");
              disableRP();
              
              put_rgb( 0, 0, 0 );
              doneDummyCopy = 1;
              
              
              break;
            }
          }
        }
    
    
      }
    }
    
    // Load NAND image.
    if ( !gpio_get( BTN0 ) ) {
      enableRP();
      
      flipLED();
      
      FATFS fs;
      FRESULT fr = f_mount(&fs, "", 1);
      if (FR_OK != fr) {
          panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
          return -1;
      }

      FIL fil;
      
      fr = f_open(&fil, "dump_4dummy.bin", FA_READ);
      printf( "opened dump_4dummy.bin\n");
      
      if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", fil, FRESULT_str(fr), fr);
        return -1;
      }
      
      eraseNAND();
      programNAND( &fil );
            
      f_unmount("");
      disableRP();
      flipLED();
    }
    
    // Dump.
    if ( !gpio_get( BTN1 ) ) {
      enableRP();
      
      flipLED();
      
      FATFS fs;
      FRESULT fr = f_mount(&fs, "", 1);
      if (FR_OK != fr) {
          panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
          return -1;
      }

      FIL fil;
      const char* const filename = "dump.bin";
      fr = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
      
      if (FR_OK != fr && FR_EXIST != fr) {
          panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
          return -1;
      }

      dumpMainNAND( &fil );

      fr = f_close(&fil);
      if (FR_OK != fr) {
          printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
      }

      f_unmount("");
      disableRP();
      flipLED();
    }
  }
}
