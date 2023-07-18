//  STM32F103-CMSIS-USART-lib.c
//      Version 1.0   07/17/2023    Updated comments and core library
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 07/2023  Started
//
//  Routines to provide serial terminal input and output via the hardware USART.
//
//  Supported global variable and routines:
//  ------------------------------------------------------------------------------------------
//  Global variable to point to the desired USART port on the microcontroller:
//    USART_TypeDef *USART_USART 
//  ------------------------------------------------------------------------------------------
//  Initalize USART port and set global USART pointer valriable
//    void
//    USART_init( USART_TypeDef *thisUSART )
//  ------------------------------------------------------------------------------------------
//  Output a single character to the serial terminal.
//    void
//    USART_putc(char c)
//  ------------------------------------------------------------------------------------------
//  Output a null-terminated string to the serial terminal.
//    void
//    USART_puts( const char *s )
//  ------------------------------------------------------------------------------------------
//  Output an Integer to the serial terminal. Base of output is set by "base".
//    void
//    USART_puti( int data, uint8_t base )
//  ------------------------------------------------------------------------------------------
//  Output a zero-padded hex value of "places" digits to the serial terminal. If the value is
//  larger than can be shown in "places" digits, then will output periods (.) instead of
//  digits. 
//    void
//    USART_puth( uint32_t number, uint8_t places )
//  ------------------------------------------------------------------------------------------
//  Get a single character from the serial terminal. The character is not displayed.
//    char
//    USART_getc()
//  ------------------------------------------------------------------------------------------
//  Get a string from the serial terminal. The string is displayed as it is typed in. The
//  string is entered after the <Enter> key is pressed. "strLen" determines the maximum length
//  of the string. The string  uint32_t
//    uint32_t
//    USART_gets( char *inStr, uint32_t strLen )
//  ------------------------------------------------------------------------------------------
//  Poll the terminal for a keyboard press. Return 0 if no key pressed, otherwise return ASCII
//  value of pressed key.
//    char
//    USART_pollc()
//  ------------------------------------------------------------------------------------------

#ifndef __STM32F103_CMSIS_USART_LIB_C
#define __STM32F103_CMSIS_USART_LIB_C

#include <stdlib.h>
#include "stm32f103xb.h"                  // Primary CMSIS header file

USART_TypeDef *USART_USART; // Global USART_USART varible to point to desired USART port

//  void
//  USART_init( USART_TypeDef *thisUSART )
//  Associate USART routines to designated USART port. The STM32F103 supports three USART
//  ports: USART1, USART2, USART3. Sets the global variable *USART_USART to the desired
//  USART port.
//  This if this routine is modified to work with other microcontrollers, then the rest of
//  the USART_ routines should work as-is.
void
USART_init( USART_TypeDef *thisUSART )
{
  USART_USART = thisUSART;    // Set global USART_USART varible to point to the desired port
  
  if( thisUSART == USART1 )
  {
    // Enable USART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    // Set Baudrate to 115200 under 8 MHz clock
    thisUSART->BRR = (4<<USART_BRR_DIV_Mantissa_Pos) | (5<<USART_BRR_DIV_Fraction_Pos);
    // Enable Tx, Rx, UART functionality
    thisUSART->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
    // Enable GPIO Port A for PA10=RX, PA9=TX, and enable Alternate Function IO
    RCC->APB2ENR |= ( RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN );
    // Reset all bits for GPIO Part A CNF10[1:0],MODE10[1:0],CNF9[1:0],MODE9[1:0]
    GPIOA->CRH &= ~( 0b11111111 << GPIO_CRH_MODE9_Pos);
    // CNF3=01 (Floating Input), MODE3=00 (Input), CNF2=10 (Alt push-pull), MODE2=00 (10 MHz)
    GPIOA->CRH |=  ( 0b01001001 << GPIO_CRH_MODE9_Pos);
  }
  else if( thisUSART == USART2 )
  {
    // Enable USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Set Baudrate to 115200 under 8 MHz clock
    thisUSART->BRR = (4<<USART_BRR_DIV_Mantissa_Pos) | (5<<USART_BRR_DIV_Fraction_Pos);
    // Enable Tx, Rx, UART functionality
    thisUSART->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
    // Enable GPIO Port A for PA3=RX, PA2=TX, and enable Alternate Function IO
    RCC->APB2ENR |= ( RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN );
    // Reset all bits for GPIO Part A CNF3[1:0],MODE3[1:0],CNF2[1:0],MODE2[1:0]
    GPIOA->CRL &= ~( 0b11111111 << GPIO_CRL_MODE2_Pos);
    // CNF3=01 (Floating Input), MODE3=00 (Input), CNF2=10 (Alt push-pull), MODE2=00 (10 MHz)
    GPIOA->CRL |=  ( 0b01001001 << GPIO_CRL_MODE2_Pos);
  }
  else  // Default to USART3
  {
    // Enable USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    // Set Baudrate to 115,200 under 8 MHz clock
    thisUSART->BRR = (4<<USART_BRR_DIV_Mantissa_Pos) | (5<<USART_BRR_DIV_Fraction_Pos);
    // Enable Tx, Rx, UART functionality
    thisUSART->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
    // Enable GPIO Port B for PB11 = RX, PA10 = TX, and enable Alternate Function IO
    RCC->APB2ENR |= ( RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN );
    // Reset all bits for GPIO Part B CNF11[1:0],MODE11[1:0],CNF10[1:0],MODE10[1:0]
    GPIOB->CRH &= ~( 0b11111111 << GPIO_CRH_MODE10_Pos);
    // CNF11=01 (Floating Input), MODE11=00 (Input), CNF10=10 (Alt push-pull), MODE10=00 (10 MHz)
    GPIOB->CRH |=  ( 0b01001001 << GPIO_CRH_MODE10_Pos);
  }
}


//  void
//  USART_putc(char c)
//  Write a single 8-bit character to the serial port.
void
USART_putc(char c)
{
  while( !(USART_USART->SR & USART_SR_TXE )) ;
  USART_USART->DR = c;
}


//  void
//  USART_puts
//  Output a string to the serial port. String should be null turminated (standard C string).
void
USART_puts( const char *s )
{
    while( *s )
        USART_putc( *s++ );
}


//  void
//  USART_puti( int data, uint8_t base )
//  Writes an integer value to the serial port. "base" determines the base of the output:
//  10 is decimal, 16 is hex, 2 is binary.
void
USART_puti( int data, uint8_t base )
{
  char myString[10];
  itoa( data, myString, base );
  USART_puts( myString );
}


//  void
//  USART_puth( uint32_t number, uint8_t places )
//  Writes a hex value to the serial port. "places" determins the number of digits to
//  output. If the value would exceed this number of digits, then a period . will be
//  printed in place of each digit. Can print from 1 to 8 hex digits.
void
USART_puth( uint32_t number, uint8_t places )
{
  uint8_t  thisDigit;
  uint32_t oob;     // 1 if out of number has more hex digits than "places", 0 if okay

  oob =  number >> ( places *4 ) ;
  for( int32_t x = (places - 1) * 4; x >=0; x -= 4 )
  {
    thisDigit = (number >> x) & 0xF;    // Extract the hex value for this digit
    if( oob )
      USART_putc( '.' );
    else
    {
      if( thisDigit < 10 )
        thisDigit += 0x30;
      else
        thisDigit += 0x37;
    
      USART_putc( thisDigit );
    }
  }
}


// USART_getc
// Waits for a character on the serial port and returns the character
// when received. The character is not displayed on the terminal.
char
USART_getc()
{
  while( !( USART_USART->SR & USART_SR_RXNE ) ) ;   // Wait for serial input
  return USART_USART->DR;                           // Return the character
}


//  char
//  USART_pollc()
//  Poll serial terminal for input. If there is no input, return 0, otherwise return ASCII
//  code for key pressed. Note that this routine will return codes for non-printable characters.
char
USART_pollc()
{
  if( USART_USART->SR & USART_SR_RXNE )
    return USART_USART->DR;
  else
    return 0;
}


//  uint32_t
//  USART_gets( char *inStr, uint32_t bufLen )
//  Gets a string input from the serial port. Characters will be added to the inpString buffer
//  until <Enter> is pressed. Once the string reaches the length of bufLen-1, only the backspace
//  or <Enter> keys will be recognized. The typed in string will be output to the terminal. The
//  inpString will be terminated with a null character (0x00) character. There is no ending
//  linefeed (\n) character attached to the string. Returns the length of the string minus the
//  ending null character.
uint32_t
USART_gets( char *inStr, uint32_t strLen )
{
  uint32_t strPos = 0;    // Track position in string
  uint8_t  oneChar;       // Hold currently entered character for processing
  
  oneChar = USART_getc();                     // Get the first character
  
  while( oneChar != 13 )                      // Loop until <Enter> key is pressed
  {
    // If got a printable character and not at end of the string...
    if( oneChar >= 0x20 && oneChar <= 0x7E && strPos < ( strLen - 1 ) )
    {
      inStr[ strPos++ ] = oneChar;            // Add character to input string
      USART_putc( oneChar );                  // Display character on terminal
    }
    else                                      // Otherwise, got some kind of control character
    {
      if( oneChar == 127 && strPos > 0 )      // Process backspace key
      {
        USART_putc( oneChar );                // Update terminal. 0x7F displays as proper delete 
        strPos--;                             // Decrement bufCnt to effectively erase the
      }                                       // previous character from inpString.
    }

    oneChar = USART_getc();                   // Get next character
  }
  inStr[ strPos ] = 0x00;                     // Terminate string with null character
  return strPos;                              // Return the length of the string not including
}                                             // the end-of-string character

#endif  /*  __STM32F103_CMSIS_USART_LIB_C  */
