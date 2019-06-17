/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  MarlinSerial1.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  Modified 28 September 2010 by Mark Sproul
  Modified 14 February 2016 by Andreas Hardtung (added tx buffer)

*/

#ifdef serial_port1

#ifndef MARLINSERIAL1_H
#define MARLINSERIAL1_H

#include "MarlinConfig.h"

#ifndef SERIAL_PORT1
  #define SERIAL_PORT1 1
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT1(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                            (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                            (port == 3 && defined(UBRR3H)))

// These are macros to build serial port register names for the selected SERIAL_PORT1 (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT1 == 1 && (!defined(UBRR1H)) // use un-numbered registers if necessary
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
  #define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial1 class (expanded depending on selected serial port)
#define M_UCSRxA1           SERIAL_REGNAME(UCSR,SERIAL_PORT1,A) // defines M_UCSRxA1 to be UCSRnA where n is the serial port number
#define M_UCSRxB1           SERIAL_REGNAME(UCSR,SERIAL_PORT1,B)
#define M_RXENx1            SERIAL_REGNAME(RXEN,SERIAL_PORT1,)
#define M_TXENx1            SERIAL_REGNAME(TXEN,SERIAL_PORT1,)
#define M_TXCx1             SERIAL_REGNAME(TXC,SERIAL_PORT1,)
#define M_RXCIEx1           SERIAL_REGNAME(RXCIE,SERIAL_PORT1,)
#define M_UDREx1            SERIAL_REGNAME(UDRE,SERIAL_PORT1,)
#define M_UDRIEx1           SERIAL_REGNAME(UDRIE,SERIAL_PORT1,)
#define M_UDRx1             SERIAL_REGNAME(UDR,SERIAL_PORT1,)
#define M_UBRRxH1           SERIAL_REGNAME(UBRR,SERIAL_PORT1,H)
#define M_UBRRxL1           SERIAL_REGNAME(UBRR,SERIAL_PORT1,L)
#define M_RXCx1             SERIAL_REGNAME(RXC,SERIAL_PORT1,)
#define M_USARTx_RX_vect1   SERIAL_REGNAME(USART,SERIAL_PORT1,_RX_vect)
#define M_U2Xx1             SERIAL_REGNAME(U2X,SERIAL_PORT1,)
#define M_USARTx_UDRE_vect1 SERIAL_REGNAME(USART,SERIAL_PORT1,_UDRE_vect)

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

// Define constants and variables for buffering serial data.
// Use only 0 or powers of 2 greater than 1
// : [0, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, ...]
#ifndef RX_BUFFER_SIZE1
  #define RX_BUFFER_SIZE1 128
#endif
// 256 is the max TX buffer limit due to uint8_t head and tail.
#ifndef TX_BUFFER_SIZE1
  #define TX_BUFFER_SIZE1 32
#endif

#ifndef USBCON

  #if RX_BUFFER_SIZE1 > 256
    typedef uint16_t ring_buffer_pos_t1;
  #else
    typedef uint8_t ring_buffer_pos_t1;
  #endif

  #if ENABLED(SERIAL_STATS_DROPPED_RX)
    extern uint8_t rx_dropped_bytes1;
  #endif

  #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
    extern ring_buffer_pos_t1 rx_max_enqueued1;
  #endif

  class MarlinSerial1 { //: public Stream

    public:
      MarlinSerial1() {};
      static void begin(const long);
      static void end();
      static int peek(void);
      static int read(void);
      static void flush(void);
      static ring_buffer_pos_t1 available(void);
      static void checkRx(void);
      static void write(const uint8_t c);
      #if TX_BUFFER_SIZE1 > 0
        static uint8_t availableForWrite(void);
        static void flushTX(void);
      #endif
      static void writeNoHandshake(const uint8_t c);
	  
      #if ENABLED(SERIAL_STATS_DROPPED_RX)
        FORCE_INLINE static uint32_t dropped() { return rx_dropped_bytes1; }
      #endif

      #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
        FORCE_INLINE static ring_buffer_pos_t1 rxMaxEnqueued() { return rx_max_enqueued1; }
      #endif

    private:
      static void printNumber(unsigned long, const uint8_t);
      static void printFloat(double, uint8_t);

    public:
      FORCE_INLINE static void write(const char* str) { while (*str) write(*str++); }
      FORCE_INLINE static void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
      FORCE_INLINE static void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
      FORCE_INLINE static void print(const char* str) { write(str); }

      static void print(char, int = BYTE);
      static void print(unsigned char, int = BYTE);
      static void print(int, int = DEC);
      static void print(unsigned int, int = DEC);
      static void print(long, int = DEC);
      static void print(unsigned long, int = DEC);
      static void print(double, int = 2);

      static void println(const String& s);
      static void println(const char[]);
      static void println(char, int = BYTE);
      static void println(unsigned char, int = BYTE);
      static void println(int, int = DEC);
      static void println(unsigned int, int = DEC);
      static void println(long, int = DEC);
      static void println(unsigned long, int = DEC);
      static void println(double, int = 2);
      static void println(void);
  };

 extern MarlinSerial1 customizedSerial1;

#endif // !USBCON

// Use the UART for Bluetooth in AT90USB configurations
#if defined(USBCON) && ENABLED(BLUETOOTH)
  extern HardwareSerial bluetoothSerial;
#endif

#endif // MARLINSERIAL_H

#endif //serial_port1
