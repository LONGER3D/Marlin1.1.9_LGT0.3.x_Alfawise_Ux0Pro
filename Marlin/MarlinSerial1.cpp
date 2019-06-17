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
 * MarlinSerial1.cpp - Hardware serial library for Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Modified 23 November 2006 by David A. Mellis
 * Modified 28 September 2010 by Mark Sproul
 * Modified 14 February 2016 by Andreas Hardtung (added tx buffer)
 * Modified 01 October 2017 by Eduardo José Tagle (added XON/XOFF)
 */

// Disable HardwareSerial.cpp to support chips without a UART (Attiny, etc.)


#include "MarlinConfig.h"

#ifdef serial_port1

#if !defined(USBCON) && (defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H))

  #include "MarlinSerial1.h"
  #include "Marlin.h"
#if ENABLED(EMERGENCY_PARSER)
#include "emergency_parser.h"
#endif

  struct ring_buffer_r1 {
    unsigned char buffer[RX_BUFFER_SIZE1];
    volatile ring_buffer_pos_t1 head, tail;
  };

  #if TX_BUFFER_SIZE1 > 0
    struct ring_buffer_t1 {
      unsigned char buffer[TX_BUFFER_SIZE1];
      volatile uint8_t head, tail;
    };
  #endif

  #if UART_PRESENT1(SERIAL_PORT1)
    ring_buffer_r1 rx_buffer1 = { { 0 }, 0, 0 };
    #if TX_BUFFER_SIZE1 > 0
      ring_buffer_t1 tx_buffer1 = { { 0 }, 0, 0 };
      static bool _written1;
    #endif
  #endif

  #if ENABLED(SERIAL_XON_XOFF)
    constexpr uint8_t XON_XOFF_CHAR_SENT1 = 0x80;  // XON / XOFF Character was sent
    constexpr uint8_t XON_XOFF_CHAR_MASK1 = 0x1F;  // XON / XOFF character to send
    // XON / XOFF character definitions
    constexpr uint8_t XON_CHAR1  = 17;
    constexpr uint8_t XOFF_CHAR1 = 19;
    uint8_t xon_xoff_state1 = XON_XOFF_CHAR_SENT1 | XON_CHAR1;
  #endif

  #if ENABLED(SERIAL_STATS_DROPPED_RX)
    uint8_t rx_dropped_bytes1 = 0;
  #endif

  #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
    ring_buffer_pos_t1 rx_max_enqueued1 = 0;
  #endif

  #if ENABLED(EMERGENCY_PARSER1)

    #include "stepper.h"
    #include "language.h"

    // Currently looking for: M108, M112, M410
    // If you alter the parser please don't forget to update the capabilities in Conditionals_post.h

    FORCE_INLINE void emergency_parser1(const unsigned char c) {

      static e_parser_state state1 = state_RESET;

      switch (state1) {
        case state_RESET:
          switch (c) {
            case ' ': break;
            case 'N': state1 = state_N;      break;
            case 'M': state1 = state_M;      break;
            default: state1 = state_IGNORE;
          }
          break;

        case state_N:
          switch (c) {
            case '0': case '1': case '2':
            case '3': case '4': case '5':
            case '6': case '7': case '8':
            case '9': case '-': case ' ':   break;
            case 'M': state1 = state_M;      break;
            default:  state1 = state_IGNORE;
          }
          break;

        case state_M:
          switch (c) {
            case ' ': break;
            case '1': state1 = state_M1;     break;
            case '4': state1 = state_M4;     break;
            default: state1 = state_IGNORE;
          }
          break;

        case state_M1:
          switch (c) {
            case '0': state1 = state_M10;    break;
            case '1': state1 = state_M11;    break;
            default: state1 = state_IGNORE;
          }
          break;

        case state_M10:
          state1 = (c == '8') ? state_M108 : state_IGNORE;
          break;

        case state_M11:
          state1 = (c == '2') ? state_M112 : state_IGNORE;
          break;

        case state_M4:
          state1 = (c == '1') ? state_M41 : state_IGNORE;
          break;

        case state_M41:
          state1 = (c == '0') ? state_M410 : state_IGNORE;
          break;

        case state_IGNORE:
          if (c == '\n') state1 = state_RESET;
          break;

        default:
          if (c == '\n') {
            switch (state1) {
              case state_M108:
                wait_for_user = wait_for_heatup = false;
                break;
              case state_M112:
				#ifdef LGT_MAC
				  kill_type = M112_KILL;
				#endif // LGT_MAC

                kill(PSTR(MSG_KILLED));
                break;
              case state_M410:
                quickstop_stepper();
                break;
              default:
                break;
            }
            state1 = state_RESET;
          }
      }
    }

  #endif // EMERGENCY_PARSER

  FORCE_INLINE void store_rxd_char1() {
    const ring_buffer_pos_t1 h = rx_buffer1.head,
                            i = (ring_buffer_pos_t1)(h + 1) & (ring_buffer_pos_t1)(RX_BUFFER_SIZE1 - 1);

    // If the character is to be stored at the index just before the tail
    // (such that the head would advance to the current tail), the buffer is
    // critical, so don't write the character or advance the head.
    const char c = M_UDRx1;
    if (i != rx_buffer1.tail) {
      rx_buffer1.buffer[h] = c;
      rx_buffer1.head = i;
    }
    else {
      #if ENABLED(SERIAL_STATS_DROPPED_RX)
        if (!++rx_dropped_bytes1) ++rx_dropped_bytes1;
      #endif
    }

    #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
      // calculate count of bytes stored into the RX buffer
      ring_buffer_pos_t1 rx_count = (ring_buffer_pos_t1)(rx_buffer1.head - rx_buffer1.tail) & (ring_buffer_pos_t1)(RX_BUFFER_SIZE1 - 1);
      // Keep track of the maximum count of enqueued bytes
      NOLESS(rx_max_enqueued1, rx_count);
    #endif

    #if ENABLED(SERIAL_XON_XOFF)

      // for high speed transfers, we can use XON/XOFF protocol to do
      // software handshake and avoid overruns.
      if ((xon_xoff_state1 & XON_XOFF_CHAR_MASK1) == XON_CHAR1) {

        // calculate count of bytes stored into the RX buffer
        ring_buffer_pos_t1 rx_count = (ring_buffer_pos_t1)(rx_buffer1.head - rx_buffer1.tail) & (ring_buffer_pos_t1)(RX_BUFFER_SIZE1 - 1);

        // if we are above 12.5% of RX buffer capacity, send XOFF before
        // we run out of RX buffer space .. We need 325 bytes @ 250kbits/s to
        // let the host react and stop sending bytes. This translates to 13mS
        // propagation time.
        if (rx_count >= (RX_BUFFER_SIZE1) / 8) {
          // If TX interrupts are disabled and data register is empty,
          // just write the byte to the data register and be done. This
          // shortcut helps significantly improve the effective datarate
          // at high (>500kbit/s) bitrates, where interrupt overhead
          // becomes a slowdown.
          if (!TEST(M_UCSRxB1, M_UDRIEx1) && TEST(M_UCSRxA1, M_UDREx1)) {
            // Send an XOFF character
            M_UDRx1 = XOFF_CHAR1;
            // clear the TXC bit -- "can be cleared by writing a one to its bit
            // location". This makes sure flush() won't return until the bytes
            // actually got written
            SBI(M_UCSRxA1, M_TXCx1);
            // And remember it was sent
            xon_xoff_state1 = XOFF_CHAR1 | XON_XOFF_CHAR_SENT1;
          }
          else {
            // TX interrupts disabled, but buffer still not empty ... or
            // TX interrupts enabled. Reenable TX ints and schedule XOFF
            // character to be sent
            #if TX_BUFFER_SIZE1 > 0
              SBI(M_UCSRxB1, M_UDRIEx1);
              xon_xoff_state1 = XOFF_CHAR1;
            #else
              // We are not using TX interrupts, we will have to send this manually
              while (!TEST(M_UCSRxA1, M_UDREx1)) {/* nada */}
              M_UDRx1 = XOFF_CHAR1;
              // And remember we already sent it
              xon_xoff_state1 = XOFF_CHAR1 | XON_XOFF_CHAR_SENT1;
            #endif
          }
        }
      }
    #endif // SERIAL_XON_XOFF

    #if ENABLED(EMERGENCY_PARSER1)
      emergency_parser1(c);
    #endif
  }

  #if TX_BUFFER_SIZE1 > 0

    FORCE_INLINE void _tx_udr_empty_irq1(void) {
      // If interrupts are enabled, there must be more data in the output
      // buffer.

      #if ENABLED(SERIAL_XON_XOFF)
        // Do a priority insertion of an XON/XOFF char, if needed.
        const uint8_t state1 = xon_xoff_state1;
        if (!(state1 & XON_XOFF_CHAR_SENT1)) {
          M_UDRx1 = state1 & XON_XOFF_CHAR_MASK1;
          xon_xoff_state1 = state1 | XON_XOFF_CHAR_SENT1;
        }
        else
      #endif
      { // Send the next byte
        const uint8_t t = tx_buffer1.tail, c = tx_buffer1.buffer[t];
        tx_buffer1.tail = (t + 1) & (TX_BUFFER_SIZE1 - 1);
        M_UDRx1 = c;
      }

      // clear the TXC bit -- "can be cleared by writing a one to its bit
      // location". This makes sure flush() won't return until the bytes
      // actually got written
      SBI(M_UCSRxA1, M_TXCx1);

      // Disable interrupts if the buffer is empty
      if (tx_buffer1.head == tx_buffer1.tail)
        CBI(M_UCSRxB1, M_UDRIEx1);
    }

    #ifdef M_USARTx_UDRE_vect1
      ISR(M_USARTx_UDRE_vect1) { _tx_udr_empty_irq1(); }
    #endif

  #endif // TX_BUFFER_SIZE

  #ifdef M_USARTx_RX_vect1
    ISR(M_USARTx_RX_vect1) { store_rxd_char1(); }
  #endif

  // Public Methods

  void MarlinSerial1::begin(const long baud) {
    uint16_t baud_setting;
    bool useU2X = true;

    #if F_CPU == 16000000UL && SERIAL_PORT1 == 1
      // Hard-coded exception for compatibility with the bootloader shipped
      // with the Duemilanove and previous boards, and the firmware on the
      // 8U2 on the Uno and Mega 2560.
      if (baud == 57600) useU2X = false;
    #endif

    if (useU2X) {
      M_UCSRxA1 = _BV(M_U2Xx1);
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else {
      M_UCSRxA1 = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    M_UBRRxH1 = baud_setting >> 8;
    M_UBRRxL1 = baud_setting;

    SBI(M_UCSRxB1, M_RXENx1);
    SBI(M_UCSRxB1, M_TXENx1);
    SBI(M_UCSRxB1, M_RXCIEx1);
    #if TX_BUFFER_SIZE1 > 0
      CBI(M_UCSRxB1, M_UDRIEx1);
      _written1 = false;
    #endif
  }

  void MarlinSerial1::end() {
    CBI(M_UCSRxB1, M_RXENx1);
    CBI(M_UCSRxB1, M_TXENx1);
    CBI(M_UCSRxB1, M_RXCIEx1);
    CBI(M_UCSRxB1, M_UDRIEx1);
  }

  void MarlinSerial1::checkRx(void) {
    if (TEST(M_UCSRxA1, M_RXCx1)) {
      CRITICAL_SECTION_START;
        store_rxd_char1();
      CRITICAL_SECTION_END;
    }
  }

  int MarlinSerial1::peek(void) {
    CRITICAL_SECTION_START;
      const int v = rx_buffer1.head == rx_buffer1.tail ? -1 : rx_buffer1.buffer[rx_buffer1.tail];
    CRITICAL_SECTION_END;
    return v;
  }

  int MarlinSerial1::read(void) {
    int v;
    CRITICAL_SECTION_START;
      const ring_buffer_pos_t1 t = rx_buffer1.tail;
      if (rx_buffer1.head == t)
        v = -1;
      else {
        v = rx_buffer1.buffer[t];
        rx_buffer1.tail = (ring_buffer_pos_t1)(t + 1) & (RX_BUFFER_SIZE1 - 1);

        #if ENABLED(SERIAL_XON_XOFF)
          if ((xon_xoff_state1 & XON_XOFF_CHAR_MASK1) == XOFF_CHAR1) {
            // Get count of bytes in the RX buffer
            ring_buffer_pos_t1 rx_count = (ring_buffer_pos_t1)(rx_buffer1.head - rx_buffer1.tail) & (ring_buffer_pos_t1)(RX_BUFFER_SIZE1 - 1);
            // When below 10% of RX buffer capacity, send XON before
            // running out of RX buffer bytes
            if (rx_count < (RX_BUFFER_SIZE1) / 10) {
              xon_xoff_state1 = XON_CHAR1 | XON_XOFF_CHAR_SENT1;
              CRITICAL_SECTION_END;       // End critical section before returning!
              writeNoHandshake(XON_CHAR1);
              return v;
            }
          }
        #endif
      }
    CRITICAL_SECTION_END;
    return v;
  }

  ring_buffer_pos_t1 MarlinSerial1::available(void) {
    CRITICAL_SECTION_START;
      const ring_buffer_pos_t1 h = rx_buffer1.head, t = rx_buffer1.tail;
    CRITICAL_SECTION_END;
    return (ring_buffer_pos_t1)(RX_BUFFER_SIZE1 + h - t) & (RX_BUFFER_SIZE1 - 1);
  }

  void MarlinSerial1::flush(void) {
    // Don't change this order of operations. If the RX interrupt occurs between
    // reading rx_buffer1_head and updating rx_buffer1_tail, the previous rx_buffer1_head
    // may be written to rx_buffer1_tail, making the buffer appear full rather than empty.
    CRITICAL_SECTION_START;
      rx_buffer1.head = rx_buffer1.tail;
    CRITICAL_SECTION_END;

    #if ENABLED(SERIAL_XON_XOFF)
      if ((xon_xoff_state1 & XON_XOFF_CHAR_MASK1) == XOFF_CHAR1) {
        xon_xoff_state1 = XON_CHAR1 | XON_XOFF_CHAR_SENT1;
        writeNoHandshake(XON_CHAR1);
      }
    #endif
  }

  #if TX_BUFFER_SIZE1 > 0
    uint8_t MarlinSerial1::availableForWrite(void) {
      CRITICAL_SECTION_START;
        const uint8_t h = tx_buffer1.head, t = tx_buffer1.tail;
      CRITICAL_SECTION_END;
      return (uint8_t)(TX_BUFFER_SIZE1 + h - t) & (TX_BUFFER_SIZE1 - 1);
    }

    void MarlinSerial1::write(const uint8_t c) {
      #if ENABLED(SERIAL_XON_XOFF)
        const uint8_t state1 = xon_xoff_state1;
        if (!(state1 & XON_XOFF_CHAR_SENT1)) {
          // Send 2 chars: XON/XOFF, then a user-specified char
          writeNoHandshake(state1 & XON_XOFF_CHAR_MASK1);
          xon_xoff_state1 = state1 | XON_XOFF_CHAR_SENT1;
        }
      #endif
      writeNoHandshake(c);
    }

    void MarlinSerial1::writeNoHandshake(const uint8_t c) {
      _written1 = true;
      CRITICAL_SECTION_START;
        bool emty = (tx_buffer1.head == tx_buffer1.tail);
      CRITICAL_SECTION_END;

      // If the buffer and the data register is empty, just write the byte
      // to the data register and be done. This shortcut helps
      // significantly improve the effective datarate at high (>
      // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
      if (emty && TEST(M_UCSRxA1, M_UDREx1)) {
        CRITICAL_SECTION_START;
          M_UDRx1 = c;
          SBI(M_UCSRxA1, M_TXCx1);
        CRITICAL_SECTION_END;
        return;
      }
      const uint8_t i = (tx_buffer1.head + 1) & (TX_BUFFER_SIZE1 - 1);

      // If the output buffer is full, there's nothing for it other than to
      // wait for the interrupt handler to empty it a bit
      while (i == tx_buffer1.tail) {
        if (!TEST(SREG, SREG_I)) {
          // Interrupts are disabled, so we'll have to poll the data
          // register empty flag ourselves. If it is set, pretend an
          // interrupt has happened and call the handler to free up
          // space for us.
          if (TEST(M_UCSRxA1, M_UDREx1))
            _tx_udr_empty_irq1();
        }
        else {
          // nop, the interrupt handler will free up space for us
        }
      }

      tx_buffer1.buffer[tx_buffer1.head] = c;
      { CRITICAL_SECTION_START;
          tx_buffer1.head = i;
          SBI(M_UCSRxB1, M_UDRIEx1);
        CRITICAL_SECTION_END;
      }
      return;
    }

    void MarlinSerial1::flushTX(void) {
      // TX
      // If we have never written a byte, no need to flush. This special
      // case is needed since there is no way to force the TXC (transmit
      // complete) bit to 1 during initialization
      if (!_written1)
        return;

      while (TEST(M_UCSRxB1, M_UDRIEx1) || !TEST(M_UCSRxA1, M_TXCx1)) {
        if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB1, M_UDRIEx1))
          // Interrupts are globally disabled, but the DR empty
          // interrupt should be enabled, so poll the DR empty flag to
          // prevent deadlock
          if (TEST(M_UCSRxA1, M_UDREx1))
            _tx_udr_empty_irq1();
      }
      // If we get here, nothing is queued anymore (DRIE is disabled) and
      // the hardware finished tranmission (TXC is set).
    }

  #else // TX_BUFFER_SIZE == 0

    void MarlinSerial1::write(const uint8_t c) {
      #if ENABLED(SERIAL_XON_XOFF)
        // Do a priority insertion of an XON/XOFF char, if needed.
        const uint8_t state1 = xon_xoff_state1;
        if (!(state1 & XON_XOFF_CHAR_SENT1)) {
          writeNoHandshake(state1 & XON_XOFF_CHAR_MASK1);
          xon_xoff_state1 = state1 | XON_XOFF_CHAR_SENT1;
        }
      #endif
      writeNoHandshake(c);
    }

    void MarlinSerial1::writeNoHandshake(uint8_t c) {
      while (!TEST(M_UCSRxA1, M_UDREx1)) {/* nada */}
      M_UDRx1 = c;    
    }

  #endif // TX_BUFFER_SIZE == 0

  /**
   * Imports from print.h
   */

  void MarlinSerial1::print(char c, int base) {
    print((long)c, base);
  }

  void MarlinSerial1::print(unsigned char b, int base) {
    print((unsigned long)b, base);
  }

  void MarlinSerial1::print(int n, int base) {
    print((long)n, base);
  }

  void MarlinSerial1::print(unsigned int n, int base) {
    print((unsigned long)n, base);
  }

  void MarlinSerial1::print(long n, int base) {
    if (base == 0)
      write(n);
    else if (base == 10) {
      if (n < 0) {
        print('-');
        n = -n;
      }
      printNumber(n, 10);
    }
    else
      printNumber(n, base);
  }

  void MarlinSerial1::print(unsigned long n, int base) {
    if (base == 0) write(n);
    else printNumber(n, base);
  }

  void MarlinSerial1::print(double n, int digits) {
    printFloat(n, digits);
  }

  void MarlinSerial1::println(void) {
    print('\r');
    print('\n');
  }

  void MarlinSerial1::println(const String& s) {
    print(s);
    println();
  }

  void MarlinSerial1::println(const char c[]) {
    print(c);
    println();
  }

  void MarlinSerial1::println(char c, int base) {
    print(c, base);
    println();
  }

  void MarlinSerial1::println(unsigned char b, int base) {
    print(b, base);
    println();
  }

  void MarlinSerial1::println(int n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial1::println(unsigned int n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial1::println(long n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial1::println(unsigned long n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial1::println(double n, int digits) {
    print(n, digits);
    println();
  }

  // Private Methods

  void MarlinSerial1::printNumber(unsigned long n, uint8_t base) {
    if (n) {
      unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
      int8_t i = 0;
      while (n) {
        buf[i++] = n % base;
        n /= base;
      }
      while (i--)
        print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else
      print('0');
  }

  void MarlinSerial1::printFloat(double number, uint8_t digits) {
    // Handle negative numbers
    if (number < 0.0) {
      print('-');
      number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding *= 0.1;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits) {
      print('.');
      // Extract digits from the remainder one at a time
      while (digits--) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
      }
    }
  }

  // Preinstantiate
 MarlinSerial1 customizedSerial1;

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For AT90USB targets use the UART for BT interfacing
#if defined(USBCON) && ENABLED(BLUETOOTH)
  HardwareSerial bluetoothSerial;
#endif

#endif //serial_port1
