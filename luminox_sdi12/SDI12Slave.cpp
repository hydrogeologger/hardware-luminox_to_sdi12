#include "SDI12Slave.h"

#include <Arduino.h>

#include "SDI12_boards.h"
#include "SDI12.h"

// Macro to obtain offset value during overflow,
// i.e _OVERFLOW_OFFSET(uint8_t) = 256
#define _OVERFLOW_OFFSET(a) (1 << (sizeof(a) * 8))


unsigned long SDI12Slave::_previous_TCNT = 0;  // previous RX transition in micros


/**
 * @brief Destroy the SDI12Slave::SDI12Slave object
 * @see SDI12Slave(uint8_t data_pin)
 * @see SDI12Slave(uint8_t data_pin, int eeprom_address)
 */
SDI12Slave::~SDI12Slave(void) {
    // Do nothing
}


/**
 * @brief Gets reference line break.
 * 
 * Needs to call SDI12Slave::ClearLineMarkingReceived() to clear the line status.
 * 
 * @return true SDI12 device has received line break
 * @return false Line break not received
 * 
 * @see LineMarkReceived(void)
 * @see ClearLineMarkingReceived(void)
 */
bool SDI12Slave::LineBreakReceived(void) {
    return !waiting_for_break_;
}


/**
 * @brief Gets reference to line marking, detectable after receiving line break.
 * 
 * Needs to call SDI12Slave::ClearLineMarkingReceived() to clear the line status.
 * 
 * @return true SDI12 device has received line marking
 * @return false Line marking not received
 * 
 * @see LineBreakReceived(void)
 * @see ClearLineMarkingReceived(void)
 */
bool SDI12Slave::LineMarkReceived(void) {
    return !waiting_for_mark_;
}


/**
 * @brief Resets the references of waiting for line marking and line break status for SDI12
 * 
 * @see LineBreakReceived(void)
 * @see LineMarkReceived(void)
 */
void SDI12Slave::ClearLineMarkingReceived(void) {
    waiting_for_break_ = true;
    waiting_for_mark_ = true;
}


/**
 * 
 * @brief Interrupt routine Override of SDI12::receiveISR() to detect line
 * break, line marking and ascii SDI-12 data.
 * 
 * @see available()
 * @see read()
 * @see LineBreakReceived(void)
 * @see LineMarkReceived(void)
 */
void SDI12Slave::receiveISR(void) {
    if (waiting_for_mark_ || waiting_for_break_) {
        // time of this data transition (plus ISR latency)
        unsigned long current_TCNT = micros();
        uint8_t pinLevel = digitalRead(getDataPin());  // current RX data level

        // Serial.print(pinLevel);
        // Serial.print(" : "); Serial.print(current_TCNT);
        // Serial.print(" : "); Serial.print(_previous_TCNT);
        // Serial.print(" : "); Serial.print((current_TCNT - _previous_TCNT));

        if (waiting_for_break_) {
            if (pinLevel == HIGH) {
                return;
            } else if ((current_TCNT - _previous_TCNT) >= SDI12SLAVE_LINE_BREAK_MICROS) {
                waiting_for_break_ = false;
            }
        } else if (waiting_for_mark_ && (pinLevel == HIGH) &&
                ((current_TCNT - _previous_TCNT) >= SDI12SLAVE_LINE_MARK_MICROS)) {
            waiting_for_mark_ = false;
        }

        // Serial.print(" : "); Serial.print(waiting_for_break_);
        // Serial.print(" : "); Serial.print((current_TCNT - _previous_TCNT) >= SDI12SLAVE_LINE_BREAK_MICROS);
        // Serial.print(" : "); Serial.print(waiting_for_mark_);
        // Serial.print(" : "); Serial.print((current_TCNT - _previous_TCNT) >= SDI12SLAVE_LINE_MARK_MICROS);
        // Serial.println("");

        _previous_TCNT = current_TCNT;  // Remember timestamp of this change!
    }
    SDI12::receiveISR();
}


// uint16_t SDI12Slave::TCNTMicros(void){
//     uint8_t m = 0;
//     sdi12timer_t t = READTIME;

// #if defined(TIFRX) && defined(TOVX)
//     if (TIFRX & _BV(TOVX)) {
//         m++;
//         TIFRX = _BV(TOVX); // Clear overflow flag
//     }
// #endif

//     return ( (m << 8) + t) * clockCyclesToMicroseconds(PRESCALE_IN_USE);
// }