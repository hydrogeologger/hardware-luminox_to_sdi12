#ifndef SDI12_SLAVE_H_
#define SDI12_SLAVE_H_

#include <stdint.h>
#include <stdbool.h>

#include "SDI12.h"

/* SDI-12 Data Buffer Size Specification */
// The following data buffer sizes does not include CR+LF and CRC
#define SDI12_VALUE_STR_SIZE 9  // Max number of characters for <value> for aDx!
                                // polarity sign + 7 digits + decimal point = 9
#define SDI12_VALUES_STR_SIZE_35 35 // Data string size (LOW) for aM! aMx!
#define SDI12_VALUES_STR_SIZE_75 75 // Data string size (High) for concurrent,
                                    // continuous, high volume ascii measurement

// SDI-12 Timing Specification
#define SDI12SLAVE_LINE_BREAK_MICROS 12000 // SDI12 "break", 12ms, in microseconds
#define SDI12SLAVE_LINE_MARK_MICROS 8333  // SDI12 "mark", 8.33ms, in microseconds


class SDI12Slave : public SDI12 {
  private:
    /* Static SDI-12 Timing Reference for SDI-12 Slave Device */
    static unsigned long _previous_TCNT; // Stores micros on last ISR() execution
    bool waiting_for_break_ = true; // References device waiting for line break
    bool waiting_for_mark_ = true; // References device waiting for line marking

  public:
    using SDI12::SDI12; // Trivial use of same constructors as SDI12
    ~SDI12Slave(void); // Deconstructor
    bool LineBreakReceived(void);
    bool LineMarkReceived(void);
    void ClearLineMarkingReceived(void);

  private:
    void receiveISR(void) override;
};

#endif