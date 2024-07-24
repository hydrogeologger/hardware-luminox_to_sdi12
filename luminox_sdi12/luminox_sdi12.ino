/**
 * @file h_SDI-12_slave_implementation.ino
 * @copyright (c) 2013-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 * @date 2016
 * @author D. Wasielewski
 *
 * @brief Example H:  Using SDI-12 in Slave Mode
 *
 * Example sketch demonstrating how to implement an arduino as a slave on an SDI-12 bus.
 * This may be used, for example, as a middleman between an I2C sensor and an SDI-12
 * datalogger.
 *
 * Note that an SDI-12 slave must respond to M! or C! with the number of values it will
 * report and the max time until these values will be available.  This example uses 9
 * values available in 21 s, but references to these numbers and the output array size
 * and datatype should be changed for your specific application.
 *
 * D. Wasielewski, 2016
 * Builds upon work started by:
 * https://github.com/jrzondagh/AgriApps-SDI-12-Arduino-Sensor
 * https://github.com/Jorge-Mendes/Agro-Shield/tree/master/SDI-12ArduinoSensor
 *
 * Suggested improvements:
 *  - Get away from memory-hungry arduino String objects in favor of char buffers
 *  - Make an int variable for the "number of values to report" instead of the
 *    hard-coded 9s interspersed throughout the code
 */
#include <EEPROM.h>

#include <SDI12Node.h>
#include <SDI12CRC.h>
#include <SDI12Sensor.h>


#define DATA_PIN 2  /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1 /*!< The sensor power pin (or -1 if not switching power) */

#define MEASUREMENT_ARRAY_MAX_SIZE 5 // Max size of floats/double array to hold sensor data
#define MEASUREMENT_STR_ARRAY_MAX_ELEMENT 10 // Max number of array elements for 0Dx! string response

#define SDI12SENSOR_SDI12_PROTOCOL "13"  // Respresent v1.3
#define SDI12SENSOR_COMPANY "UQGEC   "  // 8 Charactors depicting company name
#define SDI12SENSOR_MODEL "OXLUM "  // 6 Characters specifying sensor model
#define SDI12SENSOR_VERSION "010"  // 3 characters specifying sensor version, Represent x.x.x
#define SDI12SENSOR_OTHER_INFO "LUMINOX"  // (optional) up to 13 char for serial or other sensor info

// EEPROM Mapping, EEPROM Address reference
typedef enum EEPROMMap_e: uint8_t {
    kEEPROMSensorAddress = 0,      // 1 Byte for sensor address
    kEEPROMConfigFlag = 1    // 1 Byte for Configuration Flag
} EEPROMMap_e;

#define STATE_LOW_POWER 0
#define STATE_DO_SOMETHING 1
#define STATE_SERIAL_PASSTHROUGH 2

typedef enum ExtendedCommand_e: uint8_t {
    kEXTModeSelect      = 1,    // Sensor mode select, aXMx!
    kEXTPassThrough     = 2,    // Pass Through Mode, aXPAS!
    kEXTConfiguration   = 3     // Sensor Configuration, aXU
} ExtendedCommand_e;


// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12Node slaveSDI12(DATA_PIN);
SDI12Sensor sensor(SDI12SENSOR_DEFAULT_ADDR);

static uint8_t measurement_count = 0;

char SelectLuminoxMode(int8_t mode) {
    char mode_str[5];
    switch (mode) {
        case 0:
            strcpy(mode_str, "M 0"); // Stream mode
            break;
        case 1:
            strcpy(mode_str, "M 1"); // Poll mode
            break;
        case 2:
            strcpy(mode_str, "M 2"); // Off
            break;
        default:
            strcpy(mode_str, "M");
            break;
    }
    while (Serial.available()) { Serial.read(); } // Empty input buffer
    Serial.println(mode_str);
    String response = Serial.readStringUntil('\n');
    return response[3];
}

uint8_t ProcessLuminoxPayloadStream(float *measurementValues) {
    uint8_t count = 0; // Number of data values detected
    String luminoReply = "";
    char inChar;
    while (Serial.available()) {
        inChar = Serial.read();
        if ((inChar >= 48 && inChar <= 57) || // 0-9
                (inChar >= 43 && inChar <= 46) // [+,-.]
                ) {
            luminoReply += inChar;
        } else if (inChar == 'O' || inChar == 'T' || inChar == 'P' || inChar == '%' || inChar == 'e') {
            if (count > 0) { luminoReply += ','; }
            count++;
        } else if (inChar == '\r' || inChar == '\n') {
            break;
        }
        delay(2);
    }

    if (count == 0) { return count; }

    int8_t charPos = 0;
    uint8_t i = 0;
    do {
        measurementValues[i++] = luminoReply.substring(charPos).toFloat();
        // measurementValues[i++] = charPos;
        charPos = luminoReply.indexOf(",", charPos) + 1;
    } while (charPos > 0);

    return count;
}

void pollSensor(float *measurementValues, char command) {
    // Empty rx buffer
    while (Serial.available()) {
        Serial.read();
    }

    // Send polling command
    Serial.println(command);
    Serial.flush();

    // delay(1000);
    // Blocking delay until a character exist in serial buffer
    while (!Serial.available());

    // Zero remaining
    for (uint8_t i = ProcessLuminoxPayloadStream(measurementValues);
            i < MEASUREMENT_ARRAY_MAX_SIZE; i++) {
        measurementValues[i] = 0;
    }
}

void pollOxygenSensorInfo(String *dValues) {
    // Empty rx buffer
    while (Serial.available()) {
        Serial.read();
    }

    // Poll Oxygen
    Serial.println(F("# 0")); // Send command to initiate polling mode
    Serial.flush();
    dValues[0] = Serial.readStringUntil('\r');

    Serial.println(F("# 1")); // Send command to initiate polling mode
    Serial.flush();
    dValues[1] = Serial.readStringUntil('\r');

    Serial.println(F("# 2")); // Send command to initiate polling mode
    Serial.flush();
    dValues[2] = Serial.readStringUntil('\r');

    for (int i=0; i <= 2; i++) {
        dValues[i].trim();
    }
}

void ExtendedCommandRules(const char* cmd, SDI12Command *parsed_cmd) {
    if (parsed_cmd->primary != kExtended) { return; }

    if (cmd[0] == 'M' && BITS_IS_SET(parsed_cmd->flags, CMD_IS_END_FLAG)
            && (isdigit(cmd[1]) || cmd[1] == '\0' || cmd[1] == '!')) {
        // aXMx!, aXM!
        parsed_cmd->secondary = kEXTModeSelect;
        if (!BITS_IS_SET(parsed_cmd->flags, CMD_PARAM1_FLAG)) {
            parsed_cmd->param1 = -1; // aXM!
        }
    } else if (cmd[0] == 'U' &&
            (cmd[1] == ',' || BITS_IS_SET(parsed_cmd->flags, CMD_IS_END_FLAG))) {
        // aXU
        parsed_cmd->secondary = kEXTConfiguration;
    } else if (BITS_IS_SET(parsed_cmd->flags, CMD_IS_END_FLAG) &&
            !strcmp("PAS", cmd)) {
        // aXPAS!
        parsed_cmd->secondary = kEXTPassThrough;
    } else {
        parsed_cmd->primary = kUnknown;
    }
}

void parseSdi12Cmd(String command, SDI12Command *parsed_cmd) {
    /* Ingests a command from an SDI-12 master, sends the applicable response, and
     * (when applicable) sets a flag to initiate a measurement
     */

    // First char of command is always either (a) the address of the device being
    // probed OR (b) a '?' for address query.
    // Do nothing if this command is addressed to a different device
    char *ext_cmd = nullptr;
    *parsed_cmd = SDI12Command::ParseCommand(command.c_str(), sensor.Address(), &ext_cmd);
    if (parsed_cmd->address == sensor.Address() || parsed_cmd->address == '?') {
        sensor.SetActive();
    } else {
        return;
    }

    ExtendedCommandRules(ext_cmd, parsed_cmd);

    // If execution reaches this point, the slave should respond with something in
    // the form:   <address><responseStr><Carriage Return><Line Feed>
    // The following if-switch-case block determines what to put into <responseStr>,
    // and the full response will be constructed afterward. For '?!' (address query)
    // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
    String responseStr = "";
    responseStr = SDI12Sensor::LastActive()->Address();
    switch ((SDI12SensorCommand_e)parsed_cmd->primary) {
        case kAddressQuery: // Fall Through
        case kAcknowledge:
            // Do not need to do anything
            break;
        case kIdentification:
            if (parsed_cmd->secondary == kUnknown) {
                // Identify command
                // Slave should respond with ID message: 2-char SDI-12 version + 8-char
                // company name + 6-char sensor model + 3-char sensor version + 0-13
                // char S/N
                responseStr += SDI12SENSOR_SDI12_PROTOCOL \
                        SDI12SENSOR_COMPANY \
                        SDI12SENSOR_MODEL \
                        SDI12SENSOR_VERSION \
                        SDI12SENSOR_OTHER_INFO;
            } else {
                if (parsed_cmd->param2 <= 0) {
                    parsed_cmd->Reverse(); // Swap primary and secondary command around.
                }
                SDI12Sensor::LastActive()->SetState(STATE_DO_SOMETHING); // Handle in main loop
                return;
            }
            break;
        case kAddressChange:
            // Change address command
            // Slave should respond with blank message (just the [new] address +
            // <CR> + <LF>)
            if (SDI12Sensor::LastActive()->SetAddress(parsed_cmd->param1) &&
                    SDI12Sensor::LastActive()->Address() != (char)EEPROM.read(kEEPROMSensorAddress)) {
                EEPROM.update(kEEPROMSensorAddress, SDI12Sensor::LastActive()->Address());
            }
            responseStr = SDI12Sensor::LastActive()->Address();
            break;
        case kUnknown:
            // For DEBUG
            responseStr += "UNK";
            break;
        case kExtended:
            switch ((ExtendedCommand_e)parsed_cmd->secondary) {
                case kEXTConfiguration:
                    responseStr += "XU,";
                    break;
                case kEXTPassThrough:
                    responseStr += ext_cmd;
                    sensor.SetActive(false);
                    SDI12Sensor::SetState(STATE_SERIAL_PASSTHROUGH);
                    break;
                case kEXTModeSelect:
                    responseStr += "XM=";
                    responseStr += SelectLuminoxMode(parsed_cmd->param1);
                    break;
            }
            break;
        default:
            SDI12Sensor::SetState(STATE_DO_SOMETHING);
            return; // Handle everything else in loop
    }

    if (SDI12Sensor::IsSetLastActive()) {
        // Issue the response speficied in the switch-case structure above.
        responseStr += "\r\n";
        slaveSDI12.sendResponse(responseStr);
        SDI12Sensor::ClearLastActive();
    }
}

void EmptyOutputSDIBuffer(String *dValues) {
    for (size_t i = 0; i < (sizeof(dValues) / sizeof(dValues[0])); i++) {
        dValues[i] = "";
    }
}

bool DetectedValidAddress(const char address) {
    int avail = slaveSDI12.available();
    if (avail == 0) { return false; }
    char charReceived;
    char *commandReceived = (char*) malloc((sizeof(char) * avail) + 1);
    bool valid_address = false;

    for (int i = 0; i < avail; i++) {
        charReceived = slaveSDI12.read();
        if (charReceived == '!') {
            commandReceived[i] = '\0';
            slaveSDI12.clearBuffer();
            slaveSDI12.ClearLineMarkingReceived();
            break;
        } else {
            commandReceived[i] = charReceived;
        }
    }

    valid_address = (SDI12Command::ParseCommand(commandReceived, address).address == address);
    free(commandReceived);
    return valid_address;
}

uint8_t formatOutputSDI(float* measurementValues, String* dValues, uint8_t data_count, unsigned int maxChar) {
    uint8_t count = 0;
    /* Ingests an array of floats and produces Strings in SDI-12 output format */

    dValues[0] = "";
    int j = 0;
    char valStr[SDI12_VALUE_STR_SIZE+1] = "";
    uint8_t valStr_len = 0;

    // upper limit on i should be number of elements in measurementValues
    for (int i = 0; i < data_count; i++) {
        // Read float value "i" as a String with 6 deceimal digits
        // (NOTE: SDI-12 specifies max of 7 digits per value; we can only use 6
        //  decimal place precision if integer part is one digit)
        valStr_len = dtoa(measurementValues[i], valStr, 6, SDI12_VALUE_STR_SIZE);
        // Append dValues[j] if it will not exceed 35 (aM!) or 75 (aC!) characters
        if (dValues[j].length() + valStr_len < maxChar) {
            dValues[j] += valStr;
        } else {
            // Start a new dValues "line" if appending would exceed 35/75 characters
            dValues[++j] = valStr;
        }
    }

    if (data_count > 0) {
        count = j + 1;
    }

    // Fill rest of dValues with blank strings
    while (j < MEASUREMENT_ARRAY_MAX_SIZE) { dValues[++j] = ""; }
    return count;
}


void setup() {
    slaveSDI12.begin();
    Serial.begin(9600);
    Serial.setTimeout(1000);
    // delay(500);
    slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message

    SelectLuminoxMode(1);
    Serial.flush();

    // Update sensor address from EEPROM if available and valid
    if (!sensor.SetAddress((char)EEPROM.read(kEEPROMSensorAddress))) {
        sensor.SetAddress(SDI12SENSOR_DEFAULT_ADDR);
    };
}

void loop() {
    static float measurementValues[MEASUREMENT_ARRAY_MAX_SIZE];  // floats to hold sensor data
    static String dValues[MEASUREMENT_STR_ARRAY_MAX_ELEMENT];  // String objects to hold the responses to aD0!-aD9! commands
    static String commandReceived = "";  // String object to hold the incoming command
    SDI12Command parsed_cmd;
    String response = "";

    // PASS THROUGH MODE
    if (SDI12Sensor::state() == STATE_SERIAL_PASSTHROUGH) {
        PassThroughMode();
    }

    // If a byte is available, an SDI message is queued up. Read in the entire message
    // before proceding.  It may be more robust to add a single character per loop()
    // iteration to a static char buffer; however, the SDI-12 spec requires a precise
    // response time, and this method is invariant to the remaining loop() contents.
    if (slaveSDI12.available() < 0) {
        // Buffer is full; clear
        slaveSDI12.clearBuffer();
    }
    while (slaveSDI12.available() > 0) {
        char charReceived = slaveSDI12.read();
        // Character '!' indicates the end of an SDI-12 command; if the current
        // character is '!', stop listening and respond to the command
        if (charReceived == '!') {
            // Command string is completed; do something with it
            parseSdi12Cmd(commandReceived, &parsed_cmd);
            slaveSDI12.forceListen(); // Force listen if command is not recognized
            // Clear command string to reset for next command
            commandReceived = "";
            // '!' should be the last available character anyway, but exit the "for"
            // loop just in case there are any stray characters
            slaveSDI12.ClearLineMarkingReceived(); // Clear detected break marking
            slaveSDI12.clearBuffer();
            break;
        } else if (charReceived >= 32 && charReceived <= 126) {
            // For printable ascii characters only.
            // If the current character is anything but '!', it is part of the command
            // string.  Append the commandReceived String object.
            // Append command string with new character
            commandReceived += String(charReceived);
        }
        delay(9);  // 1 character ~ 8.33 ms @ 1200 baud
    }

    ProcessLuminoxPayloadStream(measurementValues);

    if (SDI12Sensor::state() != STATE_DO_SOMETHING || !sensor.IsActive()) {
        return;
    }

    response = sensor.Address();
    // Do whatever the sensor is supposed to do here
    switch (parsed_cmd.primary) {
        case kMeasurement: {
            // For aM!, aMx!, aMCx!
            sensor.SetCrcRequest(parsed_cmd.CRC());
            measurement_count = 0;
            if (parsed_cmd.param1 == 0) {
                measurement_count = 5;
            } else if (parsed_cmd.param1 > 0 && parsed_cmd.param1 <= MEASUREMENT_ARRAY_MAX_SIZE) {
                measurement_count = 1;
            }

            // Response should be in following format atttn<CR><LF>
            if (measurement_count > 0) {
                response += "002";
                response += measurement_count;
            } else {
                response += "0000";
            }
            response += "\r\n";
            slaveSDI12.sendResponse(response);

            if (parsed_cmd.secondary == kIdentification) {
                sensor.SetActive(false);
                break;
            }

            if (measurement_count > 0) {
                switch (parsed_cmd.param1) {
                    case 1:
                        pollSensor(measurementValues, 'O'); // Oxygen (ppO2)
                        break;
                    case 2:
                        pollSensor(measurementValues, '%'); // Oxygen (%)
                        break;
                    case 3:
                        pollSensor(measurementValues, 'T'); // Temperature
                        break;
                    case 4:
                        pollSensor(measurementValues, 'P'); // Barometric Pressure
                        break;
                    case 5:
                        pollSensor(measurementValues, 'e'); // Sensor Status
                        break;
                    default: // All values
                        pollSensor(measurementValues, 'A'); // Request All
                        break;
                }
            }

            // For compliance to cancel measurement if a line break is detected
            if (slaveSDI12.LineBreakReceived()) {
                EmptyOutputSDIBuffer(dValues);
                sensor.SetActive(false);
                measurement_count = 0;
                slaveSDI12.ClearLineMarkingReceived();
            } else {
                // Populate the "dValues" String array with the values in SDI-12 format
                formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_35);
                // For aM!, Send "service request" (<address><CR><LF>) when data is ready
                response = sensor.Address();
            }
            break;
        }
        case kConcurrentMeasurement: {
            // For aC!, aCx!, aCCx! commands
            sensor.SetCrcRequest(parsed_cmd.CRC());
            measurement_count = 0;
            if (parsed_cmd.param1 == 0) {
                measurement_count = 5;
            } else if (parsed_cmd.param1 > 0 && parsed_cmd.param1 <= MEASUREMENT_ARRAY_MAX_SIZE) {
                measurement_count = 1;
            }

            // Response should be in following format atttnn<CR><LF>
            if (measurement_count > 0) {
                response += "002";
                if (measurement_count < 10) { response += "0"; }
                response += measurement_count;
            } else {
                response += "00000";
            }
            response += "\r\n";
            slaveSDI12.sendResponse(response);

            if (parsed_cmd.secondary == kIdentification || measurement_count == 0) {
                sensor.SetActive(false);
                break;
            }

            if (measurement_count > 0) {
                switch (parsed_cmd.param1) {
                    case 1:
                        pollSensor(measurementValues, 'O'); // Oxygen (ppO2)
                        break;
                    case 2:
                        pollSensor(measurementValues, '%'); // Oxygen (%)
                        break;
                    case 3:
                        pollSensor(measurementValues, 'T'); // Temperature
                        break;
                    case 4:
                        pollSensor(measurementValues, 'P'); // Barometric Pressure
                        break;
                    case 5:
                        pollSensor(measurementValues, 'e'); // Sensor Status
                        break;
                    default: // All values
                        pollSensor(measurementValues, 'A'); // Request All
                        break;
                }
            }

            // For compliance to cancel measurement if a correct address is detected
            if (DetectedValidAddress(sensor.Address())) {
                EmptyOutputSDIBuffer(dValues);
                measurement_count = 0;
            } else {
                // Populate the "dValues" String array with the values in SDI-12 format
                formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_75);
            }
            sensor.SetActive(false);
            break;
        }
        case kVerification: {
            // For aV!
            // Do whatever the sensor is supposed to do here
            // For this example, we will just create arbitrary "simulated" sensor data
            // NOTE: Your application might have a different data type (e.g. int) and
            //       number of values to report!
            sensor.SetCrcRequest(parsed_cmd.CRC());
            measurement_count = 3;

            // Response should be in following format atttn<CR><LF>
            response += "0053\r\n";
            slaveSDI12.sendResponse(response);

            if (parsed_cmd.secondary == kIdentification) {
                sensor.SetActive(false);
                return;
            }

            pollOxygenSensorInfo(dValues);

            // For compliance to cancel measurement if a line break is detected
            if (slaveSDI12.LineBreakReceived()) {
                EmptyOutputSDIBuffer(dValues);
                sensor.SetActive(false);
                measurement_count = 0;
                slaveSDI12.ClearLineMarkingReceived();
            } else {
                // dValues already populated by pollOxygenSensorInfo(const char*)
                // Populate the "dValues" String array with the values in SDI-12 format
                // formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_35);
                // For aV!, Send "service request" (<address><CR><LF>) when data is ready
                response = sensor.Address();
            }
            break;
        }
        case kDataRequest: {
            // For aDx!
            if (parsed_cmd.param1 < MEASUREMENT_STR_ARRAY_MAX_ELEMENT) {
                response += dValues[parsed_cmd.param1];
            }
            // Add CRC if requested for appropriate commands
            if (sensor.CrcRequested()) {
                SDI12CRC crc(response.c_str());
                response += crc.ascii();
            }
            break;
        }
        case kContinuousMeasurement: {
            // For aRx!, aRCx! Commands
            sensor.SetCrcRequest(parsed_cmd.CRC());

            if (parsed_cmd.param1 > 0) { break; } // No other measurement types
            measurement_count = 5;

            // TODO: Continous measurement
            formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_75);

            response += dValues[0];

            if (sensor.CrcRequested()) {
                SDI12CRC crc(response.c_str());
                response += crc.ascii();
            }
            break;
        }
        case kHighVolumeASCII: {
            // For aHA!
            sensor.SetCrcRequest(parsed_cmd.CRC());
            measurement_count = 0;

            response += "000000";
            formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_75);
            break;
        }
        case kHighVolumeByte: {
            // For aHB! command
            // Currently not supported
            measurement_count = 0;

            // Response should be in following format atttnnn<CR><LF>
            response += "000000";
            break;
        }
        case kByteDataRequest: {
            // For aDBx!
            /* Not implemented, return respond with
                1 byte - ascii address
                2 byte - packet size (size in bytes of payload data)
                1 byte - data type
                ? byte - payload data
                2 byte - crc if requested
                */
            // TODO: DataByte request
            SDI12CRC crc;
            crc.Add(sensor.Address());
            slaveSDI12.MarkLine();
            slaveSDI12.writeBytes(sensor.Address());
            // Payload data size in number of bytes
            slaveSDI12.writeBytes((uint16_t)0);
            crc.Add((uint16_t)0);
            // DataType
            slaveSDI12.writeBytes((uint8_t)0);
            crc.Add((uint8_t)0);
            // Write CRC
            slaveSDI12.writeBytes(crc.value());
            sensor.SetActive(false);
            break;
        }
        case kIdentification: {
            // For aIX_001! - aIX_999!
            // Identify Meta Group, for measurement field information,
            // max length is 75 not including crc and <CR><LF>
            /* Response should be in the following format <addr>,<field1>,<field2>,<additional>;
            deliminated using ',' and ends with ';' followed by <CR><LF>
            field1 - is ideally using SHEF codes
            field2 - describes parameter units
            additional (optional) - additional fields are to deliminated using ',' */
            sensor.SetCrcRequest(parsed_cmd.CRC());

            if (parsed_cmd.secondary == kMeasurement ||
                parsed_cmd.secondary == kConcurrentMeasurement) {
                if ((parsed_cmd.param1 == 0 && parsed_cmd.param2 == 1) ||
                        (parsed_cmd.param2 == 1 && parsed_cmd.param1 == 1)) {
                    response += ",OXY,mbar,oxygen (O2) partial pressure;";
                } else if ((parsed_cmd.param1 == 0 && parsed_cmd.param2 == 2) ||
                        (parsed_cmd.param2 == 1 && parsed_cmd.param1 == 2)) {
                    response += ",%OX,%,oxygen (O2) percentage;";
                } else if ((parsed_cmd.param1 == 0 && parsed_cmd.param2 == 3) ||
                        (parsed_cmd.param2 == 1 && parsed_cmd.param1 == 3)) {
                    response += ",T,degrees C,sensor temperature;";
                } else if ((parsed_cmd.param1 == 0 && parsed_cmd.param2 == 4) ||
                        (parsed_cmd.param2 == 1 && parsed_cmd.param1 == 4)) {
                    response += ",P,mbar,barometric pressure;";
                } else if ((parsed_cmd.param1 == 0 && parsed_cmd.param2 == 5) ||
                        (parsed_cmd.param2 == 1 && parsed_cmd.param1 == 5)) {
                    response += ",e,number,sensor status;";
                }
            } else if (parsed_cmd.secondary == kVerification) {
                switch (parsed_cmd.param2) {
                    case 1:
                        response += ",# 0,YYYYY DDDDD,manufacture date;";
                        break;
                    case 2:
                        response += ",# 1,serial;";
                        break;
                    case 3:
                        response += ",# 2,software rev;";
                        break;
                }
            }

            if (sensor.CrcRequested()) {
                SDI12CRC crc(response.c_str());
                response += crc.ascii();
            }
            break;
        }
        default: {
            // Debug purposes only
            response += "--";
            response += parsed_cmd.primary;
            response += "-";
            response += parsed_cmd.secondary;
            response += "-";
            response += parsed_cmd.param1;
            response += "-";
            response += parsed_cmd.param2;
            break;
        }
    }
    if (sensor.IsActive()) {
        response += "\r\n";
        slaveSDI12.sendResponse(response);
        sensor.SetActive(false);
    }

    SDI12Sensor::SetState(STATE_LOW_POWER);
    slaveSDI12.forceListen();  // sets SDI-12 pin as input to prepare for
                                // incoming message AGAIN
}


void PassThroughMode(void) {
    String commandReceived = "";
    String response = "";
    while (true) {
        if (slaveSDI12.available() < 0) { slaveSDI12.clearBuffer(); }
        while (slaveSDI12.available() > 0) {
            char c = slaveSDI12.read();
            if (c == '\x03' || c == '\n' || c == '\r') {
                if (commandReceived.equals("exit!")) {
                    break;
                }

                // Empty rx buffer
                while (Serial.available()) {
                    Serial.read();
                }

                // Send polling command
                Serial.println(commandReceived);
                Serial.flush();

                // delay(2000);
                // Blocking delay until a character exist in serial buffer
                while (!Serial.available());

                while(Serial.available() > 0) {
                    char c = Serial.read();
                    if ((c != '\n') && (c != '\r')) {
                        response += c;
                    }
                    delay(2);  // 1 character ~ 1.04 ms @ 9600 baud
                }
                if (response.length() > 0) {
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);  // write the response to the screen
                }

                slaveSDI12.forceListen(); // Force listen if command is not recognized
                response = "";
                commandReceived = "";
                slaveSDI12.ClearLineMarkingReceived(); // Clear detected break marking
                slaveSDI12.clearBuffer();
            } else if (c >= 32 && c <= 126) {
                commandReceived += String(c);
            }
            delay(10);  // 1 character ~ 8.33 ms @ 1200 baud
        }

        if (commandReceived.equals("exit!")) {
            response = sensor.Address();
            response += "\r\n";
            slaveSDI12.sendResponse(response);
            commandReceived = "";
            slaveSDI12.clearBuffer();
            slaveSDI12.ClearLineMarkingReceived();
            SDI12Sensor::SetState(STATE_LOW_POWER);
            return;  // Exit pass through mode
        }
    }
}