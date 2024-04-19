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

#include <SDI12Slave.h>
#include <SDI12CRC.h>
#include <SDI12Sensor.h>


#define DATA_PIN 2  /*!< The pin of the SDI-12 data bus */
#define SDI12_ADDRESS_EEPROM_ADDRESS 0
#define POWER_PIN -1 /*!< The sensor power pin (or -1 if not switching power) */

#define MEASUREMENT_ARRAY_MAX_SIZE 5 // Max size of floats/double array to hold sensor data
#define MEASUREMENT_STR_ARRAY_MAX_ELEMENT 10 // Max number of array elements for 0Dx! string response

#define SDI12SENSOR_SDI12_PROTOCOL "13"  // Respresent v1.3
#define SDI12SENSOR_COMPANY "UQGEC"  // 8 Charactors depicting company name
#define SDI12SENSOR_MODEL "000000"  // 6 Characters specifying sensor model
#define SDI12SENSOR_VERSION "0.1"  // 3 characters specifying sensor version
#define SDI12SENSOR_OTHER_INFO "LUMINOX"  // (optional) up to 13 char for serial or other sensor info

// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12Slave slaveSDI12(DATA_PIN);
SDI12Sensor sensor('0', SDI12_ADDRESS_EEPROM_ADDRESS);

static uint8_t measurement_count = 0;

void pollSensor(float* measurementValues, char command) {
    // measurementValues[0] = 1.1;
    // measurementValues[1] = -2.22;
    // measurementValues[2] = 3.333;
    // measurementValues[3] = -4.4444;
    // measurementValues[4] = 5.55555;
    // measurementValues[5] = -6.666666;
    // measurementValues[6] = 78.777777;
    // measurementValues[7] = -890.888888;
    // measurementValues[8] = -0.11111111;

    // Empty rx buffer
    while (Serial.available()) {
        Serial.read();
    }

    // Poll Oxygen
    Serial.print(String(command) + "\r\n"); // Send command to initiate polling mode
    Serial.flush();
    delay(1000);

    String luminoReply = "";
    uint8_t inChar;
    while (Serial.available()) {
        inChar = Serial.read();
        if ((inChar >= 48 && inChar <= 57) || // 0-9
                (inChar >= 43 && inChar <= 46) // [+,-.]
                ) {
            luminoReply += (char)inChar;
        } else if (inChar == 'T' || inChar == 'P' || inChar == '%' || inChar == 'e') {
            luminoReply += ',';
        } else if (inChar == '\n') {
            break;
        }
    }

    int8_t charPos = 0;
    uint8_t i = 0;
    do {
        measurementValues[i++] = luminoReply.substring(charPos).toFloat();
        // measurementValues[i++] = charPos;
        charPos = luminoReply.indexOf(",", charPos) + 1;
    } while (charPos > 0);

    // Zero remaining
    for (i; i < MEASUREMENT_ARRAY_MAX_SIZE; i++) {
        measurementValues[i] = 0;
    }
}

void pollOxygenSensorInfo(String* dValues) {
    // Empty rx buffer
    while (Serial.available()) {
        Serial.read();
    }

    // Poll Oxygen
    Serial.print("# 0\r\n"); // Send command to initiate polling mode
    Serial.flush();
    dValues[0] = Serial.readStringUntil('\r');
    Serial.print("# 1\r\n"); // Send command to initiate polling mode
    Serial.flush();
    dValues[1] = Serial.readStringUntil('\r');
    Serial.print("# 2\r\n"); // Send command to initiate polling mode
    Serial.flush();
    dValues[2] = Serial.readStringUntil('\r');
}

void parseSdi12Cmd(String command, SDI12CommandSet_s *parsed_cmd) {
    /* Ingests a command from an SDI-12 master, sends the applicable response, and
     * (when applicable) sets a flag to initiate a measurement
     */

    // First char of command is always either (a) the address of the device being
    // probed OR (b) a '?' for address query.
    // Do nothing if this command is addressed to a different device
    *parsed_cmd = SDI12Sensor::ParseCommand(command.c_str());
    if (parsed_cmd->address == sensor.Address()) {
        sensor.SetActive();
    } else if (parsed_cmd->primary == kAddressQuery) {
        // Let loop handle "?!" command
        sensor.SetActive();
        sensor.SetState(kStateReady);
        return;
    } else {
        return;
    }

    SDI12Sensor::LastActive()->DefineState(*parsed_cmd);

    // If execution reaches this point, the slave should respond with something in
    // the form:   <address><responseStr><Carriage Return><Line Feed>
    // The following if-switch-case block determines what to put into <responseStr>,
    // and the full response will be constructed afterward. For '?!' (address query)
    // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
    String responseStr = "";
    responseStr = SDI12Sensor::LastActive()->Address();
    // Only perform some basic sensor operations here, i.e aI! and a!
    switch ((SDI12SensorCommand_e)parsed_cmd->primary) {
        case kAcknowledge:
            break; // Do not need to do anything
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
            }
            break;
        case kAddressChange:
            // Change address command
            // Slave should respond with blank message (just the [new] address +
            // <CR> + <LF>)
            SDI12Sensor::LastActive()->SetAddress(parsed_cmd->param1);
            responseStr = SDI12Sensor::LastActive()->Address();
            break;
        case kUnknown:
            // For DEBUG
            responseStr += "UNK\r\n";
            break;
        default:
            return; // Handle everything else in loop
    }

    if (SDI12Sensor::IsSetLastActive()) {
        responseStr += "\r\n";
        slaveSDI12.sendResponse(responseStr);
        SDI12Sensor::ClearLastActive();
    }
}

void formatOutputSDI(float* measurementValues, String* dValues, uint8_t data_count, unsigned int maxChar) {
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

    // Fill rest of dValues with blank strings
    while (j < MEASUREMENT_ARRAY_MAX_SIZE) { dValues[++j] = ""; }
}

void setup() {
    slaveSDI12.begin();
    Serial.begin(9600);
    // delay(500);
    slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message
    Serial.print("M 1\r\n"); // Send command to initiate polling mode
}

void loop() {
    static float measurementValues[MEASUREMENT_ARRAY_MAX_SIZE];  // floats to hold sensor data
    static String dValues[MEASUREMENT_STR_ARRAY_MAX_ELEMENT];  // String objects to hold the responses to aD0!-aD9! commands
    static String commandReceived = "";  // String object to hold the incoming command
    SDI12CommandSet_s parsed_cmd;
    String response = "";


    // If a byte is available, an SDI message is queued up. Read in the entire message
    // before proceding.  It may be more robust to add a single character per loop()
    // iteration to a static char buffer; however, the SDI-12 spec requires a precise
    // response time, and this method is invariant to the remaining loop() contents.
    int avail = slaveSDI12.available();
    if (avail < 0) {
        slaveSDI12.clearBuffer();
    } else if (avail > 0) {
        // Buffer is full; clear
        for (int a = 0; a < avail; a++) {
            char charReceived = slaveSDI12.read();
            // Character '!' indicates the end of an SDI-12 command; if the current
            // character is '!', stop listening and respond to the command
            if (charReceived == '!') {
                // eliminate the chance of getting anything else after the '!'
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
            } else {
                // If the current character is anything but '!', it is part of the command
                // string.  Append the commandReceived String object.
                // Append command string with new character
                commandReceived += String(charReceived);
            }
        }
    }

    if (sensor.IsActive() || parsed_cmd.primary == kAddressQuery) {
        response = sensor.Address();
        switch ((SDI12SensorState_e)sensor.State()) {
            case kStateLowPower:
                break;

            case kStateReady:
                if (parsed_cmd.primary == kAddressQuery) {
                    // Do nothing for a!, response is already appropriate
                } else if (parsed_cmd.primary == kDataRequest) {
                    // For aDx!
                    if (parsed_cmd.param1 < MEASUREMENT_STR_ARRAY_MAX_ELEMENT) {
                        response += dValues[parsed_cmd.param1];
                    }
                    // Add CRC if requested for appropriate commands
                    if (sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str());
                        response += crc.GetAscii();
                    }
                } else if (parsed_cmd.primary == kByteDataRequest) {
                    // For aDBx!
                    /* Not implemented, return respond with
                     1 byte - ascii address
                     2 byte - packet size
                     1 byte - data type
                     2 byte - crc if requested
                     */
                    slaveSDI12.sendResponse(""); // Empty send response just to send line marking
                    slaveSDI12.writeBytes(sensor.Address());
                    slaveSDI12.writeBytes((uint16_t)0); // Packet size
                    slaveSDI12.writeBytes((uint8_t)kInvalidDataType); // Data type
                    // No binary data payload to transmit
                    if (sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str()); // CRC of address
                        crc.Add((uint16_t)0); // CRC of packet size
                        crc.Add((uint8_t)kInvalidDataType); // CRC of data type
                        // No binary data payload to calcualte CRC
                        slaveSDI12.writeBytes(crc.Get()); // Write CRC to data line
                    }
                    sensor.SetActive(false); // Stop further ascii transmission
                } else if (parsed_cmd.primary == kIdentification) {
                    // For aIX_001! - aIX_999!
                    // Identify Meta Group, for measurement field information,
                    // max length is 75 not including crc and <CR><LF>
                    /*
                     Response should be in the following format ,<field1>,<field2>,<additional>;
                     deliminated using ',' and ends with ';' followed by <CR><LF>
                     field1 - is ideally using SHEF codes
                     field2 - describes parameter units
                     additional (optional) - additional fields are to deliminated using ','
                    */
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
                            response += ",E,number,error code;";
                        }
                    } else if (parsed_cmd.secondary == kContinuousMeasurement) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kHighVolumeASCII) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kHighVolumeByte) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kVerification) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
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

                    // Add CRC if requested for appropriate commands
                    if (parsed_cmd.param2 > 0 && sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str());
                        response += crc.GetAscii();
                    }
                }

                break;

            case kStateMeasurement:
                // For aM! and aMx! commands
                // Do whatever the sensor is supposed to do here
                // For this example, we will just create arbitrary "simulated" sensor data
                // NOTE: Your application might have a different data type (e.g. int) and
                //       number of values to report!
                // Response should be in following format atttn<CR><LF>
                if (parsed_cmd.param1 >= 0 && parsed_cmd.param1 <= MEASUREMENT_ARRAY_MAX_SIZE) {
                    response += "002";
                    if (parsed_cmd.param1 > 0) {
                        measurement_count = 1;
                    } else {
                        measurement_count = 5;
                    }
                    response += String(measurement_count);
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentification) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    // delay(2000); // 2 Second delay to simulate sensor measurement
                    switch (parsed_cmd.param1) {
                        case 1: // Request Oxygen 'O'
                            pollSensor(measurementValues, 'O');
                            break;
                        case 2:
                            pollSensor(measurementValues, '%');
                            break;
                        case 3:
                            pollSensor(measurementValues, 'T');
                            break;
                        case 4:
                            pollSensor(measurementValues, 'P');
                            break;
                        case 5:
                            pollSensor(measurementValues, 'e');
                            break;
                        default: // All values
                            pollSensor(measurementValues, 'A');
                            break;
                    }
                } else {
                    response += "0000";
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentification) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    sensor.SetActive(false);
                }

                // For compliance to cancel measurement if a line break is detected
                if (slaveSDI12.LineBreakReceived() || !sensor.IsActive()) {
                    for (size_t i = 0; i < (sizeof(dValues)/sizeof(*dValues)); i++) {
                        dValues[i] = "";
                    }
                    sensor.SetActive(false);
                } else {
                    // Populate the "dValues" String array with the values in SDI-12 format
                    formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_35);
                    // For aM!, Send "service request" (<address><CR><LF>) when data is ready
                    response = sensor.Address();
                }
                slaveSDI12.ClearLineMarkingReceived();
                break;

            case kStateConcurrent:
                // For aC! and aCx! commands
                // Do whatever the sensor is supposed to do here
                // For this example, we will just create arbitrary "simulated" sensor data
                // NOTE: Your application might have a different data type (e.g. int) and
                //       number of values to report!
                // Response should be in following format atttnn<CR><LF>
                if (parsed_cmd.param1 >= 0 && parsed_cmd.param1 <= MEASUREMENT_ARRAY_MAX_SIZE) {
                    response += "0020";
                    if (parsed_cmd.param1 > 0) {
                        measurement_count = 1;
                    } else {
                        measurement_count = 5;
                    }
                    response += String(measurement_count);
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentification) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    // delay(2000); // 2 Second delay to simulate sensor measurement
                    switch (parsed_cmd.param1) {
                        case 1: // Request Oxygen 'O'
                            pollSensor(measurementValues, 'O');
                            break;
                        case 2:
                            pollSensor(measurementValues, '%');
                            break;
                        case 3:
                            pollSensor(measurementValues, 'T');
                            break;
                        case 4:
                            pollSensor(measurementValues, 'P');
                            break;
                        case 5:
                            pollSensor(measurementValues, 'e');
                            break;
                        default: // All values
                            pollSensor(measurementValues, 'A');
                            break;
                    }
                } else {
                    response += "00000";
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentification) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    sensor.SetActive(false);
                }

                // For compliance to cancel measurement if a correct address is detected
                for (int a = 0; a < slaveSDI12.available(); a++) {
                    char charReceived = slaveSDI12.read();
                    if (charReceived == '!') {
                        slaveSDI12.clearBuffer();
                        break;
                    } else {
                        commandReceived += charReceived;
                    }
                }
                if (!sensor.IsActive() ||
                    SDI12Sensor::ParseCommand(commandReceived.c_str()).address == sensor.Address()) {
                    for (size_t i = 0; i < (sizeof(dValues)/sizeof(*dValues)); i++) {
                        dValues[i] = "";
                    }
                } else {
                    // Populate the "dValues" String array with the values in SDI-12 format
                    formatOutputSDI(measurementValues, dValues, measurement_count, SDI12_VALUES_STR_SIZE_75);
                }
                sensor.SetActive(false);
                break;

            case kStateHighMeasurement:
                // For aHA! and aHB! commands
                // Do whatever the sensor is supposed to do here
                // For this example, we will just create arbitrary "simulated" sensor data
                // NOTE: Your application might have a different data type (e.g. int) and
                //       number of values to report!
                // Response should be in following format atttnnn<CR><LF>
                response += "000000";
                // No need to perform measurement if identification requested
                for (size_t i = 0; i < (sizeof(dValues)/sizeof(*dValues)); i++) {
                    dValues[i] = "";
                }
                break;

            case kStateContinuous:
                // For aRx! commands
                // Data should be available and broadcasted immediately similar to aDx! commands
                // Message <values> length is limited to 75 characters long
                // if (parsed_cmd.param1 < MEASUREMENT_ARRAY_MAX_SIZE) {
                //     pollSensor(measurementValues, 'A');
                //     char temp_string_value[SDI12_VALUE_STR_SIZE+1];
                //     if (dtoa(measurementValues[parsed_cmd.param1], temp_string_value,6, SDI12_VALUE_STR_SIZE)) {
                //         response += temp_string_value;
                //     }
                // } else {
                //     // Do nothing
                // }

                // Add CRC if requested for appropriate commands
                if (sensor.CrcRequested()) {
                    SDI12CRC crc(response.c_str());
                    response += crc.GetAscii();
                }
                break;

            case kStateVerify:
                // For aV!
                // Response should be in following format atttn<CR><LF>
                // Not implemented, return respond with "00000"
                response += "0053";
                // No need to perform measurement if identification requested
                if (parsed_cmd.primary == kIdentification) { break; }
                response += "\r\n";
                slaveSDI12.sendResponse(response);
                // For aV!, Send "service request" (<address><CR><LF>) when data is ready
                response = sensor.Address();
                
                pollOxygenSensorInfo(dValues);
                // aV! does not have a way to cancel
                // Populate the "dValues" String array with the values in
                // SDI-12 format up to 35 characters long max
                for (size_t i = 3; i < (sizeof(dValues)/sizeof(*dValues)); i++) {
                    dValues[i] = "";
                }
                // sensor.SetActive(false);
                break;
        }

        if (sensor.IsActive() || parsed_cmd.primary == kAddressQuery) {
            response += "\r\n";
            slaveSDI12.sendResponse(response);
        }
        sensor.SetState(kStateReady);
        sensor.SetActive(false);
        slaveSDI12.forceListen();   // sets SDI-12 pin as input to prepare for
                                    // incoming message AGAIN
    }
}
