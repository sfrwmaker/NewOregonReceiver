#include "oregon_v2.h"

volatile static uint8_t     rx_bit      = 0;        // Bit mask of the Receive Port, initialized by pin_number
volatile static uint8_t     *rx_port    = 0;        // Pointer  to the Receive Port, initialized by pin_number
volatile static DecodeOOK	*pDecoder	= 0;		// The Decore instance

bool DecodeOOK::init(uint8_t pin_number) {
    int interrupt = digitalPinToInterrupt(pin_number);
    if (interrupt >= 0) {
        uint8_t port = digitalPinToPort(pin_number);
        if (port != NOT_A_PIN) {
            rx_port = portInputRegister(port);
            rx_bit  = digitalPinToBitMask(pin_number);
            attachInterrupt(interrupt, DecodeOOK::itr, CHANGE);
            pDecoder    = this;
            return true;
        }
    }
    return false;
}

void DecodeOOK::setPulseParameters(uint16_t min_pulse, uint16_t short_pulse, uint16_t long_pulse, uint16_t end_pulse) {
    if (50 <= min_pulse && min_pulse <= 0xefff)
        min_pulse = min_pulse;
    if (short_pulse > min_pulse)
	    this->short_pulse	= short_pulse;
    if (long_pulse > short_pulse)
	    this->long_pulse	= long_pulse;
    if (end_pulse > long_pulse)
	    this->end_pulse		= end_pulse;
}

void DecodeOOK::checkPreamble(bool check) {
    check_preamble = check;
}

/*
 * The receiver Interrupt Handler
 * Check the pulse interval in mks,
 * If the pulse interval is 
 */
void DecodeOOK::itr(void) {
    volatile static uint32_t last = 0;
    noInterrupts();
    // determine the pulse length in microseconds, for either polarity
    volatile uint32_t n = micros();
    if (n > last) {
        volatile uint32_t wl_pulse = n - last;
        last = n;
        bool rf_status = (*rx_port & rx_bit);           // Current RF port status
	    pDecoder->pulseDecode(wl_pulse, rf_status);     // Previous RF status defines the bit
    }
    interrupts();
}

// The OOK pulse decoder
PulseStatus DecodeOOK::pulseDecode(uint16_t width, bool rf_status) {
    PulseStatus res = PULSE_UNKNOWN;
    if (got_message) return res;                        // Wait message to be decoded, no new data
    if (half_time == 0) {                               // No message started yet
        // The preamble is a series of long pulses
        if (short_pulse < width && width <= long_pulse) {
            res = PULSE_NO_MESSAGE;
            if (!rf_status) {                           // Start new message when long pulse is LOW
                half_time = 2;                          // half_time counter initialized, started to receive a message
                res = PULSE_START_MSG;
            }
        }
    } else {                                            // The message has been started
        if (width > short_pulse) {
            if (width <= long_pulse) {                  // Long pulse
                if (half_time & 1) {                    // ERROR: ODD bit
                    res = PULSE_ODD_TIME;
                    reset();
                } else {                                // EVEN bit
                    res = PULSE_LONG;
                    half_time += 2;
                    if ((half_time & 3) == 0) {         // Write out only every 4-th bit
                        if (rf_status) c_data |= c_bit; // Write "1" if HIGH and "0" if LOW
                        c_bit >>= 1;
                        res = PULSE_LONG_OUT;
                    }
                }
            } else if (width > end_pulse && half_time >= 200) {
                if (c_bit) data[pos++] = c_data;        // Flush the cache to the output buffer
                got_message = true;                     // Got new message in the buffer
                res = PULSE_END_MSG;
                if (check_preamble && !isPreambleValid()) {
                    res = PULSE_SHORT_PREAMBLE;
                    reset();
                }
            } else {                                    // Invalid pulse, greater than long, but not end of message
                res = PULSE_WRONG;
                reset();
            }
        } else {
            if (width > min_pulse) {                    // Short pulse
                res = PULSE_SHORT;
                ++half_time;
                if ((half_time & 3) == 0) {             // Write out every 4-th bit only
                    if (rf_status) c_data |= c_bit;     // Write "1" if HIGH and "0" if LOW
                    c_bit >>= 1;
                    res = PULSE_SHORT_OUT;
                }
            } else {                                    // Invalid message, too short
                res = PULSE_TOO_SHORT;
                reset();
            }
        }
        if (!c_bit) {                                   // Whole word has been filled, flush to the buffer
            data[pos]   = c_data;
            c_data      = 0;
            c_bit       = 0x80000000;
           if (++pos >= DATA_BUFF_SIZE) pos = DATA_BUFF_SIZE-1;
        }
    }
    return res;
}
/*
// The OOK pulse decoder
PulseStatus DecodeOOK::pulseDecode(uint16_t width, bool rf_status) {
    PulseStatus res = PULSE_UNKNOWN;
	if (!got_message) {									// Not received a message yet
		if (width > long_pulse) {
			if (width > end_pulse && half_time >= 200) {
                if (c_bit) data[pos++] = c_data;        // flush the cache to the output buffer
                got_message = true;                     // Got long message in the buffer
                res = PULSE_END_MSG;
                if (check_preamble && !isPreambleValid()) {
                    res = PULSE_SHORT_PREAMBLE;
                    reset();
                }
			} else if (half_time > 0) {                 // ERROR: reset the decoder
                res = PULSE_WRONG;
				reset();
			}
		} else if (width >= short_pulse) {              // Long pulse
			if (half_time > 0) {
				if (half_time & 1) {                    // ERROR: ODD bit
                    res = PULSE_ODD_TIME;
					reset();
				} else {							    // EVEN bit
                    res = PULSE_LONG;
                    if (half_time > 0) {
                        half_time += 2;
                        if ((half_time & 3) == 0) {     // Write out only every 4-th bit
					        if (rf_status) c_data |= c_bit; // Write "1" if HIGH and "0" if LOW
					        c_bit >>= 1;
                            res = PULSE_LONG_OUT;
                        }
                    }
				}
			} else {
                res = PULSE_NO_MESSAGE;
				if (!rf_status) {                       // Start new message when long pulse is LOW
                    half_time = 2;                      // half_time counter initialized, started to receive a message
                    res = PULSE_START_MSG;		    
				}
            }
		} else if (width >= min_pulse) {			    // Short pulse
            res = PULSE_SHORT;						
			if (half_time > 0) {
			    ++half_time;
			    if ((half_time & 3) == 0) {             // Write out only every 4-th bit
				    if (rf_status) c_data |= c_bit;	    // Write "1" if HIGH and "0" if LOW
				    c_bit >>= 1;
                    res = PULSE_SHORT_OUT;
			    }
			}
		} else {                                        // Too short pulse
            res = PULSE_TOO_SHORT;
            reset();
        }
		if (!c_bit) {                   				// Whole word has beed filled, flush to the buffer
			data[pos]   = c_data;
			c_data      = 0;
			c_bit       = 0x80000000;
           if (++pos > 9) pos = 9;
		}
	}
    return res;
}
*/

void DecodeOOK::reset(void) {
    half_time   = 0;
    pos         = 0;
    got_message = false;
    c_bit       = 0x80000000;
    c_data      = 0;
}

uint16_t DecodeOOK::dataBits(void) {
    uint16_t bits = 0;
    uint32_t m_bit = c_bit;
    while (m_bit <<= 1) ++bits;
    bits += pos * 32;
    if (c_bit) {
        bits -= 8;                                      // The last word has been already flushed
    }
    return bits;
}

bool DecodeOOK::isPreambleValid(void) {
    uint32_t p_bit = 0x80000000;                        // 31-th bit
    for (uint8_t b = 0; b < 8; ++b) {                   // Scan first 8 bits
        if ((data[0] & p_bit) == 0) {                   // The preamble end
             return false;
        }
        p_bit >>= 1;
    }
    return true;
}

void DecodeOOK::dump(void) {
    uint8_t bits = dataBits();
    if (bits > 0) {
        Serial.print("half_time = ");   Serial.print(half_time);
        Serial.print(", received ");    Serial.print(bits); Serial.print(" bits");
        Serial.print(", allocated ");   Serial.print(pos);  Serial.println(" 32-bit words");
        for (uint8_t i = 0; i < pos; ++i) {
            Serial.print(data[i], HEX);
            Serial.print(",");
        }
        Serial.println("");
    }
}

//--------------------------------------- OregonDecoderV2 ------------------------
bool OregonDecoderV2::receiveData(uint8_t& ch, uint8_t& id, int16_t& temp, uint8_t& hum, bool& battOK) {
    if (!got_message) return false;
    uint16_t bits = dataBits();
    for (uint8_t s = 0; s < num_sensors; ++s) {
        int16_t  packet = msgStart(validID[s], bits);
        if (packet > 0) {
            if (packet + 9*8 < bits) {
                uint8_t buff[9];
                for (uint8_t i = 0; i < 9; ++i) {
                    buff[i] = mirror(getByte(packet+i*8));
                }
                if (decodeTempHumidity(buff, temp, hum, battOK))  {
                    ch  = buff[2];
                    id  = buff[3];
                    DecodeOOK::reset();
                    return true;
                }
            }
            
            break;
        }
    }
    DecodeOOK::reset();
    return false;
}

uint8_t OregonDecoderV2::mirror(uint8_t source) {
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t bt = source & 1;
        source  >>= 1;
        result  <<= 1;
        result  |= bt;
    }
    return result;
}

int16_t OregonDecoderV2::msgStart(uint16_t id, uint16_t bits) {
    uint8_t s1  = mirror(id>>8);                        // Oregon data sent LSB order, firts byte of sensor ID    
    uint8_t s2  = mirror(id & 0xff);                    // second byte of sensor ID 
    for (uint16_t i = 0; i < bits-8*8; ++i) {           // Minimum message length is 8 bytes
        uint8_t  b = getByte(i);                        // read byte starting from i-th bit
        if (s1 == b) {                                  // First byte matches
            b = getByte(i+8);
            if (s2 == b) {                              // Second byte matches also
                return i;
            }
        }
    }
    return -1;
}

// return byte starting from firth_bit bit
uint8_t OregonDecoderV2::getByte(uint16_t first_bit) {
    uint16_t w      = first_bit >> 5;                   // starting 32-bits word, first_bit / 32
    uint8_t  left   = first_bit & 0x1f;                 // first bit inside the 32-bits word 
    if (left <= 24) {                                   // Fits one 32-bits word
        uint8_t rshft = 24-left;                        // Right shift
        return data[w] >> rshft;
    }
    uint8_t  lshft  = left-24;                          // First 32-bits word should be shifted left
    uint32_t res    = (data[w] << lshft);
    uint32_t nxt    = 0;                                // Next word from the data buffer
    if (w+1 < pos) nxt = data[w+1];
    nxt >>= 32-lshft;
    res |= nxt;
    return res & 0xff;
}

bool OregonDecoderV2::decodeTempHumidity(const uint8_t buff[], int16_t& temp, uint8_t& hum, bool& battOK) {
    uint16_t sensor_ID  = buff[0]<<8 | buff[1];
    bool     is_summ_ok = isSummOK(buff, sensor_ID);
    if (is_summ_ok) {
        int16_t t = buff[5] >> 4;                       // 1st decimal digit
        t *= 10;
        t += buff[5] & 0x0F;                            // 2nd decimal digit
        t *= 10;
        t += buff[4] >> 4;                              // 3rd decimal digit
        if (buff[6] & 0x08) t *= -1;
        temp = t;
        hum = 0;
        battOK = !(buff[4] & 0x0C);
        if (sensor_ID == 0x1A2D) {                      // THGR2228N
            hum  = buff[7] & 0xF;
            hum *= 10;
            hum += buff[6] >> 4;
            hum = constrain(hum, 0, 100);
        }
    }
    return is_summ_ok;
}

bool OregonDecoderV2::isSummOK(const uint8_t buff[], uint16_t sensor_ID) {
    uint8_t s1 = 0;
    uint8_t s2 = 0;

    switch (sensor_ID) {
        case 0x1A2D:                                // THGR2228N
            s1 = (sum(buff, 8) - 0xa) & 0xFF;
            return (buff[8] == s1);
        case 0xEA4C:                                // TNHN132N
            s1 = (sum(buff, 6) + (buff[6]&0xF) - 0xa) & 0xff;
            s2 = (s1 & 0xF0) >> 4;
            s1 = (s1 & 0x0F) << 4;
            return ((s1 == buff[6]) && (s2 == buff[7]));
        default:
            break;
    }
    return false;
}

uint8_t OregonDecoderV2::sum(const uint8_t buff[], uint8_t count) {
    uint16_t s = 0;
    for (uint8_t i = 0; i < count; ++i) {
        s += (buff[i]&0xF0) >> 4;
        s += (buff[i]&0xF);
    }
    if (int16_t(count) != count)
        s += (buff[count]&0xF0) >> 4;
    return uint8_t(s);
}

void OregonDecoderV2::dump(void) {
    if (!got_message) return;
    Serial.println("\nReceived new packet");
    DecodeOOK::dump();
    uint16_t bits = dataBits();
    Serial.print("Data bits = "); Serial.println(bits);
    for (uint8_t s = 0; s < num_sensors; ++s) {
        int16_t  packet = msgStart(validID[s], bits);
        if (packet > 0) {
            uint16_t code  = mirror(validID[s]>>8) << 8;    
            code |= mirror(validID[s] & 0xff);
            Serial.print("Found "); Serial.print(validID[s], HEX); Serial.print(" ("); Serial.print(code, HEX); Serial.print(") header at ");
            Serial.print(packet); Serial.println(" bit");
            if (packet + 9*8 < bits) {
                uint8_t buff[9];
                Serial.println("Trying to decode following message: ");
                for (uint8_t i = 0; i < 9; ++i) {
                    buff[i] = mirror(getByte(packet+i*8));
                    Serial.print(buff[i], HEX); Serial.print(",");
                }
                
                if (isSummOK(buff, validID[s])) {
                    Serial.println("; CRC is OK");
                } else {
                    Serial.println("; CRC is NOT OK");
                }
                int16_t temp    = 0;
                uint8_t hum     = 0;
                bool    battOK  = true;
                if (decodeTempHumidity(buff, temp, hum, battOK))  {
                    Serial.print("Decoded data from "); Serial.print(buff[3], HEX);
                    Serial.print(" sensor. Temp = "); Serial.print(temp);
                    Serial.print(", humidity = "); Serial.println(hum);
                    return;
                }
            }
            
            break;
        }
    }
    return;
}

void OregonDecoderV2::simulate(uint16_t data[], uint8_t size, bool first_pulse) {
    for (uint8_t i = 0; i < size; ++i) {
        char num[8];
        sprintf(num, "%4d: ", i);
        Serial.print(num);
        if (first_pulse) Serial.print("HI "); else Serial.print("LO ");
        Serial.print(data[i]); Serial.print(": ");
        switch(pulseDecode(data[i], first_pulse)) {
            case PULSE_START_MSG:
                Serial.print("New message");
                break;
            case PULSE_END_MSG:
                Serial.print("End of message");
                break;
            case PULSE_SHORT_PREAMBLE:
                Serial.print("Short preamble");
                break;
            case PULSE_ODD_TIME:
                Serial.print("Long pulse in ODD period");
                break;
            case PULSE_LONG:
                Serial.print("Long pulse");
                break;
            case PULSE_SHORT:
                Serial.print("Short pulse");
                break;
            case PULSE_TOO_SHORT:
                Serial.print("Too short pulse");
                break;
            case PULSE_LONG_OUT:
                Serial.print("Long pulse, write bit to output");
                break;
            case PULSE_SHORT_OUT:
                Serial.print("Short pulse, write bit to output");
                break;
            case PULSE_WRONG:
                Serial.print("Pulse is longer than max");
                break;
            case PULSE_NO_MESSAGE:
                Serial.print("No message yet, ignore");
                break;
            default:
                Serial.print("Unknown");
                break;
                
        }
        Serial.print("; half_time = "); Serial.print(half_time);
        Serial.print(", out_data = "); Serial.print(c_data, BIN);
        Serial.print(", out_bit = ");  Serial.println(c_bit, HEX);
        first_pulse = !first_pulse;
    }
    Serial.println("\n END");
    if (c_bit) data[pos++] = c_data;                        // Flush output buffer
    got_message = true;
    dump();
}
