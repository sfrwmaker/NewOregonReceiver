#ifndef _OREGON_V2_H
#define _OREGON_V2_H

#include <Arduino.h>

#define DATA_BUFF_SIZE  (4)

typedef enum {
    PULSE_UNKNOWN, PULSE_START_MSG = 1, PULSE_END_MSG, PULSE_NO_MESSAGE, PULSE_SHORT_PREAMBLE,
    PULSE_ODD_TIME, PULSE_LONG, PULSE_SHORT, PULSE_TOO_SHORT, PULSE_LONG_OUT, PULSE_SHORT_OUT, PULSE_WRONG 
} PulseStatus;

class DecodeOOK {
    public:
        DecodeOOK(void)									{ reset();              }
        bool            init(uint8_t pin_number);
		void			setPulseParameters(uint16_t min_pulse, uint16_t short_pulse, uint16_t long_pulse, uint16_t end_pulse);
        void            checkPreamble(bool check = true);
        bool            isMessageReceived(void)         { return got_message;   }
        void            dump(void);
        inline void     reset(void);
    protected:
        static void		itr(void);                      // The interrupt handler
        PulseStatus     pulseDecode(uint16_t width, bool rf_status);
        uint16_t        dataBits(void);                 // Flush cached data to the buffer, return number of bits received
        bool            isPreambleValid(void);
		uint16_t		min_pulse	= 200;
		uint16_t		short_pulse	= 700;
		uint16_t		long_pulse	= 1200;
		uint16_t		end_pulse	= 3500;
        bool            check_preamble    = true;       // Perform testing minimal length of the preamble when packet received
        uint32_t        data[DATA_BUFF_SIZE]    = {0};
        volatile uint16_t   half_time   = 0;            // Time conter from message start (short intervls)
        volatile uint32_t   c_bit       = 0x80000000;   // The manchester bit to be written into data array
        volatile uint32_t   c_data      = 0;            // The manchester word in progress
        volatile uint8_t    pos         = 0;            // Current index in the data arrray
        volatile bool       got_message = false;        // Some message received
};

class OregonDecoderV2 : public DecodeOOK {
    public:   
        OregonDecoderV2()                               { }
		bool            receiveData(uint8_t& ch, uint8_t& id, int16_t& temp, uint8_t& hum, bool& battOK);
        void            dump(void);
        void            simulate(uint16_t data[], uint8_t size, bool first_pulse);
    private:
        uint8_t         mirror(uint8_t source);
        int16_t         msgStart(uint16_t id, uint16_t bits);
        uint8_t         getByte(uint16_t first_bit);
        bool            decodeTempHumidity(const uint8_t buff[], int16_t& temp, uint8_t& hum, bool& battOK);
        bool            isSummOK(const uint8_t buff[], uint16_t snesor_ID);
        uint8_t         sum(const uint8_t buff[], uint8_t count);
        const uint16_t  validID[2] = {0x1A2D, 0xEA4C};  // THGR2228N, THGN132N
        const uint8_t   num_sensors = sizeof(validID)/sizeof(uint16_t);
};

#endif
