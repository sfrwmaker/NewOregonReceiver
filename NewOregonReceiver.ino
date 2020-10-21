#include "oregon_v2.h"
 
OregonDecoderV2 decoder;

void setup() {
    Serial.begin(115200);
    decoder.init(2);
    decoder.checkPreamble(false);
}

void loop() {
    uint8_t channel, sensorID, hum;
    int16_t temp    = 0;
    bool    battOK  = true;

    bool received = decoder.isMessageReceived();
    if (received) {
        decoder.dump();
    }
    if (decoder.receiveData(channel, sensorID, temp, hum, battOK)) {
        Serial.print("Sensor "); Serial.print(sensorID, HEX);
        Serial.print(", ch. "); Serial.print(channel);
        Serial.print(":, temp = "); 
        if (temp < 0) { Serial.print("-"); temp *= -1; }
        Serial.print(temp/10); Serial.print("."); Serial.print(temp%10);
        Serial.print(", humidity = "); Serial.print(hum); Serial.print("%");
        if (battOK) {
            Serial.println(", battery OK");
        } else {
            Serial.println(", battery LOW");
        }
    }
    if (received) {
        decoder.reset();
    }
    delay(2000);
}
