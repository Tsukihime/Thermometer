#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <microDS18B20.h>

RF24 radio(16, 10);

const uint8_t address[6] = { "NrfMQ" };
const uint8_t channel = 0x6f;

MicroDS18B20<14> sensor;

void initRadio() {
    radio.begin();                   // инициализация
    radio.setPALevel(RF24_PA_HIGH);  // уровень питания усилителя RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
    radio.enableDynamicPayloads();
    radio.setDataRate(RF24_1MBPS);   // скорость обмена данными RF24_1MBPS или RF24_2MBPS
    radio.setCRCLength(RF24_CRC_8);  // размер контрольной суммы 8 bit или 16 bit
    radio.setChannel(channel);       // установка канала
    radio.setAutoAck(false);         // автоответ
    radio.openWritingPipe(address);  // открыть трубу на отправку  
    radio.stopListening();           // радиоэфир не слушаем, только передача
}

void initWatchdog() {
    MCUSR &= ~(1 << WDRF);              // Just to be safe since we can not clear WDE if WDRF is set
    cli();                              // disable interrupts so we do not get interrupted while doing timed sequence
    WDTCSR |= (1 << WDCE) | (1 << WDE); // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
    WDTCSR = (1 << WDP1) | (1 << WDP2)  // timeout in 1 second, disable reset mode. Must be done in one operation
    | (1 << WDIE);                      // enable watchdog interrupt only mode 
    sei();
}

ISR(WDT_vect) {}

uint16_t getBatteryVoltage() {
    ADMUX = (0 << REFS1) | (1 << REFS0) |                          // select AVCC as reference
            (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // measure bandgap reference voltage
    ADCSRA = (1 << ADEN)                                           // enable ADC
           | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);           // ADC Prescaler Selections Div8
    _delay_us(500);                            // a delay rather than a dummy measurement is needed to give a stable reading!
    ADCSRA |= (1 << ADSC);                     // start conversion
    while(bit_is_set(ADCSRA, ADSC));           // wait to finish
    uint16_t voltage = (1080UL * 1023 / ADC);  // AVcc = Vbg / ADC * 1023 = 1.1V * 1023 / ADC
    ADCSRA &= ~(1 << ADEN);                    // disable ADC
    return voltage;
}

void requestTemp() {
    PORTC |= (1 << PC1);  // sensor power up
    sensor.requestTemp();
}

int16_t getTemperature() {
    int16_t temp = sensor.getRaw() * 10 / 16; // temp in 0.1C
    PORTC &= ~(1 << PC1); // sensor power down
    return temp;
}

void enterSleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

void sendMQTTMessage(const char* topic, const char* payload) {
    char buffer[32];
    uint8_t topicLength = strlen(topic);
    uint8_t payloadLength = strlen(payload);
    uint8_t messageLength = topicLength + payloadLength + 2;
    if(messageLength > 32) return;

    buffer[0] = topicLength;
    buffer[1] = payloadLength;
    memcpy(&buffer[2], topic, topicLength);
    memcpy(&buffer[2 + topicLength], payload, payloadLength);
    radio.write(buffer, messageLength);
}

char* uintToStr(uint16_t value, char buffer[5]) {
    uint32_t subtrahend = 10000;
    for (uint8_t i = 0; i < 5; i++) {
        buffer[i] = '0';
        while (value >= subtrahend) {
            value -= subtrahend;
            buffer[i]++;
        }
        subtrahend /= 10;
    }
    for(uint8_t i = 0; i < 5; i++) {
        if(buffer[i] != '0') return &buffer[i];
    }
    return &buffer[4];
}

void sendBatteryVoltage(uint16_t voltage) {
    char payload[5];
    uintToStr(voltage, payload);
    payload[0] = payload[1];
    payload[1] = '.';
    payload[4] = '\0';
    sendMQTTMessage("thermometer/batt", payload);
}

void sendTemperature(int16_t temperature) {
    char buff[5 + 3];
    bool negative = temperature < 0;
    uint16_t temp = (negative) ? -temperature : temperature;

    char* payload = uintToStr(temp, &buff[1]);
    buff[7] = '\0';
    buff[6] = buff[5];
    buff[5] = '.';

    if(negative) {
        payload--;
        payload[0] = '-';
    }
    sendMQTTMessage("thermometer/temp", payload);
}

void setup() {
    DDRC |= (1 << PC1); // output
    PORTC |= (1 << PC1); // sensor power up
    sensor.setResolution(12, 0);
    PORTC &= ~(1 << PC1); // sensor power down

    initRadio();
    initWatchdog();
    sendMQTTMessage("thermometer", "Start!");
}

void loop() {
    const uint16_t period = 60 * 10; // 1sec * 60 * 10 = update every 10 min
    static uint8_t counter = 0;

    switch(counter) {
        case 0:
            requestTemp();
            break;

        case 1:
            int16_t temp = getTemperature();
            uint16_t voltage = getBatteryVoltage();
            radio.powerUp();
            sendBatteryVoltage(voltage);
            sendTemperature(temp);
            radio.powerDown();
            break;        
    }

    if(++counter >= period) counter = 0;
    enterSleep();
}
