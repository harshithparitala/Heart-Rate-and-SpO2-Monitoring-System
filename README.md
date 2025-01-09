# Heart-Rate-and-SpO2-Monitoring-System

## Project Overview
This project implements a real-time heart rate and SpO2 monitoring system. The transmitter is based on an Arduino board using a MAX30100 pulse oximeter sensor, and the receiver uses the VSD Squadron Mini board equipped with a CH32V processor and an I2C-interfaced LCD display.

The data is transmitted wirelessly via an HC-05 Bluetooth module connected to the Arduino (transmitter) and displayed on the VSD Squadron Mini board's LCD (receiver).

---

## Features
1. Real-time monitoring of heart rate and SpO2.
2. Wireless data transmission via Bluetooth (HC-05 module).
3. Display on I2C LCD connected to the VSD Squadron Mini board.
4. Reliable and accurate data averaging over a configurable period.
5. Modular code structure for easy adaptability.

---

## Pin Configurations
### Arduino (Transmitter)
| Component          | Pin  | Function       |
|--------------------|------|----------------|
| HC-05 (Bluetooth)  | TX   | Transmit Data  |
| HC-05 (Bluetooth)  | RX   | Receive Data   |
| MAX30100           | SDA  | A4             |
| MAX30100           | SCL  | A5             |
| MAX30100           | VCC  | 3.3V           |
| MAX30100           | GND  | GND            |
| HC-05              | VCC  | 5V             |
| HC-05              | GND  | GND            |

### VSD Squadron Mini Board (Receiver)
| Component          | Pin   | Function       |
|--------------------|-------|----------------|
| HC-05 (Bluetooth)  | PD5   | RX             |
| HC-05 (Bluetooth)  | PD6   | TX             |
| I2C LCD            | SDA   | PC1            |
| I2C LCD            | SCL   | PC2            |
| I2C LCD            | VCC   | 3.3V           |
| I2C LCD            | GND   | GND            |

---

## Novelty
1. Integration of wireless communication with real-time bio-signal processing.
2. Efficient averaging technique to ensure accurate and stable readings.
3. Low-cost implementation utilizing widely available components.
4. Compatibility with VSD Squadron Mini board enhances usability for educational and experimental purposes.

---

## Code

### Transmitter Code (Arduino)
```cpp
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000
#define AVERAGING_PERIOD_SEC 5

PulseOximeter pox;
uint32_t tsLastReport = 0;
uint8_t measurementCount = 0;
float heartRateSum = 0;
float spo2Sum = 0;

void setup() {
    Serial.begin(9600); // Bluetooth communication
    if (!pox.begin()) {
        while (true); // Halt if initialization fails
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
}

void loop() {
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        heartRateSum += pox.getHeartRate();
        spo2Sum += pox.getSpO2();
        measurementCount++;

        if (measurementCount >= AVERAGING_PERIOD_SEC) {
            float avgHeartRate = heartRateSum / measurementCount;
            float avgSpO2 = spo2Sum / measurementCount;

            Serial.print("HR:");
            Serial.print(avgHeartRate, 2);
            Serial.print(",SPO2:");
            Serial.println(avgSpO2, 2);

            heartRateSum = 0;
            spo2Sum = 0;
            measurementCount = 0;
        }
        tsLastReport = millis();
    }
}
```

### Receiver Code (VSD Squadron Mini Board)
```c
#include "ch32v00x.h"

#define SDA_PIN GPIO_Pin_1
#define SCL_PIN GPIO_Pin_2
#define LCD_Address 0x27

void delay_ms(unsigned int ms);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_string(const char *str);
void lcd_init(void);

void setup() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SDA_PIN | SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    lcd_init();
    lcd_send_string("Waiting...");
}

void loop() {
    static char buffer[32];
    static int index = 0;

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
        char receivedChar = USART_ReceiveData(USART1);

        if (receivedChar == '\n') {
            buffer[index] = '\0';
            lcd_send_cmd(0x80);
            lcd_send_string("Heart rate: ");
            lcd_send_string(buffer);
            index = 0;
        } else if (index < sizeof(buffer) - 1) {
            buffer[index++] = receivedChar;
        }
    }
}
```

---

## How to Use
1. Connect the MAX30100 and HC-05 to the Arduino as per the pin configuration.
2. Connect the HC-05 and LCD to the VSD Squadron Mini board as per the pin configuration.
3. Load the transmitter code onto the Arduino.
4. Load the receiver code onto the VSD Squadron Mini board.
5. Power both boards and observe the readings on the LCD.

---

## Demonstration 

[Implementation Video Link](https://drive.google.com/file/d/1nXNX1kRaElo2Wi4rP-v26Or7xCAF0exa/view?usp=sharing)
