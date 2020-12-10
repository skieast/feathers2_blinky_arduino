//#include "Arduino.h"
#include "USB.h"
#include <Adafruit_DotStar.h>

USBCDC USBSerial;


#ifdef  ARDUINO_UMFEATHERS2
  #define STMPE_CS 32
  #define TFT_CS 15
  #define TFT_DC 33
  #define SD_CS 14
  #define NEOPIXEL_PIN 27
  #define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(10))
  #define PIN_APA102_DATA       40
  #define PIN_APA102_SCK        45
  #define PIN_APA102_PWR        21
  #define PIN_LDO2              21
  #define APA102_BRIGHTNESS    175
  #define PIN_LED               13
#endif

#define NUMPIXELS 1
Adafruit_DotStar strip(NUMPIXELS, PIN_APA102_DATA, PIN_APA102_SCK);


static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
  if(event_base == ARDUINO_USB_EVENTS){
    arduino_usb_event_data_t * data = (arduino_usb_event_data_t*)event_data;
    switch (event_id){
      case ARDUINO_USB_STARTED_EVENT:
        Serial.println("USB PLUGGED");
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        Serial.println("USB UNPLUGGED");
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
        break;
      case ARDUINO_USB_RESUME_EVENT:
        Serial.println("USB RESUMED");
        break;

      default:
        break;
    }
  } else if(event_base == ARDUINO_USB_CDC_EVENTS){
    arduino_usb_cdc_event_data_t * data = (arduino_usb_cdc_event_data_t*)event_data;
    switch (event_id){
      case ARDUINO_USB_CDC_CONNECTED_EVENT:
        Serial.println("CDC CONNECTED");
        break;
      case ARDUINO_USB_CDC_DISCONNECTED_EVENT:
        Serial.println("CDC DISCONNECTED");
        break;
      case ARDUINO_USB_CDC_LINE_STATE_EVENT:
        Serial.printf("CDC LINE STATE: dtr: %u, rts: %u\n", data->line_state.dtr, data->line_state.rts);
        break;
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
        Serial.printf("CDC LINE CODING: bit_rate: %u, data_bits: %u, stop_bits: %u, parity: %u\n", data->line_coding.bit_rate, data->line_coding.data_bits, data->line_coding.stop_bits, data->line_coding.parity);
        break;
      case ARDUINO_USB_CDC_RX_EVENT:
        Serial.printf("CDC RX: %u\n", data->rx.len);
        {
            uint8_t buf[data->rx.len];
            size_t len = USBSerial.read(buf, data->rx.len);
            Serial.write(buf, len);
        }
        break;

      default:
        break;
    }
  }
}

void setup()
{
  pinMode(PIN_APA102_PWR,OUTPUT);
  digitalWrite(PIN_APA102_PWR,HIGH);
  strip.begin();
  strip.setBrightness(APA102_BRIGHTNESS);
  strip.show();
  pinMode (PIN_LED, OUTPUT);
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  USB.onEvent(usbEventCallback);
  USB.productName("ESP32S2-USB");
  USB.begin();

  USBSerial.onEvent(usbEventCallback);
  USBSerial.begin(115200);

// The following goes out the default serial port. Or should
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
}

uint32_t color = 0xFF0000;      // 'On' color (starts red)
void loop()
{
  while(1){
    strip.setPixelColor(0, color); // 'On' pixel at head
    strip.show();                     // Refresh strip
    delay(20);
    if((color >>= 8) == 0)          //  Next color (R->G->B) ... past blue now?
      color = 0xFF0000;
    //size_t l = Serial.available();
    //uint8_t b[l];
    //l = Serial.read(b, l);
    //USBSerial.write(b, l);

    digitalWrite(PIN_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED, LOW);
    delay(1000);

    USBSerial.printf("Free Heap: %d \n\r", ESP.getFreeHeap());
    USBSerial.printf("Total Heap: %d\n\r", ESP.getHeapSize());
    USBSerial.printf("Total PSRAM: %d\n\r", ESP.getPsramSize());
    USBSerial.printf("Free PSRAM: %d\n\r", ESP.getFreePsram());
  }
}