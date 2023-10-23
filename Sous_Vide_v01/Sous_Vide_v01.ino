#include "RTClib.h"
#include "OneWireESP32.h"
#include <LiquidCrystal_I2C.h>
#include "device.h"
#include <ESP32Encoder.h>  // https://github.com/madhephaestus/ESP32Encoder.git


TaskHandle_t task_Read_Temperature;
TaskHandle_t task_Read_RTC;
TaskHandle_t task_PID;
TaskHandle_t task_Info;
TaskHandle_t task_Control;

QueueHandle_t xQueueSeconds;
QueueHandle_t xQueue_Time;
QueueHandle_t xQueue_Temperature;
QueueHandle_t xEncoder_Count;


void Task_PID(void* pvParameters) {
  delay(1000);
  for (;;) {
    delay(1000);
  }
};
void Task_Control(void* pvParameters) {
#define CLK 5  // CLK ENCODER
#define DT 18  // DT ENCODER
  long newPosition, oldPosition;

  ESP32Encoder encoder;

  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  for (;;) {

    long newPosition = encoder.getCount();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      xQueueSend(xEncoder_Count, &newPosition, 0);
    }

    delay(50);
  }
}

void Task_Read_RTC(void* pvParameters) {
  RTC_DS3231 rtc;
  while (!rtc.begin()) {
    delay(10);
  }

  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
  DateTime now = rtc.now();
  for (;;) {
    DateTime now = rtc.now();
    //int8_t tick = millis() / 1000;
    int8_t tick = now.second();
    xQueueSend(xQueueSeconds, &tick, 0);
    xQueueSend(xQueue_Time, &now, 0);

    delay(1000);
  }
}


void Task_Info(void* pvParameters) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(" device error report ");

  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.begin();
  // lcd  device error report
  lcd.blink_off();
  lcd.cursor();

  float now_temperature, old_temperature;
  long nowEncoder_Count, oldEncoder_Count;

  DateTime now_time;
  for (;;) {

    xQueueReceive(xQueue_Time, &now_time, /*portMAX_DELAY*/ 0);
    xQueueReceive(xQueue_Temperature, &now_temperature, /*portMAX_DELAY*/ 0);
    xQueueReceive(xEncoder_Count, &nowEncoder_Count, 0);

    uint8_t mess[17];
    //lcd.clear();
    lcd.setCursor(0, 0);
    sprintf((char*)mess, "%02.02f%cC   63.5%cC", now_temperature, 223,223);
    lcd.print((char*)mess);
    //    lcd.print(now_temperature);
    lcd.setCursor(0, 1);
    sprintf((char*)mess, "%02d:%02d:%02d   05:07", now_time.hour(), now_time.minute(), now_time.second());
    lcd.print((char*)mess);
    lcd.setCursor(nowEncoder_Count % 16, 0);


    Serial.print((char*)mess);
    Serial.print('/');
    Serial.print(now_temperature);
    Serial.print('/');

    Serial.print("Encoder");
    Serial.print(nowEncoder_Count);
    Serial.print('/');

    Serial.print(now_time.year(), DEC);
    Serial.print('/');
    Serial.print(now_time.month(), DEC);
    Serial.print('/');
    Serial.print(now_time.day(), DEC);
    Serial.print(" (");
    Serial.print(") ");
    Serial.print(now_time.hour(), DEC);
    Serial.print(':');
    Serial.print(now_time.minute(), DEC);
    Serial.print(':');
    Serial.print(now_time.second(), DEC);
    Serial.println();

    delay(200);
  }
}

void Task_Read_Temperature(void* pvParameters) {
  float currTemp[MaxDevs];

  OneWire32 ds(23, 0, 1, 0);  //gpio pin, tx, rx, parasite power There are 8 RMT channels (0-7) available on ESP32 for tx/rx
  uint64_t addr[MaxDevs];

  uint8_t devices = ds.search(addr, MaxDevs);  //to find addresses
  for (uint8_t i = 0; i < devices; i += 1) {
    //Serial.printf("%d: 0x%llx,\n", i, addr[i]);
  }

  for (;;) {
    ds.request();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    for (byte i = 0; i < MaxDevs; i++) {
      uint8_t err = ds.getTemp(addr[i], currTemp[i]);
      if (err) {
        const char* errt[] = { "", "CRC", "BAD", "DC", "DRV" };
        float error = -1 * err;
        xQueueSend(xQueue_Temperature, &error, 0);

        // Serial.print(i);
        // Serial.print(": ");
        // Serial.println(errt[err]);
      } else {
        float temperature = currTemp[0];
        xQueueSend(xQueue_Temperature, &temperature, 0);

        // Serial.print(i);
        // Serial.print(": ");
        // Serial.println(currTemp[i]);
      }
    }

    delay(2000 - 750);
  }
}


void setup() {



  xQueueSeconds = xQueueCreate(10, sizeof(uint8_t));
  xQueue_Time = xQueueCreate(10, sizeof(DateTime));
  xQueue_Temperature = xQueueCreate(10, sizeof(float));
  xEncoder_Count = xQueueCreate(1, sizeof(long));



  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task_Read_Temperature,  /* Task function. */
    "Read_Temperature",     /* name of task. */
    2048,                   /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &task_Read_Temperature, /* Task handle to keep track of created task */
    0);                     /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
    Task_Read_RTC,  /* Task function. */
    "Read_RTC",     /* name of task. */
    2048,           /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &task_Read_RTC, /* Task handle to keep track of created task */
    0);             /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task_PID,  /* Task function. */
    "PID",     /* name of task. */
    2048,      /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &task_PID, /* Task handle to keep track of created task */
    0);        /* pin task to core 1 */
  delay(500);

  xTaskCreatePinnedToCore(
    Task_Info,  /* Task function. */
    "Info",     /* name of task. */
    2048,       /* Stack size of task */
    NULL,       /* parameter of the task */
    1,          /* priority of the task */
    &task_Info, /* Task handle to keep track of created task */
    0);         /* pin task to core 1 */
  delay(500);

  xTaskCreatePinnedToCore(
    Task_Control,  /* Task function. */
    "Control",     /* name of task. */
    2048,          /* Stack size of task */
    NULL,          /* parameter of the task */
    1,             /* priority of the task */
    &task_Control, /* Task handle to keep track of created task */
    0);            /* pin task to core 1 */
  delay(500);
}


void loop() {
}