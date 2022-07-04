#include <Arduino_FreeRTOS.h>

//The functions are defined
void Task_LED_blink(void *param);
void Task_Print(void *param);

//Define task handle
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

void setup() {
  Serial.begin(9600);
  //Create the task
  xTaskCreate(Task_Print1,"Task1",100,NULL,1,&Task_Handle1);
  xTaskCreate(Task_Print2,"Task2",100,NULL,1,&Task_Handle2);

  pinMode(8,INPUT);
  pinMode(13,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void Task_LED_blink(void *param){
  (void) param;

  if(digitalRead(8) == 0){
    while(digitalRead(8) == 0){
    digitalWrite(13,LOW);
    }else {digitalWrite(13,HIGH)
    vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    }
  }

void Task_print(void *param){
  (void) param;

  for(;;){
    Serial.print("Task 1");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  }
