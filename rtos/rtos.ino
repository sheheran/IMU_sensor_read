#include <Arduino_FreeRTOS.h>
//Define Task functions
void Task_print1(void *param);
void Task_print2(void *param);
//Define Taask handle
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

void setup() {
  Serial.begin(9600);
  //Create the tasks and assign prority
  xTaskCreate(Task_Print1, "Task1", 100, NULL, 1, &Task_Handle1);
  xTaskCreate(Task_Print2, "Task2", 100, NULL, 1, &Task_Handle2);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void Task_Print1(void *param) {
  (void) param;
  int i = 0;
  while (1) {
    Serial.println("Task");
    if (i < 1) {
      Serial.println("Task 1");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      i++;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_Print2(void *param) {
  (void) param;
  while (1) {
    Serial.println("Task 2");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
