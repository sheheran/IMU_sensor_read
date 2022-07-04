#include <Arduino_FreeRTOS.h>
#include <Wire.h>

//Defining the functions
void Task_PrintIMU_Acc_data(void *param);
void Task_PrintIMU_Gyro_data(void *param);

//Define task handle
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

//Print IMU data ax, ay, az, (Accelerometer) and gx, gy, gz (Gyroscope) data in serial monitor
const int MPU = 0x68;
//Initialize variables
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;

void setup() {
  Serial.begin(9600);
  Serial.println("Assign Parameters to the sensor");
  Serial.println("Talking to 6B register");
  Wire.beginTransmission(MPU); //Start the communction with 0x68 MPU6050
  Wire.write(0x6B); //talk to register 6B
  Wire.write(0x101);
  Wire.endTransmission(true);

  //Set sample rate to 500Hz
  Serial.println("Setting the sample rate to 500Hz");
  Wire.beginTransmission(MPU);
  Wire.write(0x19);
  Wire.write(0x0F);
  Wire.endTransmission(true);

  //Activate DLPF Digital gyro low pass filter to 188Hz
  Serial.println("Setting DLPF gyro to 188Hz");
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x01);
  Wire.endTransmission(true);

  //Set Gyro range to 2000DPS (degrees per secoend)
  Serial.println("Set gyro to 2000 degrees per secoend");
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  //Set Accelerometer range to 8G (degrees per secoend)
  Serial.println("Set gyro to 8G gravitational acceleration");
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  //Create the tasks and assign prority
  xTaskCreate(Task_PrintIMU_Acc_data, "Task1", 100, NULL, 2, &Task_Handle1);
  xTaskCreate(Task_PrintIMU_Gyro_data, "Task2", 100, NULL, 1, &Task_Handle2);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void Task_PrintIMU_Acc_data(void *param) {
  (void) param;
  while (1) {
    Serial.println("Printing IMU Acceleration data");
    //Read accelerometer data//
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    //Print data
    Serial.println(AccX);
    Serial.println(AccY);
    Serial.println(AccZ);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task_PrintIMU_Gyro_data(void *param) {
  (void) param;
  while (1) {
    Serial.println("Printing IMU Gyro data");
    //Read Gyroscope data
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroAngleX = gyroAngleX + GyroX * elapsedTime;
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;
    //Print data
    Serial.println(gyroAngleX);
    Serial.println(gyroAngleY);
    Serial.println(gyroAngleZ);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
