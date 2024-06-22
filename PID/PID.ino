#include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define MPU6050_ADDRESS 0x68 // Địa chỉ I2C của cảm biến MPU6050

Servo right_prop; // Khai báo đối tượng Servo cho cánh quạt phải
Servo left_prop; // Khai báo đối tượng Servo cho cánh quạt trái
Adafruit_MPU6050 mpu;  // Khai báo đối tượng cảm biến MPU6050

float Total_angle[2]; // Mảng để lưu góc đo được từ cảm biến
float desired_angle = 0; // Góc mong muốn (góc đích)

unsigned long startMillis;  // Thời gian bắt đầu
float elapsedTime, current_time, timePrev; // Thời gian đã trôi qua, thời gian hiện tại, và thời gian trước đó

float PID, error, previous_error;   // Giá trị PID, lỗi hiện tại và lỗi trước đó
int pwmLeft, pwmRight; // Giá trị PWM cho cánh quạt trái và phải
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
double kp = 4.45;   // Hệ số P của bộ điều khiển PID
double ki = 0.0075;   // Hệ số I của bộ điều khiển PID
double kd = 2.65; // Hệ số D của bộ điều khiển PID
double throttle = 1200;   // Giá trị ga (throttle)
float rad_to_deg = 180 / 3.141592654;   // Hệ số chuyển đổi từ radian sang độ

void setup() {
  Serial.begin(115200);  // Khởi động Serial với tốc độ 115200 baud
  right_prop.attach(5);   // Gán chân GPIO 5 cho servo cánh quạt phải
  left_prop.attach(25);   // Gán chân GPIO 25 cho servo cánh quạt trái
  left_prop.writeMicroseconds(1000);   // Đặt giá trị ban đầu cho servo trái
  right_prop.writeMicroseconds(1000);  // Đặt giá trị ban đầu cho servo phải
  delay(7000);   // Chờ 7 giây
  mpu.begin(MPU6050_ADDRESS, &Wire);   // Khởi động cảm biến MPU6050
}

void loop() {
  timePrev = millis();   // Lưu lại thời gian hiện tại
  current_time = millis();  // Cập nhật thời gian hiện tại
  elapsedTime = (current_time - timePrev) / 1000;   // Tính toán thời gian đã trôi qua


// Nếu đã trôi qua 50ms, in ra giá trị góc và PWM
  if (millis() - startMillis >= 50) {
    Serial.print(Total_angle[0]);
    Serial.print("  ");
    Serial.print(pwmLeft);
    Serial.print("  ");
    Serial.print(pwmRight);
    Serial.print(" ");
    Serial.print(error);
    Serial.println("  ");
    startMillis = millis();  // Cập nhật thời gian bắt đầu
  }

  sensors_event_t accel, gyro, temp;  // Khai báo các biến để lưu giá trị đo được từ cảm biến
  mpu.getEvent(&accel, &gyro, &temp);   // Lấy giá trị đo được từ cảm biến


// Tính toán góc dựa trên giá trị gia tốc đo được
  Total_angle[0] = atan(accel.acceleration.y / sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.z, 2))) * rad_to_deg;
  Total_angle[1] = atan(-1 * accel.acceleration.x / sqrt(pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2))) * rad_to_deg;
  error = Total_angle[1] - desired_angle;  // Tính toán lỗi (góc đo được - góc mong muốn)

  pid_p = kp * error;  // Tính toán thành phần P của PID

// Tính toán thành phần I của PID nếu lỗi trong khoảng -2.5 đến 2.5

  if (-2.5 < error && error < 2.5) {
    pid_i += ki * error;
    pid_i = constrain(pid_i, -500, 500);  // Giới hạn giá trị pid_i trong khoảng -500 đến 500
  }

// Tính toán thành phần D của PID

  if (elapsedTime != 0) {
    pid_d = kd * ((error - previous_error) / elapsedTime);
  } else {
    // Xử lý khi elapsedTime bằng 0, ví dụ gán giá trị mặc định cho pid_d
    pid_d = 0;
  }

  PID = pid_p + pid_i + pid_d;  // Tính toán tổng giá trị PID
  PID = constrain(PID, -1000, 1000);  // Giới hạn giá trị PID trong khoảng -1000 đến 1000

  pwmRight = throttle + PID;  // Tính toán giá trị PWM cho cánh quạt phải
  pwmLeft = throttle - PID;  // Tính toán giá trị PWM cho cánh quạt trái
  
  pwmLeft = constrain(pwmLeft, 1100, 1600);  // Giới hạn giá trị PWM cho cánh quạt trái trong khoảng 1100 đến 1600
  pwmRight = constrain(pwmRight, 1100, 1600);  // Giới hạn giá trị PWM cho cánh quạt phải trong khoảng 1100 đến 1600

  left_prop.writeMicroseconds(pwmLeft);  // Gửi giá trị PWM tới servo trái
  right_prop.writeMicroseconds(pwmRight);  // Gửi giá trị PWM tới servo phải
  previous_error = error;  // Cập nhật lỗi trước đó
}
