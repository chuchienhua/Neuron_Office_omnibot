// #define DEBUG

// Pin Definitions
#define LED_R 2
#define LED_Y 4
#define LED_G 6
#define BUTTON 21

// Message Frame
#define FRAME_LEN 16 // Length of bytes of a single control message frame (including SOF, EOF and CRC16)
#define DATA_LEN 12  // Length of bytes of the data field
#define RX_BUFFER_SIZE 64
#define FAILSAFE_TIMEOUT (100) // 100ms failsafe timeout, disable motors automatically

// User Frame Variables
char rxBuff[RX_BUFFER_SIZE];   // Serial receive buffer
int rxIndex = 0;               // Serial receive buffer index
unsigned long lastRxTime = 0;  // Last valid received command time in milliseconds
unsigned long rxCount = 0;     // Valid received message count
float vx = 0.0f;               // x-axis velocity (m/s) (暫無使用)
float vy = 0.0f;               // y-axis velocity (m/s) (暫無使用)
float w = 0.0f;                // angular velocity (m/s) (暫無使用)
bool emergencyStopFlag = 0;    // 0: Normal mode, 1: Emergency Stop
uint8_t button_status = 0x00;  // if button pressed, 0xAA; else 0x00 (小車子上button按鈕)
uint8_t onesitearrived = 0x00; // 0XBB:green light, 0XBC:red light, 0XBD:all close light(小車子上三色燈)

// button deliver msg to ROS
int button_value;
unsigned long button_time;
unsigned long delay_deliver_time;
bool button_flag = false;
uint8_t button_MSG = 0x00;

// Message Frame Declaration
typedef struct __attribute__((__packed__))
{
  uint8_t sof;
  union
  {
    uint8_t data[DATA_LEN];
    struct
    {
      float vx;
      uint8_t onesitearrived;
      uint8_t button_status;
    };
  };
  uint16_t checksum;
  uint8_t eof;
} CtrlMsg;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(1115200);

  /*-------------Pin Setup-------------*/
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(LED_G, LOW);
  button_time = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  handleReceivedMessage();
  buttonPressed();
}

void buttonPressed()
{
  button_value = digitalRead(BUTTON);
  if (button_value == LOW)
  {
    button_MSG = 0x00;
  }
  else
  {
    button_flag = true;
  }
  if (button_flag == true)
  {
    if (millis() - button_time < 4500)
    {
      digitalWrite(LED_Y, HIGH);
      if (millis() - delay_deliver_time > 2000)
      {
        button_MSG = 0xAA;
      }
    }
    else
    {
      button_flag = false;
    }
  }
  else
  {
    digitalWrite(LED_Y, LOW);
    button_time = millis();
    delay_deliver_time = millis();
  }
}

void serialEvent()
{
  if (Serial.available())
  {
    rxBuff[rxIndex++] = Serial.read();
    if (rxIndex >= RX_BUFFER_SIZE)
    {
      rxIndex = 0;
    }
  }
}

void handleReceivedMessage()
{
  if (rxIndex < FRAME_LEN)
    return;                                               // Check message length
  CtrlMsg *msg = (CtrlMsg *)&rxBuff[rxIndex - FRAME_LEN]; // Convert data to message frame
  if (msg->eof != 0xCD)
    return; // Check EOF = 0xCD
  if ((msg->sof & 0xFE) != 0xAA)
    return; // Check SOF = 0xAA or 0xAB
  if (msg->checksum != checkSum(msg->data, DATA_LEN))
    return;

  emergencyStopFlag = msg->sof & 0x01;

  unsigned long delta_t = millis() - lastRxTime;
  vx = msg->vx;
  onesitearrived = msg->onesitearrived;
  button_status = msg->button_status;

  //---led---//
  if (msg->onesitearrived == 187) // 0XBB Arrived site green light
  {
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_R, LOW);
  }
  else if (msg->onesitearrived == 188)
  {
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, HIGH);
  }
  else // 0XBD
  {
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, LOW);
  }

  // Send response
#ifdef DEBUG
  Serial.println("Valid message");
  Serial.print("emergencyStopFlag = ");
  Serial.println(emergencyStopFlag);
  Serial.print("Vx = ");
  Serial.println(vx);
  Serial.print("onesitearrived = ");
  Serial.println(onesitearrived);
  Serial.print("button_status = ");
  Serial.println(button_status);
#else
  sendResponse();
#endif
  rxCount++;
  lastRxTime = millis();
  rxIndex = 0;
}

void sendResponse()
{
  CtrlMsg msg;
  msg.sof = 0xAA | emergencyStopFlag;
  msg.eof = 0xCD;
  msg.vx = vx;
  msg.onesitearrived = onesitearrived;
  msg.button_status = button_MSG;
  msg.checksum = checkSum(msg.data, DATA_LEN);

  Serial.write((uint8_t *)&msg, FRAME_LEN);
}

inline uint16_t checkSum(const char *buf, int len)
{
  uint16_t sum = 0;
  for (int i = 0; i < len / 2; i++)
  {
    sum ^= *(((uint16_t *)buf) + i);
  }

  return sum;
}
