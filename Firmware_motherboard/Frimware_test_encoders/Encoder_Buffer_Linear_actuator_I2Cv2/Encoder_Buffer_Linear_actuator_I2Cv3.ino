#include <SPI.h>
//#include <Encoder_Buffer.h>
#include <LS7366.h>
#include <Wire.h>

int i2cAddress = 0x42;
//const unsigned int MAX_INPUT = 3;

#define EncoderCS1 7 //grupo 1
#define EncoderCS2 8 //grupo 2

#define D0 5 //IN1 1
#define D1 6 //IN2 1
#define D2 3  //IN3 2 2 to 3
#define D3 9  //IN4 2 4 to 9


//////////////////////////////////////////////////////////////////////
int pulsexmm = 17.1;
int position = 0;
int varL = 0;
int varR = 0;

String INP;

//////////////////////////////////////////////////////////////////////
long encoderLReading = 0;
long encoderRReading = 0;

LS7366 EncoderL(EncoderCS1);  //8 is the chip select pin.
LS7366 EncoderR(EncoderCS2);  //8 is the chip select pin.

//Encoder_Buffer EncoderL(EncoderCS1);
//Encoder_Buffer EncoderR(EncoderCS2);

/////////////////////////////////////////////////////////////////////
//                      Functions
////////////////////////////////////////////////////////////////////
void full_up()
{

  analogWrite(D0, 0);
  analogWrite(D2, 0);

  delayMicroseconds(50);

  analogWrite(D1, 255);
  analogWrite(D3, 255);

  delay(4000);
  analogWrite(D0, 0);
  analogWrite(D1, 0);
  analogWrite(D2, 0);
  analogWrite(D3, 0);

  //EncoderL.clearEncoderCount();
  //EncoderR.clearEncoderCount();
  EncoderR.clear_counter();
  EncoderL.clear_counter();
  delay(100);
  encoderLReading = EncoderL.read_counter();//Read Encoder
  encoderRReading = EncoderR.read_counter();//Read Encoder
}

void full_down()
{
  analogWrite(D1, 0);
  analogWrite(D3, 0);
  delayMicroseconds(50);
  analogWrite(D0, 255);
  analogWrite(D2, 255);

  delay(4000);

  analogWrite(D0, LOW);
  analogWrite(D1, LOW);
  analogWrite(D2, LOW);
  analogWrite(D3, LOW);
  delay(500);
  encoderLReading = EncoderL.read_counter();//Read Encoder
  encoderRReading = EncoderR.read_counter();//Read Encoder
}


void go_pos(int x)
{
  //Serial.println(x);

  analogWrite(D1, 0);
  analogWrite(D3, 0);
  delayMicroseconds(50);
  analogWrite(D0, 45);
  analogWrite(D2, 45);

  varL = 0;
  varR = 0;

  EncoderL.clear_counter();
  EncoderR.clear_counter();

  while ( varL == 0 or varR == 0 ) {
    encoderLReading = EncoderL.read_counter();//Read Encoder
    encoderRReading = EncoderR.read_counter();//Read Encoder

    Serial.print("Encoder L: ");
    Serial.print(encoderLReading);
    Serial.print("Encoder R: ");
    Serial.println(encoderRReading);

    if ( encoderLReading > encoderRReading + 2 ) {
      analogWrite(D2, 35);
    }
    else if (varR == 0)
    {
      analogWrite(D2, 45);
    }

    if ( encoderRReading > encoderLReading + 2 ) {
      analogWrite(D0, 35);
    }
    else if (varL == 0)
    {
      analogWrite(D0, 45);
    }

    if (encoderRReading >= x) {
      digitalWrite(D0, LOW);
      varL = 1;
    }

    if (encoderLReading >= x) {
      digitalWrite(D2, LOW);
      varR = 1;
    }
  }
}

int state = 0;
int pos_r = 0;

void receiveEvent(int bytes) {
  state = Wire.read();    // read one character from the I2C

  Serial.println(state);
}

void setup() {
  Wire.begin(i2cAddress);                // join i2c bus with address #0x42
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(sendData);

  Serial.begin(9600);

  pinMode(EncoderCS1, OUTPUT);
  pinMode(EncoderCS2, OUTPUT);

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);

  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  analogWrite(D0, 0);
  analogWrite(D1, 0);
  analogWrite(D2, 0);
  analogWrite(D3, 0);
  SPI.begin();

  EncoderR.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  EncoderR.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  EncoderR.clear_counter();
  EncoderR.clear_status_register();
  EncoderR.write_data_register(4);

  EncoderL.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  EncoderL.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  EncoderL.clear_counter();
  EncoderL.clear_status_register();
  EncoderL.write_data_register(4);

  //EncoderL.initEncoder();
  //EncoderL.clearEncoderCount();
  //EncoderR.initEncoder();
  //EncoderR.clearEncoderCount();

}

void loop() {


  if (state == 1)
  {
    full_up();
    sendData();
    state = 0;
  }
  if (state == 2)
  {
    full_down();
    sendData();
    state = 0;
  }
  if (state >= 7 && state <= 40 )
  {
    Serial.println("Position ");
    state = state * pulsexmm;
    go_pos(state);
    sendData();
    state = 0;
  }
}

void sendData() {
  String data;
  int left;
  int right;
  left = int (encoderLReading);
  right = int (encoderRReading);
  Serial.println(left);
  //data= String( String(left) + "*" + String(right));
  data = String( "L" + String(left) + "*R" + String(right));
  //Wire.beginTransmission(2); // transmit to device
  Wire.write(data.c_str());
  //Wire.endTransmission(); // stop transmitting
}
