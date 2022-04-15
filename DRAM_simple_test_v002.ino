// Arduino Nano simple DRAM tester
// 2022-04-15

#include <Arduino.h>
#include <U8x8lib.h>

#define DI  3 //D3
#define DO  18 //A4
#define CAS 19 //A5
#define RAS 5 //D5
#define WE  4 //D4

#define xA0 6 //D6
#define xA1 8 //D8
#define xA2 7 //D7
#define xA3 16 //A2
#define xA4 15 //A1
#define xA5 14 //A0
#define xA6 17 //A3
#define xA7 13 //D13
#define xA8 2 //D2

#define M_TYPE  12 //D12

#define OLED_SCL 0 //RX0
#define OLED_SDA 1 //TX1

#define BUS_SIZE  9

const unsigned int a_bus[BUS_SIZE] = {
  xA0, xA1, xA2, xA3, xA4, xA5, xA6, xA7, xA8
};

U8X8_SSD1306_128X32_UNIVISION_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  //Serial.begin(57600);
  //while (!Serial); /* wait */
  //Serial.println();
  //Serial.print("DRAM test ");

  u8x8.begin();
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setFont(u8x8_font_pxplusibmcga_r);

  pinMode(DI, OUTPUT);
  pinMode(DO, INPUT);
  pinMode(CAS, OUTPUT);
  pinMode(RAS, OUTPUT);
  pinMode(WE, OUTPUT);

  for (int i = 0; i < BUS_SIZE; i++) {
    pinMode(a_bus[i], OUTPUT);
  }

  pinMode(M_TYPE, INPUT);

  digitalWrite(CAS, HIGH);
  digitalWrite(RAS, HIGH);
  digitalWrite(WE, HIGH);

  for (int i = 0; i < BUS_SIZE; i++) {
    digitalWrite(a_bus[i], LOW);
  }

  char buf[32];
  
  int bus_size;
  int row = 0;
  int col = 0;
  int value = 0;
  int err = 0;
  unsigned long err_all = 0;

  if (digitalRead(M_TYPE)) {
    /* jumper not set - 41256 */
    bus_size = BUS_SIZE;
    //Serial.println("256kx1");
    u8x8.drawString(0,0,"DRAM 256k x1");
  } else {
    /* jumper set - 4164 */
    bus_size = BUS_SIZE - 1;
    //Serial.println("64kx1");
    u8x8.drawString(0,0,"DRAM 64k x1");
  }

  //sprintf(buf, "Addr: $%05X", ((unsigned long)row << bus_size) + col);
  //u8x8.drawString(0,1,buf);
  sprintf(buf, "Test: %u%%", int((float(row+1)/float(1<<bus_size))*100));
  u8x8.drawString(0,2,buf);
  sprintf(buf, "Err: %lu", err_all);
  u8x8.drawString(0,3,buf);


  for (row = 0; row < (1<<bus_size); row++) {
    for (col = 0; col < (1<<bus_size); col++) {
      err = 0;
      value = 1;
      dWrite(row,col,value);
      if (value != dRead(row,col)) err++;
      value = 0;
      dWrite(row,col,value);
      if (value != dRead(row,col)) err++;
      if (err) err_all++;
    }
    //sprintf(buf, "Addr: $%05X", ((unsigned long)row << bus_size) + col);
    //u8x8.drawString(0,1,buf);

    sprintf(buf, "%u%%", int((float(row+1)/float(1<<bus_size))*100));
    u8x8.drawString(6,2,buf);

    sprintf(buf, "%lu", err_all);
    u8x8.drawString(5,3,buf);
  }
  setAddr(0);
  //Serial.print("DRAM ");
  //Serial.println(digitalRead(R_LED)?"FAILED!":"is OK!");

}

void loop() {
}

void setAddr(unsigned int a) {
  for (int i = 0; i < BUS_SIZE; i++) {
    digitalWrite(a_bus[i], a & 1);
    a = a >> 1;
  }
}

void dWrite(int row, int col, int value) {
  // Set row address
  setAddr(row);

  // Pull RAS LOW
  digitalWrite(RAS, LOW);

  // Pull Write LOW (Enables write)
  digitalWrite(WE, LOW);   

  // Set DataIn pin
  digitalWrite(DI, (value & 1)? HIGH : LOW);

  // Set column address
  setAddr(col);

  // Pull CAS LOW
  digitalWrite(CAS, LOW);
  
  // Pull RAS, CAS and Write HIGH
  digitalWrite(RAS, HIGH);
  digitalWrite(CAS, HIGH);
  digitalWrite(WE, HIGH);
}

int dRead(int row, int col) {
  int ret;
  // Set row address
  setAddr(row);
  // Pull RAS LOW
  digitalWrite(RAS, LOW);

  // Set column address
  setAddr(col);

  // Pull CAS LOW
  digitalWrite(CAS, LOW);

  ret = digitalRead(DO);

  // Pull RAS and CAS HIGH
  digitalWrite(RAS, HIGH);
  digitalWrite(CAS, HIGH);

  return ret;
}

void error(int row, int col) {
  //digitalWrite(R_LED, HIGH);
  //digitalWrite(B_LED, LOW);
  //Serial.print("FAILED at ROW: ");
  //Serial.print(row);
  //Serial.print(" / COL: ");
  //Serial.println(col);
  while (1);
}
