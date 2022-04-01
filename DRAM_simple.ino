// Arduino Nano DRAM tester
// 2022-03-16

#define DI  3 //D3
#define DO  14 //A0
#define CAS 15 //A1
#define RAS 5 //D5
#define WE  4 //D4

#define xA0 6 //D6
#define xA1 8 //D8
#define xA2 7 //D7
#define xA3 9 //D9
#define xA4 10 //D10
#define xA5 11 //D11
#define xA6 13 //D13
#define xA7 12 //D12
#define xA8 2 //D2

#define M_TYPE  16 //A2
#define R_LED 17 //A3
#define G_LED 18 //A4
#define B_LED 19 //A5

#define BUS_SIZE  9

const unsigned int a_bus[BUS_SIZE] = {
  xA0, xA1, xA2, xA3, xA4, xA5, xA6, xA7, xA8
};

void setup() {
  Serial.begin(115200);
  while (!Serial); /* wait */
  Serial.println();
  Serial.print("DRAM test ");

  pinMode(DI, OUTPUT);
  pinMode(DO, INPUT);
  pinMode(CAS, OUTPUT);
  pinMode(RAS, OUTPUT);
  pinMode(WE, OUTPUT);

  for (int i = 0; i < BUS_SIZE; i++) {
    pinMode(a_bus[i], OUTPUT);
  }

  pinMode(M_TYPE, INPUT);
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);

  digitalWrite(CAS, HIGH);
  digitalWrite(RAS, HIGH);
  digitalWrite(WE, HIGH);

  for (int i = 0; i < BUS_SIZE; i++) {
    digitalWrite(a_bus[i], LOW);
  }

  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);
  digitalWrite(B_LED, LOW);

  int bus_size;
  int row = 0;
  int col = 0;
  int value = 0;
  int err = 0;
  int err_all = 0;

  if (digitalRead(M_TYPE)) {
    /* jumper not set - 41256 */
    bus_size = BUS_SIZE;
    Serial.println("256kx1");
  } else {
    /* jumper set - 4164 */
    bus_size = BUS_SIZE - 1;
    Serial.println("64kx1");
  }

  for (row = 0; row < (1<<bus_size); row++) {
    Serial.print("ROW ");
    Serial.print(row);
    digitalWrite(B_LED, !digitalRead(B_LED));
    err = 0;

    for (col = 0; col < (1<<bus_size); col++) {
      value = 1;
      dWrite(row,col,value);
      if (value != dRead(row,col)) err++;
      value = 0;
      dWrite(row,col,value);
      if (value != dRead(row,col)) err++;
    }
    
  if (err) digitalWrite(R_LED, HIGH);
  Serial.println(err!=0?" ERR!":" OK!");
  }

  Serial.print("DRAM ");
  Serial.println(digitalRead(R_LED)?"FAILED!":"is OK!");

  digitalWrite(G_LED, digitalRead(R_LED)?LOW:HIGH);
  digitalWrite(B_LED, LOW);
}

void loop() {
}

void setAddr(unsigned int a) {
  for (int i = 0; i < BUS_SIZE; i++) {
    digitalWrite(a_bus[i], a & 1);
    a /= 2;
  }
}

void dWrite(int row, int col, int value) {
  // Set row address
  setAddr(row);

  // Pull RAS LOW
  digitalWrite(RAS, LOW);

  // Pull Write LOW (Enables write)
  digitalWrite(WE, LOW);   

  // Set Data in pin to HIGH (write a one)
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
  digitalWrite(R_LED, HIGH);
  digitalWrite(B_LED, LOW);
  Serial.print("FAILED at ROW: ");
  Serial.print(row);
  Serial.print(" / COL: ");
  Serial.println(col);
  while (1);
}
