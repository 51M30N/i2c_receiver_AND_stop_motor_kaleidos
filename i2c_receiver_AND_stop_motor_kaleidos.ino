// Wire Slave Receiver & step motor
// by 51m30N for k4leidos
// Created 03/01/2021

//Import the library required
#include <Wire.h>
//Slave Address for the Communication
#define SLAVE_ADDRESS 0x15
// i2c baudrate
#define BAUD 9600 //

// Define stepper motor connections and steps per revolution:
#define dirPinZ 4
#define stepPinZ 7
#define dirPinX 3
#define stepPinX 6
#define dirPinY 2
#define stepPinY 5
#define stepsPerRevolution 200

unsigned long number;
bool change = false;
String mode;
int serial_in;
//MOTOR var
byte count; //
byte motor = 0; //motor X Y Z <=> 0 1 2
byte Direction[3] = {0, 0, 0};// 0 up 1 down (range from 0 to 255
unsigned long Nombre_de_pas[4] = {0, 0, 0, 0}; //range from 0 to 4,294,967,295 (2^32 - 1)
unsigned int Vitesse = 0; // range of 0 to 65,535


void setup() {
  // Declare pins as output:
  pinMode(stepPinZ, 7);
  pinMode(dirPinZ, 4);
  pinMode(stepPinX, 6);
  pinMode(dirPinX, 3);
  pinMode(stepPinY, 5);
  pinMode(dirPinY, 2);
  //// i2c config
  
  Wire.begin(SLAVE_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveData); // register event
  Serial.begin(BAUD);           // start serial for output
}

void loop() {
  delay(10);
  if (change == true){
      change = false;
      move_XYZ();
  }
}


// callback for received data
void receiveData(int byteCount) {
  // nomenclature
  while (Wire.available()) {
    serial_in = Wire.read();
    //Serial.print("serial_in-");
    //Serial.println(serial_in);
  }
  // TEST DU PREMIER BYTE
  if (serial_in == 86 or serial_in == 118){ //V
   RAZ_vitesse();
   ;
   }
  else if (serial_in == 88 or serial_in == 120) RAZ(0); //X
  else if (serial_in == 89 or serial_in == 121) RAZ(1); //Y
  else if (serial_in == 90 or serial_in == 122) RAZ(2); //Z

  // VERIFIE SI SERIAL IN EST BIEN COMPOSE DE CHIFFRES
  if (mode == "mouvement"){
    if (serial_in <=57 && serial_in >=48){
      if (count == 1){Direction[motor] = Direction[motor] * 10 + (serial_in-48);}
      else if (count == 2){Nombre_de_pas[motor] = Nombre_de_pas[motor] * 10 + (serial_in-48);}
      }
    // CHANGE DE CIBLE A CHAQUE SPACE
    else if (serial_in == 32) count = count + 1;
    // PRINT RESULTAT APRES \n
    else if(count == 2 or serial_in == 10){
      for(motor=0; motor<=2; motor++){
        Serial.print("Direction[");Serial.print(motor); Serial.print("] = "); Serial.print(Direction[motor]);
        Serial.print("  -  Nombre_de_pas[");Serial.print(motor); Serial.print("] = "); Serial.println(Nombre_de_pas[motor]);
        }
    change = true;
    }
  }
  else if (mode == "vitesse"){
    if (serial_in <=57 && serial_in >=48){
      Vitesse = Vitesse * 10 + (serial_in - 48);
    }
    else if (serial_in == 10){
      Serial.print("Vitesse = "); Serial.println(Vitesse);
    }
      
  }
}

void RAZ(int i){
  mode = "mouvement";
  Serial.println("mode mouvement");
  motor = i;
  count = 0;
  Direction[motor] = 0;
  Nombre_de_pas[motor] = 0;
  //Vitesse[motor] = 0;
  Serial.print("RAZ_motor-");
  Serial.println(motor);
  }
  
void RAZ_vitesse(){
  mode = "vitesse";
  Serial.println("mode vitesse");
  Vitesse = 0;
  //Serial.println("RAZ-Vitesse");
  }

void move_XYZ(){
    STEP_counter();
    Serial.println("move_XYZ");
      // definir le sens de rotation
    digitalWrite(dirPinX, Direction[0]);
    digitalWrite(dirPinY, Direction[1]);
    digitalWrite(dirPinZ, Direction[2]);

    for (int i = 0; i <= Nombre_de_pas[3]; i++) {
      // These four lines result in 1 step:
      if (Nombre_de_pas[0] > i) digitalWrite(stepPinX, HIGH);
      if (Nombre_de_pas[1] > i) digitalWrite(stepPinY, HIGH);
      if (Nombre_de_pas[2] > i) digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(Vitesse);
      digitalWrite(stepPinX, LOW);
      digitalWrite(stepPinY, LOW);
      digitalWrite(stepPinZ, LOW);
      delayMicroseconds(400);
    //Serial.print("step-");
    //Serial.println(i);
    }
}


void STEP_counter(){
  Nombre_de_pas[3] = max(Nombre_de_pas[0], Nombre_de_pas[1]);
  Nombre_de_pas[3] = max(Nombre_de_pas[2], Nombre_de_pas[3]);
  Serial.print("STEP_counter-MAX=");
  Serial.println(Nombre_de_pas[3]);
  }
