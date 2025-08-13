// I2C comm:
//            -recieve 10 tare now
//            -recieve 20 aktual weight
//            -recieve 30-34 angle position
//            -recieve 40 hold position
//            -recieve 50 cleanse errors
//            -recieve zbytek error
#include "HX711.h"
#include "Wire.h"
#define SLAVE_ADD 0x08
// ------------- Definition of I/O pins ---------------
const int hall_senz = A0;             // z hall sondy
const int DIR = 3;                    // do driveru direction
const int PUL = 5;                    // do driveru kroky
//const int move45 = 4;                 // tlacitko povel otocenni
const int wake_up = 6;                // do driveru vzbuzeni
const int tenzometr_DT = 7;           // do ADC comm
const int tenzometr_SCK = 8;          // do ADC comm
//const int measure_start = 9;          // tlacitko povel mereni
//const int tare_btn = 10;              // tlacitko povel tare a stop otaceni na 10 s

long angle = 0;                       // aktualni poloha prepocet "poloha"
int desired_angle = 0;
//int slot[5] = {32, 122, 182, 243, 305}; // og uhly na BC
int slot[5] = {32, 144, 206, 264, 324}; // nove uhly pro vyoseny zasobnik vuci motoru

float base_A;                         // dobry by bylo zahodit floaty i double
float base_weight_A;                  // dobry by bylo zahodit floaty i double
int num_prijem = 0;                   // cislo prikazu
int slot_i = 0;                       // aktivni slot


struct output_data{
  uint16_t data_out = 0;              // weight
  uint8_t message = B00000000;
  // XXXX XXXX binary means XXX0 0000 = active slot, 
  //                        000X 0000 = tray active,
  //                        0000 X000 = scale tared,
  //                        0000 0XXX = error number, 000 = all OK, 001 = unknown command, 010 weight overload, 011 = des. pos. not found
  };
struct output_data output;        // od martina

struct inner_data{
  uint16_t weight = 0;
  uint16_t previous_weight = 0;
  uint8_t active_slot = 0;
  bool scale_tared = 0;
  bool tray_active = 0;
  bool pos_not_found = 0;
  bool unknown_command = 0;
  bool magasine_stuck = 0;
  bool sound_of_silence = 0;      // 0 = komunikace poslouchá; 1 = neposlouchá
  bool weight_overload = 0;
  };
struct inner_data inner;

HX711 scale;                          // z knihovky pro ADC

// ----------- FUNKCE --------------
void hold_Xsec(int m_seconds = 10000){  // holding the magasine for X sec, enabling manipulation with cells, has to be declared before main loop
  digitalWrite(wake_up, HIGH);           // cerveny HIGH
  delay(m_seconds);                     // nastavitelny cas
  digitalWrite(wake_up, LOW);          // cerveny LOW
}

void setup() {
  Serial.begin(57600);
  Wire.begin(SLAVE_ADD);
  Wire.onReceive(receive_data);
  Wire.onRequest(send_data);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  //pinMode(measure_start, INPUT_PULLUP);
  //pinMode(move45, INPUT_PULLUP);
  //pinMode(tare_btn, INPUT_PULLUP);          // predelat na 10sec delay
  pinMode(hall_senz, INPUT);
  pinMode(wake_up, OUTPUT);

  scale.begin(tenzometr_DT, tenzometr_SCK);   // innit dvojkanálového ADC
  scale.set_gain(128); //128                        // tare tenzometru pro zesileni 128
  base_A = abs(scale.read_average(10));
  Serial.print("scale base: \t\t");
  Serial.println(base_A);
  //measureWeights(); // fnc for calibration
  
  //desired_angle = 0;
  angle = (360.0 / 675.0) * analogRead(hall_senz);        // 675 je max hodnota vstupu 3,3v do adc prevodniku arduina
  //desired_angle = angle;
  move(slot[slot_i], 3840);
  Serial.println("slot 0 aktivni, program pripraven na funkci");
  Serial.print("adc hodnota hall senzoru: \t\t");
  Serial.println(analogRead(hall_senz));
  Serial.print("aktualni uhel: \t\t");
  Serial.println(angle);
  Serial.println("Version 4m 6d");
}

void receive_data(int numBytes) {         // detekce a vypsani prichozi komunikace; v numBytes je pocet bytu prichozi zpravy
  //if(inner.tray_active == 0){ //inner.sound_of_silence == 0
  //inner.sound_of_silence = 1;
  num_prijem = Wire.read();
  Serial.print("------------FLAG----------- Received data: ");
  Serial.println(num_prijem);
  //}
}

void send_data() {                        // odeslani vyzadanych dat
  const uint8_t out_arr_size = 3;
  uint8_t output_array[out_arr_size] = {0};
  // ------------nove---------------------------------------------
  output.data_out = inner.weight;
  output.message = 8*inner.scale_tared + 16*inner.tray_active; 
  if(inner.weight_overload>1){    // vypis errorovych hlasek
    output.message = output.message + 1;
  }
  else if (inner.unknown_command>1) {
    output.message = output.message + 2;
  }
  else if (inner.pos_not_found>1) {
    output.message = output.message + 1 + 2;
  }

  if(output.message >= 32){                         // kdyz je zasobnik v nejake pozici
    output.message = output.message && B00011111;   // pre-inserted active slot 0
  }
  switch (inner.active_slot) {
    case 0:
      output.message = output.message + 32; // slot 0 active
      break;
    case 1:
      output.message = output.message + 64;  // slot 1 active
      break;
    case 2:
      output.message = output.message + 32 + 64;  // slot 2 active
      break;
    case 3:
      output.message = output.message + 128;   // slot 3 active 
      break;
    case 4:
      output.message = output.message + 32 + 128;  // slot 4 active
      break;
    default:        
      // what R U doin'?                          // no slot active
      // nothin' just hanging around
      break;
  }
  //------------end nove------------------------------------------
  output_array[0] = lowByte(output.data_out);
  output_array[1] = highByte(output.data_out);
  output_array[2] = output.message;
  Wire.write(output_array, out_arr_size); 
  
  Serial.print("vaha sent highByte: \t");
  Serial.println(highByte(output.data_out));
  Serial.print("vaha sent lowByte: \t");
  Serial.println(lowByte(output.data_out));
  Serial.println("message: ");
  Serial.println(output.message,BIN);

  Serial.println("communication resolved");
}

int deal_w_comm(){
  int old_pos = inner.active_slot;
  switch(num_prijem){
      case 10:
        if(tare_bases() > 0){
          inner.scale_tared = 1;
        }
        break;
      case 20:
        output.data_out = measure();
        inner.weight = output.data_out;
        inner.tray_active = 0;
        break;
      case 30:
        if(move(slot[0], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 3840);                          // vraceni na puvodni pozici
          inner.active_slot = old_pos;
          //inner.tray_active = 0;
        } 
        else{
          inner.active_slot = 0;                              // informace o slotu 0
          //inner.tray_active = 0;
        }
        break;
      case 31:
        if(move(slot[1], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 3840);                          // vraceni na puvodni pozici
          inner.active_slot = old_pos;
          //inner.tray_active = 0;
        }
        else {
          inner.active_slot = 1;                              // informace o slotu 1
          //inner.tray_active = 0;
        }
        break;
      case 32:
        if(move(slot[2], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 3840);                          // vraceni na puvodni pozici
          inner.active_slot = old_pos;
          //inner.tray_active = 0;
        }
        else {
          inner.active_slot = 2;                              // informace o slotu 2
          //inner.tray_active = 0;
        }
        break;
      case 33:
        
        if(move(slot[3], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 3840);                          // vraceni na puvodni pozici
          inner.active_slot = old_pos;
          //inner.tray_active = 0;
        }
        else{
          inner.active_slot = 3;                              // informace o slotu 3
          //inner.tray_active = 0;
        }
        break;
      case 34:
        if(move(slot[4], 1920) < 0){  // 4!!!
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 3840);                          // vraceni na puvodni pozici
          inner.active_slot = old_pos;
          //inner.tray_active = 0;
        }
        else{
          inner.active_slot = 4;          // 4!!!                    // informace o slotu 3
          //inner.tray_active = 0;
        }
        break;
      case 40:
        hold_Xsec(10000);
        break;
      case 50:
        inner.scale_tared = 0;
        inner.weight_overload = 0;
        inner.unknown_command = 0;
        inner.pos_not_found = 0;
        break;
      default:
        inner.unknown_command = 1;                        // error, unknown command
        break;
  }
  inner.sound_of_silence = 0;
  num_prijem = 0;
  return 1;
}

int move(int chci_sem, int timeout){           // Imax = 400mA; timeout 120 kroku max -> 1/4 pomer = 480, 1/8 pomer = 960, 1/16 pomer = 1920, 1/32 pomer = 3840;
  inner.tray_active = 1;
  int old_slot = inner.active_slot;
  inner.active_slot = 5;          // po dobu pohybu neni aktivnni zadny slot
  int po_smeru = chci_sem - angle;
  if(po_smeru < 0){
    po_smeru = 360 + po_smeru;
  }
  int proti_smeru = 360 - po_smeru;
  int orig_angle = angle;

  if(proti_smeru > po_smeru){
    digitalWrite(DIR, HIGH);
  }
  else {
    digitalWrite(DIR, LOW);
  }
  digitalWrite(wake_up, HIGH);
  
  while(chci_sem != angle && abs(chci_sem - angle) > 1){                     // cisty pohyb dokud není podmínka uhlu splnena, odchylka < 0,5
    if(timeout > 0){
      timeout--;
      if((abs(chci_sem - angle) > 30) && (abs(orig_angle - angle) > 18)){
        digitalWrite(PUL, HIGH);
        delay(1.5);//delay(12);
        digitalWrite(PUL, LOW);
        delay(1.5);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
      else if((abs(chci_sem - angle) > 20) && (abs(orig_angle - angle) > 12)){ // bylo 15
        digitalWrite(PUL, HIGH);
        delay(5);//delay(12);
        digitalWrite(PUL, LOW);
        delay(5);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
      else if((abs(chci_sem - angle) > 10) && (abs(orig_angle - angle) > 6)){
        digitalWrite(PUL, HIGH);
        delay(8);//delay(12);
        digitalWrite(PUL, LOW);
        delay(8);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
      else{
        digitalWrite(PUL, HIGH);
        delay(10);//delay(12);
        digitalWrite(PUL, LOW);
        delay(10);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
    }
    else{
      angle = chci_sem;                                // ends while cycle
      //inner.pos_not_found = 1;                         // desired position not found
      inner.magasine_stuck = 1;
      inner.tray_active = 0;
      return -1;
    }
  }
  
  desired_angle = chci_sem;                            // reinicializace aktualni polohy
  Serial.print("úhel natoceni: \t\t");
  Serial.println(desired_angle);
  delay(500);                                          // drzeni aktualni pozice ať nepřekmitnu
  digitalWrite(wake_up, LOW);                          // driver jde do hajan
  inner.active_slot = old_slot;
  inner.tray_active = 0;    // 22.4.2025
  return 1;
}

double measure(){                                        // mereni vahy (weight_A)
  inner.tray_active = 1;
  double real_weight_A;
  scale.power_up();                                    // huh, you are finally awake
  scale.set_gain(128);//128                                 // nastaveni aktivni kanal A zesileni 128

  Serial.println("KANAL A");

  Serial.print("read average: \t\t");
  float tmp_vaha = abs(scale.read_average(5));
  Serial.println(tmp_vaha);  	                          // average raw hodnota
  Serial.print("read - base: \t\t");
  Serial.println(tmp_vaha - base_A);

  real_weight_A = (tmp_vaha - base_A - 550)/1499.04;    // should be 450           
  //real_weight_A = (tmp_vaha - base_A + 2513.842)/1433.964;       
  //real_weight_A = (tmp_vaha - base_A - 204.757)/1434.219;        //               funguje na starem zaspojeni zasobniku
          // 0,97 je systematická odchylka idk jestli provizorní konstrukcí nebo zlým měřítkem
  Serial.print("weight in grams: \t\t");
  Serial.println(real_weight_A);

  scale.power_down();                            // ADC do hajan
  //inner.tray_active = 0;

  if(inner.previous_weight/10 + real_weight_A > 2000){   // 2500 g je max hranice zatížení -> 2000 g je mnou nastavna hranice, 500 g je rezerva
    inner.weight_overload = 1;
  }

  return real_weight_A * 10;  // x10 pro desetinne misto
}

int tare_bases(){                 //  later erase, or built to operator panel
  inner.tray_active = 1;
  inner.previous_weight = inner.previous_weight + inner.weight;
  scale.power_up();                // wake the fuck up

  scale.set_gain(128);             // nastavenim zesileni 128 je nastaven kanal A 
  base_A = abs(scale.read_average(10));
  inner.tray_active = 0;

  scale.power_down();              // a do hajan
  return 1;
}

void measureWeights() {
  const int weights[] = {100, 300, 500, 700, 900, 1000};
  const int numWeights = sizeof(weights) / sizeof(weights[0]);
  float adcValues[numWeights];

  Serial.println("Scale measurement program starting...");

  for (int i = 0; i < numWeights; i++) {
    Serial.print("Place ");
    Serial.print(weights[i]);
    Serial.println(" g on the scale.");
    Serial.println("Press Enter when ready...");

    // Wait for Enter
    while (Serial.available()) Serial.read(); // clear buffer
    while (!Serial.available()) { }           // wait for input
    while (Serial.available()) Serial.read(); // clear after Enter

    float tmp_vaha = abs(scale.read_average(5)); // 5-sample average
    adcValues[i] = tmp_vaha;

    Serial.print("Measured ADC for ");
    Serial.print(weights[i]);
    Serial.print(" g: ");
    Serial.println(tmp_vaha, 2);
  }

  Serial.println("\nMeasurement results:");
  Serial.println("Weight(g)\tADC Value");
  for (int i = 0; i < numWeights; i++) {
    Serial.print(weights[i]);
    Serial.print("\t\t");
    Serial.println(adcValues[i], 2);
  }
}

void loop() {
  angle = (360.0 / 675.0) * analogRead(hall_senz);  // kontrola aktualniho uhlu
  //---------------- MANUAL MODE -----------------------------
  /*
  if(digitalRead(move45) == LOW){                   // pohyb na zadanou polohu (kdyz neni plna)
    inner.tray_active = 1;
    int old_pos = inner.active_slot;
    Serial.println("pohyb motoru o jednu pozici");
    if(slot_i >= 4){  // 4!!
      slot_i = 0;
    }
    else{
      slot_i = slot_i + 1;
    }
    if(move(slot[slot_i], 1500) < 0){               // fnc pohyb na zadaný úhel, timeout ma byt (200*16)/4 = 800 -> 1000 pouze pro ORIG uhly
      if(slot_i <= 0){  
        slot_i = 4;
      }
      else {
        slot_i = slot_i - 1;
      }
      Serial.print("nebyl dosazen zadany slot, aktivni slot: ");
      Serial.println(slot_i);
      inner.active_slot = old_pos;
      inner.tray_active = 0;
    }
    else {
      Serial.print("aktivni slot dosazen");
      Serial.println(slot_i);
      inner.active_slot = slot_i;
      inner.tray_active = 0;
    }
  }
  else if(digitalRead(measure_start) == LOW){            // mereni vahy zasobniku
    inner.tray_active = 1;
    Serial.println("detekce startu mereni");
    output.data_out = measure();
    inner.weight = output.data_out;
    inner.tray_active = 0;
  }
  else if(digitalRead(tare_btn) == LOW){                 // tare tenzometru (vynulovani)
    ///*
    Serial.println("measuring sequence, step is 10°");
    int angle_by_10 = 32;
    for (int i = 0; i<35; i++) {
      move(angle_by_10, 2000);
      measure();
      angle_by_10 = angle_by_10 + 10;
      if(angle_by_10>360){
        angle_by_10 = angle_by_10-360;
      }
    }
    // tady ma byt konec comm
    inner.tray_active = 1;
    Serial.println("Hold and Tare button pushed");
    hold_Xsec(10000);
    if(tare_bases()>0){
      inner.scale_tared = 1;
    }
    inner.tray_active = 0;
  }
  */
  if(num_prijem == 0 && (desired_angle != angle && abs(desired_angle - angle) > 14)){  // hold the line, pro odchylku nad 5 stupnu pohyb
    delay(3000);
    angle = (360.0 / 675.0) * analogRead(hall_senz);  // kontrola aktualniho uhlu
    Serial.print("pohnuto hrideli -> uprava; aktualni uhel: ");
    Serial.println(angle);
    move(desired_angle, 3840);                                   // melo by byl 1920
    delay(100);
  }
  
  if(num_prijem != 0){                        // deal with communication
    // inner.tray_active = 1; // odkomentovat? dopredu zaberu fungovani zasobniku
    if(deal_w_comm() < 0){
      Serial.println("Communication unresolved");
      //inner.sound_of_silence = 0;
    }
    Serial.println("Comm resloved successfully");
    //inner.sound_of_silence = 0;
  }

  delay(10);                                            // aby program neproběhl 666x za vteřinu
}