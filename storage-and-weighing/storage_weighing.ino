// I2C comm:
//            -recieve 10 tare now
//            -recieve 20 aktual weight
//            -recieve 30-33 angle position
//            -recieve 40 hold position
//            -recieve 50 cleanse errors
//            -recieve rest error
#include "HX711.h"
#include "Wire.h"
#define SLAVE_ADD 0x08
// ------------- Definition of I/O pins ---------------
const int hall_senz = A0;             // z hall sondy
const int DIR = 3;                    // do driveru direction
const int PUL = 5;                    // do driveru kroky
const int move45 = 4;                 // tlacitko povel otocenni
const int wake_up = 6;                // do driveru vzbuzeni
const int tenzometr_DT = 7;           // do ADC comm
const int tenzometr_SCK = 8;          // do ADC comm
const int measure_start = 9;          // tlacitko povel mereni
const int tare_btn = 10;              // tlacitko povel tare a stop otaceni na 10 s

long angle = 0;                       // aktualni poloha prepocet "poloha"
int desired_angle = 0;
//int slot[4] = {32, 121, 212, 300}; // orig uhly 34 124 214 305 uhly zmeneny kvuli pólům motoru a jejich magnetizaci ve stavu bez napětí
int slot[5] = {32, 122, 182, 242, 300}; // orig uhly 34 124 214 305 uhly zmeneny kvuli pólům motoru a jejich magnetizaci ve stavu bez napětí
//bool full_slot[4] = {0, 0, 0, 0};
float base_A;                         // weight of base -> weight on motor and storage torso, to be substracted from measured amount
float base_B;                         
//float base_weight_A;                  
//float base_weight_B;  
int num_prijem = 0;                   // cislo prikazu
int slot_i = 0;                       // aktivni slot
float previous_weight = 0;

struct output_data{
  uint16_t data_out = 0;              // weight
  uint8_t message = B00000000;
  // XXXX XXXX binary means XX00 0000 = active slot, 
  //                        00X0 0000 = scale tared, 
  //                        000X 0000 = manual mode on, 
  //                        0000 XXXX = error number, 1000 = Desired position not found, 0100 = unknown command, 0010 = magasine stuck, 0001 weight overload
  // NEW
  // XXXX XXXX binary means XXX0 0000 = active slot, 
  //                        000X 0000 = tray active,
  //                        0000 X000 = scale tared,
  //                        0000 0XXX = error number, 000 = all OK, 001 = unknown command, 010 weight overload, 011 = des. pos. not found, 100 = maasine stuck
  };
struct output_data output;        // od martina

struct inner_data{
  uint16_t weight = 0;
  uint8_t active_slot = 0;
  bool scale_tared = 0;
  bool manual_mode = 0;
  bool tray_active = 0;
  bool pos_not_found = 0;
  bool unknown_command = 0;
  bool magasine_stuck = 0;
  bool sound_of_silence = 0;      // 0 = komunikace poslouchá; 1 = neposlouchá; NEAKTIVNI
  bool weight_overload = 0;
  uint8_t time = 0;
  };
struct inner_data inner;

HX711 scale;                          // z knihovky pro ADC

void hold_Xsec(int m_seconds = 10000){  // holding the magasine for X sec, enabling manipulation with cells, has to be declared before main loop
  digitalWrite(wake_up, HIGH);
  delay(m_seconds);                     // nastavitelny cas
  digitalWrite(wake_up, LOW);
}

void setup() {
  Serial.begin(57600);
  Wire.begin(SLAVE_ADD);
  Wire.onReceive(receive_data);
  Wire.onRequest(send_data);

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(measure_start, INPUT_PULLUP);
  pinMode(move45, INPUT_PULLUP);
  pinMode(tare_btn, INPUT_PULLUP);
  pinMode(hall_senz, INPUT);
  pinMode(wake_up, OUTPUT);

  scale.begin(tenzometr_DT, tenzometr_SCK);   // innit dvojkanálového ADC
  scale.set_gain(128); //128                        // tare tenzometru pro zesileni 128
  base_A = scale.read_average(5);

  scale.set_gain(32);                         // tare tenzometru pro zesileni 32
  base_B = scale.read_average(5);
  
  angle = (360.0 / 675.0) * analogRead(hall_senz);        // 675 je max hodnota vstupu 3,3v do adc prevodniku arduina
  //desired_angle = angle;
  move(slot[slot_i], 3840);
  Serial.println("slot 0 aktivni, program pripraven na funkci");
  Serial.print("adc hodnota hall senzoru: \t\t");
  Serial.println(analogRead(hall_senz));
  Serial.println("Version 2m 13d");
}

void loop() {
  angle = (360.0 / 675.0) * analogRead(hall_senz);  // kontrola aktualniho uhlu
  //---------------- MANUAL MODE -----------------------------
  if(digitalRead(move45) == LOW){                   // pohyb na zadanou polohu (kdyz neni plna)
    inner.tray_active = 1;
    Serial.println("pohyb motoru o jednu pozici");
    if(slot_i >= 4){
      slot_i = 0;
    }
    else{
      slot_i = slot_i + 1;
    }
    if(move(slot[slot_i], 2500) < 0){               // fnc pohyb na zadaný úhel, timeout ma byt 1920
      if(slot_i <= 0){
        slot_i = 4;
      }
      else {
        slot_i = slot_i - 1;
      }
      Serial.print("nebyl dosazen zadany slot, aktivni slot: ");
      Serial.println(slot_i);
      inner.tray_active = 0;
    }
    else {
      Serial.print("aktivni slot");
      Serial.println(slot_i);
      inner.tray_active = 0;
    }
  }
  else if(digitalRead(measure_start) == LOW){            // mereni vahy zasobniku
    inner.tray_active = 1;
    Serial.println("detekce startu mereni");
    measure();
    inner.tray_active = 0;
  }
  else if(digitalRead(tare_btn) == LOW){                 // tare tenzometru (vynulovani)
    inner.tray_active = 1;
    Serial.println("Hold and Tare button pushed");
    hold_Xsec(10000);
    tare_bases();
    inner.tray_active = 0;
  }
  if(num_prijem == 0 && (desired_angle != angle && abs(desired_angle - angle) > 5)){  // hold storage torso from moving
  //if (inner.time > 20) {                          // jednou za dve vteriny projde if
    delay(3000);                                    // delayed action for event to pass
    angle = (360.0 / 675.0) * analogRead(hall_senz);  // kontrola aktualniho uhlu
    //if(num_prijem == 0 && (desired_angle != angle && abs(desired_angle - angle) > 5)){  // hold the line, pro odchylku > 2 se pohni
    Serial.print("pohnuto hrideli -> uprava; aktualni uhel: ");
    Serial.println(angle);
    move(desired_angle, 3840);                                   // melo by byl 1920
    //inner.time = 0;
    //}
  }

  if(num_prijem != 0){                        // deal with communication
    if(deal_w_comm() < 0){
      Serial.println("Communication unresolved");
      inner.sound_of_silence = 0;
    }
    Serial.println("Comm resloved successfully");
    inner.sound_of_silence = 0;
  }

  delay(100);                                            // aby program neproběhl 666x za vteřinu
  //++inner.time;
}

void receive_data(int numBytes) {         // detekce a vypsani prichozi komunikace
  if(inner.tray_active == 0){ //inner.sound_of_silence == 0
    inner.sound_of_silence = 1;
    num_prijem = Wire.read();
    Serial.print("Received data: ");
    Serial.println(num_prijem);
  }
}
void send_data() {                        // odeslani vyzadanych dat
  const uint8_t out_arr_size = 3;
  uint8_t output_array[out_arr_size] = {0};
  // ------------nove---------------------------------------------
  output.data_out = inner.weight;
  output.message = 8*inner.scale_tared + 16*inner.tray_active; 
  if(inner.weight_overload>1){
    output.message = output.message + 1;
  }
  else if (inner.unknown_command>1) {
    output.message = output.message + 2;
  }
  else if (inner.pos_not_found>1) {
    output.message = output.message + 1 + 2;
  }

  
  if(output.message > 32){
    output.message = output.message && B00111111;   // pre-inserted active slot 0
  }
  switch (inner.active_slot) {
    case 0:
      output.message = output.message + 32; // slot 0 active
      // what R U doin'?
      // nothin' just hanging around
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

  Serial.println("\n vaha sent highByte: ");
  Serial.println(highByte(output.data_out));
  Serial.println("vaha sent lowByte: ");
  Serial.println(lowByte(output.data_out));
  Serial.println("message: ");
  Serial.println(output.message,BIN);

  Serial.println("communication resolved");
}
int deal_w_comm(){
  int old_pos = inner.active_slot;
  switch(num_prijem){
      case 10:
        tare_bases();
        inner.scale_tared = 1;                            // scale ready report
        break;
      case 20:
        output.data_out = measure();
        inner.weight = output.data_out;
        break;
      case 30:
        if(move(slot[0], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 1920);                          // vraceni na puvodni pozici
        } 
        else{
          inner.active_slot = 0;                              // informace o slotu 0
        }
        break;
      case 31:
        if(move(slot[1], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 1920);                          // vraceni na puvodni pozici
        }
        else {
          inner.active_slot = 1;                              // informace o slotu 1
        }
        break;
      case 32:
        if(move(slot[2], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 1920);                          // vraceni na puvodni pozici
        }
        else {
          inner.active_slot = 2;                              // informace o slotu 2
        }
        break;
      case 33:
        if(move(slot[3], 1920) < 0){
          inner.pos_not_found = 1;                            // error, nebyl dosazen cilovy slot
          move(slot[old_pos], 1920);                          // vraceni na puvodni pozici
        }
        else{
          inner.active_slot = 3;                              // informace o slotu 3
        }
        break;
      case 40:
        hold_Xsec(10000);
        break;
      case 50:
        inner.magasine_stuck = 0;
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
  //timeout = 100;            // overwrite zadaneho timeoutu
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
  
  digitalWrite(wake_up, HIGH);                  // vzbuzeni driveru

  //  if(num_prijem == 0 && (desired_angle != angle && abs(desired_angle - angle) > 2)){  // hold the line, pro odchylku > 2 se pohni
  int pocet = 10;
  int previous_angles[pocet];
  for (int i = 0; i<pocet; ++i) {     // naplneni pole
    previous_angles[i] = 10000;       // 10k je jen hodnota která dostatecne ovlivní prumer aby neskocil error hned
  }
  int index = 0;
  int sum;

  while(chci_sem != angle && abs(chci_sem - angle) > 1){                     // cisty pohyb dokud není podmínka uhlu splnena, odchylka < 0,5
    previous_angles[index] = angle;
    index = (index + 1) % pocet;              // z rostoucího indexu pocitam zbytek po deleni tzn 0-9 hodnotu
    for (int i = 0; i<pocet; ++i) {
      sum = sum + previous_angles[i];         // vypocet aktualniho prumeru
    }
    int avg_10 = (double)sum/10;

    if(timeout > 0 || abs(avg_10 - angle) > 20){   // jestli avg_10 funguje timeout je redundantní
      timeout--;
      if((abs(chci_sem - angle) > 20) && (abs(orig_angle - angle) > 5)){
        digitalWrite(PUL, HIGH);
        delay(1);//delay(12);
        digitalWrite(PUL, LOW);
        delay(1);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
      else{
        digitalWrite(PUL, HIGH);
        delay(4);//delay(12);
        digitalWrite(PUL, LOW);
        delay(4);//delay(12);
        angle = (360.0 / 675.0) * analogRead(hall_senz);   // vypocet aktualniho uhlu od 0 do 360;  675 je maximalni hodnota ADC prevodniku arduina pro 3,3V ((3,3/5)*1024bit)
      }
    }
    else{
      angle = chci_sem;                                // ends while cycle
      //inner.pos_not_found = 1;                         // desired position not found
      inner.magasine_stuck = 1;
      return -1;
    }
  }
  desired_angle = chci_sem;                            // reinicializace aktualni polohy
  Serial.print("úhel natoceni: \t\t");
  Serial.println(desired_angle);
  hold_Xsec(500);                                      // previously delay
  digitalWrite(wake_up, LOW);                          // driver jde do hajan
  return 1;
}
double measure(){                                        // mereni vahy (weight_A a weight_B)
  double real_weight_A;
  double real_weight_B;
  //double max_val = 8377608;                        //16777216 , 8377608
  scale.power_up();                                    // huh, you are finally awake
  scale.set_gain(128);//128                                 // nastaveni aktivni kanal A zesileni 128

  Serial.println("KANAL A");

  Serial.print("read average: \t\t");
  float tmp_vaha = scale.read_average(5);
  Serial.println(tmp_vaha);  	                          // average raw hodnota
  Serial.print("read - base: \t\t");
  Serial.println(tmp_vaha - base_A);

  //real_weight_A = (tmp_vaha - base_A)/1605,51;                // 401,3775 je narust hodnoty ADC pri zmene o 0,5g OLD
  real_weight_A = (tmp_vaha - base_A - 1,78)/1605,99602;        // vchazi z lin. rov. y=823,71x+1312,5                  NETESTOVANO
  Serial.print("weight in grams: \t\t");
  Serial.println(real_weight_A);

  //    -----------  ted mereni kanalu B   --------------------------

  Serial.println("KANAL B");
  scale.set_gain(32);                          // nastaveni zesileni na 32 tzn cteni kanalu B

  Serial.print("read average: \t\t");
  tmp_vaha = scale.read_average(5);            // avg hodnota z peti zmerenych
  Serial.println(tmp_vaha);
  Serial.print("read - base: \t\t");
  Serial.println(tmp_vaha - base_B);

  //real_weight_B = (tmp_vaha - base_B)/768,5;                  // 192,125 je narust hodnoty ADC pri zmene o 0,5g   OLD
  real_weight_B = (tmp_vaha - base_B + 1,39)/768,00273;        // vychzi z lin. rov. y=383,23x+1155,5                NETESTOVANO
  Serial.print("weight in grams: \t\t");
  Serial.println(real_weight_B);

  scale.power_down();                            // ADC do hajan

  Serial.print("final weight: \t\t");
  Serial.println(real_weight_B + real_weight_A); //real_weight_B - base_weight_B + real_weight_A - base_weight_A

  if(real_weight_A + real_weight_B + previous_weight > 1200){         // váha báze je 700g -> 1200g mohu měřit a zbyde mi 100g jako rezerva overloadu
    inner.weight_overload = 1;
  }
  return real_weight_B + real_weight_A;
}
void tare_bases(){                 // nullify scale for next reading
  scale.power_up();                // wake the fuck up

  previous_weight = previous_weight + measure();

  scale.set_gain(128);             // nastavenim zesileni 128 je nastaven kanal A je to zaroven TARE!
  base_A = scale.read_average(5);
  scale.set_gain(32);              // nastavenim zesileni 32 je nastaven kanal A je to zaroven TARE!
  base_B = scale.read_average(5);
  scale.power_down();              // a do hajan
}