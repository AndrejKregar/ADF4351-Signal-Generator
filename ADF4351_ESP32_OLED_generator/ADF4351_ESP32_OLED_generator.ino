/* 
 * ESP32   Dev Module
 * ADF je na HSPI
 * VSPI je prost za TFT zaslon
 *
 *  pin  ESP32     ADF
 *   14  CLK    -> CLK
 *   13  MOSI   -> DATA
 *   27  CS, CE -> LE
 *   25  locked -> MUX
 *   26  OUT    -> LD
 *       EN     -> CE
 *
 *
 *   VSPI     5 SS, CS, CE     - SW default 
 *           18 SCK            - HW default
 *           19 MISO, SDO      - HW default
 *           23 MOSI, SDI, SDA - HW default
 *           nn A0, DC, CD, RS - SW
 *              RESET          - SW ali RESET
 *
 *    ADF    27 SS, CS, CE     - SW 
 *           14 SCK            - HW default
 *           12 MISO, SDO      - HW default - ni v uporabi
 *           13 MOSI, SDI, SDA - HW default
 *
 *           andrejkregar53@gmail.com          
*/


// za vpis zadnje frekvence in moči
#include <EEPROM.h>

// OLED zadeve    !!!!!!  ne pozabi PULL UP uporov 3,9 kOhm na SDA in SCL !!!!!!
// https://github.com/olikraus/u8g2/wiki/u8x8setupc

#include <U8x8lib.h>
#include <Wire.h>                                                    // I2C knjižnica
//U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);                // u8x8 ime zaslona MALI BELI
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);               // u8x8 ime zaslona SSD1309 VELIKI BELI ali MALI RUMENO-MODRI
//U8X8_SSD1327_EA_W128128_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);      // u8x8 ime zaslona SSD1327 VELIKI RUMENI

// SPI zadeve
#include <SPI.h>
SPIClass ADF(HSPI);                           // naredimo še en SPI Class in mu damo ime ADF. Fizično je vezan na drugi SPI vmesnik

// ADF pini
#define ADF_LE_Pin  27                        // SPI signal CE, CS, SS
#define ADF_MUX_pin 25                        // digital Lock - locked
#define ADF_LD_pin  26                        // izhod - zelena LED - OUT

// tipke                                      // ti pini zahtevajo HW Pull-Up upore nekaj kOhm! Tipke vežemo na maso.
#define pinLEVO  39                           // siva
#define pinDOL   36                           // bela
#define pinGOR   34                           // vijolična
#define pinDESNO 35                           // modra

// za CASE
#define DESNO 10
#define GOR   11
#define DOL   12
#define LEVO  13
#define NIC   14

byte tipke = 0;
byte kurzor  = 5;                             // kurzor od 0 do 15, 5 je začetni položaj
byte vejica  = 8;
byte vrstica = 0;                             // začetni položaj              - nekje je še nekaj narobe. 
byte vrstica_F = 0;                           // vrstica za frekvenco         - vrstica in vrstica_F morata obe imeti isto vrednost
byte vrstica_M = 3;                           // vrstica za moč
byte vrstica_S = 6;                           // vrstica za stanje

uint32_t zdaj, prej;                          // za utripanje kurzorja

uint64_t frekvenca =  123456789;              // nekaj čudnega: pri uint32_t nekatere frekvence ne delajo
uint64_t frekvenca_nova;
uint32_t frekvenca_MHz;
uint32_t frekvenca_dec;

uint32_t frekvenca_min = 34376000;            // najnižja in  najvišja frekvenca
uint32_t frekvenca_max = 3600000000;          // 3,6 GHz, odvisno od prescalerja
uint8_t  RF_out_pwr_nova;
uint8_t  naslov;

#define REF_CLK 25000000                      // izmerjena interna referenca 24.999.864 Hz

#define SHL(x,y) ((uint32_t)1<<y)*x           // za vpis v registre

// REGISTRI
// Register 0:
uint16_t INT = 0;
uint16_t FRAC = 0;

// Register 1:
uint8_t  phase_adj = 0;                       // 0 OFF   1 ON        0 - dela???                                        -   RF Settings
uint8_t  prescaler = 0;                       // 0 4/5   1 8/9       0 - 3,6 GHz  1 -  4,4 GHz                          -   RF Settings
uint16_t phase_val = 1;                       // biti mora manj kot MOD !!!!                                            -   RF Settings
uint16_t MOD = 4095;                          // interpolator modulus - določa resolucijo, [max 4095]                   -   RF Settings
                                              // MOD = pfd_freq / resolucija - za 100 Hz max, -> glej enačba 4

// Register 2:
uint8_t  Low_Noise_Spur = 0b00;               // 0b00 Low Loise,  0b11 Low Spur
uint8_t  LDP = 0;                             // 0 10nS,     1 6nS
uint8_t  Mux_out = 0b110;                     // 0b110 digital lock, 0b101 analog lock  [b28, b27, b2]
uint8_t  PD_polarity = 1;                     // 0 negative, 1 positive
uint8_t  Dbl_buf = 1;                         // 0 Disabled, 1 Enabled
uint8_t  Power_down = 0;                      // 0 Disabled, 1 Enabled
uint8_t  Charge_pump_current = 0b0111;        // 2,50ma      prej 0b1100 4,06 mA
uint8_t  CP_3_state = 0;                      // 0 Disabled, 1 Enabled
uint8_t  LDF = 0;                             // 0 FRAC_N,   1 INT_N         1 včasih utripa ????
uint8_t  Counter_reset = 0;                   // 0 Disabled, 1 Enabled       1 ne dela
uint16_t Ref_counter = 10;                    // od 1 do 1023  ???           10                                         -   RF Settings
uint8_t  Ref_doubler = 0;                     // 0 Disabled, 1 Enabled       če je ref frekvenca 25 MHz mora biti 0     -   RF Settings
uint8_t  Ref_div2 = 0;                        // 0 Disabled, 1 Enabled       bolje 1                                    -   RF Settings

// Register 3:
uint8_t  Band_Select_CLK_Mode = 1;            // 0 Low, 1 High    0 dela, bolje je 1, ampak mora biti Enable FRAC/MOD GCD ????????
uint8_t  ABP = 0;                             // 0 6nS, 1 3 nS
uint8_t  Charge_cancel = 0;                   // 0 Disabled, 1 Enabled
uint8_t  CSR = 0;                             // 0 Disabled, 1 Enabled
uint16_t CLK_div_value = 150;                 // od 1 do 4095  ??? timeout   150 po SW
uint8_t  CLK_div_mode = 0b01;                 // 0b00 CLK_div_mode OFF, 01 fast lock enable 10 resync enable ->  pin SW poveže na AGND

// Register 4:
uint8_t  Feedback_sig = 1;                    // 0 divided, 1 fundamental                                               -   RF Settings
uint8_t  Output_div   = 2;                    // kasneje se spreminja v odvisnosti od izbrane frekvence                 -   RF Settings
uint8_t  VCO_pwrdown  = 0;                    // 0 VCO powered UP, 1 powered DOWN
uint8_t  MTLD = 1;                            // mute 0 Disabled, 1 Enabled
uint8_t  AUX_out_sel  = 0;                    // 0 divided, 1 fundamental
uint8_t  AUX_out_ena  = 0;                    // 0 Disabled, 1 Enabled
uint8_t  AUX_pwr = 0;                         // 0 -4dBm, 1 -1dBm, 2 +2dBm, 3 +5dBm
uint8_t  RF_enab = 1;                         // 0 - output disabled
uint8_t  RF_out_pwr = 2;                      // 0 -4dBm, 1 -1dBm, 2 +2dBm, 3 +5dBm
                                              // 0,4 mW   0,8 mW   1,6 mW   3,16 mW
uint8_t  Band_Select_CLK = 100;               // 0b00..01 1, 0b00..10 2, 0b11..00 252, 0b11..11 255

// Register 5:
uint8_t  LD_pinmode = 1;                      // 1 digital lock detect

// Register 5:
uint32_t reg[6] = {0, 0, 0, 0, 0, 0};

// enačba 4
uint32_t pfd_freq = (REF_CLK * (1 + Ref_doubler)) / (Ref_counter * ((1 + Ref_div2)));        


///////////// setup /////////////
void setup()
{
//  Serial.begin(115200);

  pinMode (ADF_MUX_pin, INPUT);
  digitalWrite(ADF_MUX_pin, LOW);
  pinMode (ADF_LD_pin, INPUT);
  digitalWrite(ADF_LD_pin, LOW);  

  pinMode(pinDOL,   INPUT);
  pinMode(pinLEVO,  INPUT);
  pinMode(pinGOR,   INPUT);
  pinMode(pinDESNO, INPUT);  

  // za OLED
  Wire.setClock(3400000);                     // najvišja še zanesljiva hitrost v Hz
  u8x8.begin();                               // (SDA, SCL)  !!!!!!  ne pozabi PULL UP uporov 3,9 kOhm  !!!!!!
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_7x14_1x2_f);         // Izberemo font
  napisiOsnovo();
  u8x8.setFont(u8x8_font_7x14B_1x2_f);        // bold

  // SPI_2
  pinMode (ADF_LE_Pin, OUTPUT);
  ADF.begin(14, 12, 13, 27);                  // CLK, MISO, MOSI, SS     privzete vrednosti, razen SS!
  delayMicroseconds(10);                      // dodano 11.4                    Kako je že rekla nuna?
  ADF.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));         // dodano 19.4     

  EEPROM.begin(8);                            // najmanj ta velikost, max je 512 bytov, od 0 do 511
  naslov = 0;                                 // sem smo spravili krekvenco
  frekvenca = EEPROM.readULong(naslov);       // preberemo zadnjo frekvenco
  naslov += sizeof(unsigned long);
  RF_out_pwr = EEPROM.readUInt(naslov);       // preberemo zadnjo moč
  frekvenca_nova = frekvenca;                 // ??? če takoj berem frekvenca_nova ob vklopu nekaj trzne
  RF_out_pwr_nova = RF_out_pwr;
  
  delay(50);
  
  pripravi_registre();                        // vpišemo prebrano stanje
  posodobi_vse_registre();
  delay(50);
}

///////////// loop /////////////
void loop()
{
  tipke = beri_tipke();
  racunaj_tipke();
  napisiRezultate();

  if ((frekvenca_nova != frekvenca) || (RF_out_pwr_nova != RF_out_pwr))       // v registre ADF in EEPROM vpisujemo samo spremembe
  {
    frekvenca = frekvenca_nova;
    RF_out_pwr = RF_out_pwr_nova;

    naslov = 0;                               // pisanje v EEPROM
    EEPROM.writeULong(naslov, frekvenca);
    naslov += sizeof(unsigned long);
    EEPROM.writeUInt(naslov, RF_out_pwr);
    EEPROM.commit();

    pripravi_registre();
    posodobi_vse_registre();
  }
}


///////////// podprogrami /////////////
void pripravi_registre()
{
  if (frekvenca >= 2200000000) Output_div = 0;
  if (frekvenca < 2200000000) Output_div = 1;
  if (frekvenca < 1100000000) Output_div = 2;
  if (frekvenca < 550000000) Output_div = 3;
  if (frekvenca < 275000000) Output_div = 4;
  if (frekvenca < 137500000) Output_div = 5;
  if (frekvenca < 68750000) Output_div = 6;

// osnovni enačbi
  INT = (frekvenca * (1 << Output_div)) / pfd_freq;
  FRAC = (((frekvenca * (1 << Output_div)) % pfd_freq) * 4095) / pfd_freq;

  reg[0] = SHL(INT, 15) | SHL(FRAC, 3) | 0b000;
  reg[1] = SHL(phase_adj, 28)  | SHL(prescaler, 27)  | SHL(phase_val, 15)  | SHL(MOD, 3) | 0b001;
  reg[2] = SHL(Low_Noise_Spur, 29) | SHL(Mux_out, 26) | SHL(Ref_doubler, 25) | SHL(Ref_div2, 24) 
         | SHL(Ref_counter, 14) | SHL(Dbl_buf, 13) | SHL(Charge_pump_current, 9) | SHL(LDF, 8) 
         | SHL(LDP, 7) | SHL(PD_polarity, 6) | SHL(Power_down, 5) | SHL(CP_3_state, 4) 
         | SHL(Counter_reset, 3) | 0b010;
  reg[3] = SHL(Band_Select_CLK_Mode, 23) | SHL(ABP, 22) | SHL(Charge_cancel, 21) | SHL(CSR, 18) 
         | SHL(CLK_div_mode, 15) | SHL(CLK_div_value, 3) | 0b011;
  reg[4] = SHL(Feedback_sig, 23) | SHL(Output_div, 20) | SHL(Band_Select_CLK, 12) | SHL(VCO_pwrdown, 9)
         | SHL(MTLD, 10) | SHL(AUX_out_sel, 9) | SHL(AUX_out_ena, 8) | SHL(AUX_pwr, 6) 
         | SHL(RF_enab, 5) | SHL(RF_out_pwr, 3) | 0b100;
  reg[5] = SHL(LD_pinmode, 22) | SHL(0b11, 19) | 0b101;
}

void vpisi_ADF_registre(uint16_t reg_id)
{
  digitalWrite(ADF_LE_Pin, LOW);
  delayMicroseconds(10);
  ADF.transfer((uint8_t)(reg[reg_id] >> 24));
  ADF.transfer((uint8_t)(reg[reg_id] >> 16));
  ADF.transfer((uint8_t)(reg[reg_id] >> 8));
  ADF.transfer((uint8_t)(reg[reg_id]));
  delayMicroseconds(10);
  digitalWrite(ADF_LE_Pin, HIGH);
  delayMicroseconds(2500);
}

void posodobi_vse_registre()
{
  for (int i = 5; i >= 0; i--)
  {
    vpisi_ADF_registre(i);
  }
}

void napisiOsnovo()
{
  u8x8.setCursor(0, vrstica_F);               // stolpec, vrstica
  u8x8.print("Fre:");
  u8x8.drawString(13, vrstica_F, "MHz");
  u8x8.setCursor(0, vrstica_M);               // stolpec, vrstica
  u8x8.print("Moc:");
  u8x8.drawString(7, vrstica_M, "dBm");
  u8x8.drawString(14, vrstica_M, "mW");  
  u8x8.setCursor(0, vrstica_S);               // stolpec, vrstica
  u8x8.print("LCK:");
  u8x8.setCursor(8, vrstica_S);               // stolpec, vrstica
  u8x8.print("OUT:");
}

void napisiRezultate()                        // na OLED   !!!!! POENOSTAVI !!!!!
{
  frekvenca_MHz = frekvenca / 1000000;
  u8x8.setCursor(4, vrstica_F);               // stolpec, vrstica
  if (frekvenca_MHz < 1000) u8x8.print(" ");
  if (frekvenca_MHz < 100)  u8x8.print(" ");
  if (frekvenca_MHz < 10)   u8x8.print(" ");
  u8x8.print(frekvenca_MHz);
//  Serial.print("frekvenca_MHz ") ; Serial.println(frekvenca_MHz);     
  u8x8.setCursor(vejica, vrstica_F);
  u8x8.print(",");
  
  frekvenca_dec = (frekvenca - (frekvenca_MHz * 1000000)) / 100;   // deljeno s 100, če želimo 100 Hz resolucijo
  
  if (frekvenca_dec < 1000) u8x8.print("0");
  if (frekvenca_dec < 100)  u8x8.print("0");
  if (frekvenca_dec < 10)   u8x8.print("0");                                                                   
  u8x8.print(frekvenca_dec);
//  Serial.print("frekvenca_dec ") ; Serial.println(frekvenca_dec);
  
  zdaj = millis();
  if (zdaj - prej >= 800)                     // utrip
  {
    u8x8.setCursor(kurzor, vrstica);
    u8x8.print("_");
    prej = zdaj;
  }
  delay(200);
  
  if (RF_out_pwr == 0)   {u8x8.drawString(5, vrstica_M, "-4"); u8x8.drawString(11, vrstica_M, "0,4");}
  if (RF_out_pwr == 1)   {u8x8.drawString(5, vrstica_M, "-1"); u8x8.drawString(11, vrstica_M, "0,8");}
  if (RF_out_pwr == 2)   {u8x8.drawString(5, vrstica_M, "+2"); u8x8.drawString(11, vrstica_M, "1,6");}
  if (RF_out_pwr == 3)   {u8x8.drawString(5, vrstica_M, "+5"); u8x8.drawString(11, vrstica_M, "3,2");}

  u8x8.setFont(u8x8_font_open_iconic_check_2x2);            //   u8x8.setFont(u8x8_font_open_iconic_check_2x2);
  u8x8.setCursor(5, vrstica_S);
  if  ((digitalRead(ADF_MUX_pin) == 1))  u8x8.write(67);    // 64, 65
  else u8x8.write(68);                                      // 66

  u8x8.setCursor(13, vrstica_S);
  if  ((digitalRead(ADF_LD_pin) == 1))    u8x8.write(67);
  else u8x8.write(68);                                      // 66
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
}

int beri_tipke()
{
  if (digitalRead(pinGOR)   == LOW) return GOR;
  if (digitalRead(pinDOL)   == LOW) return DOL;
  if (digitalRead(pinLEVO)  == LOW) return LEVO;
  if (digitalRead(pinDESNO) == LOW) return DESNO;
  return NIC;                                               // noben gumb ni pritisnjen
}

void racunaj_tipke()
{
  switch (tipke)
  {
    case 10:                                                // desno
//      Serial.println("desno");
      kurzor ++;              // kurzor na desno
      
      if (kurzor == 3) kurzor = 4;                          // da ne more levo - itak ne more, ker tiščimo desno tipko !!!

      if ((vrstica == vrstica_F) && (kurzor == vejica))     // da preskoči vejico
      {
        kurzor ++;
        vrstica = vrstica_F;    
      }

      if ((vrstica == vrstica_F) && (kurzor == 13))         // skoči na moč
      {
        kurzor = 6;
        vrstica = vrstica_M;
      }

      if ((vrstica == vrstica_M) && ((kurzor < 6) || (kurzor > 6)))
      {
        kurzor = 5;                                         // skoči nazaj na frekvenco
        vrstica = vrstica_F;
      }
//      Serial.print("kurzor ") ; Serial.println(kurzor);     
     break;

    case 11:                                                // gor
      if (vrstica == vrstica_F)
      {
        if (kurzor == 4)  frekvenca_nova = frekvenca + 1000000000 ;     // 1 GHz
        if (kurzor == 5)  frekvenca_nova = frekvenca + 100000000 ;
        if (kurzor == 6)  frekvenca_nova = frekvenca + 10000000 ;
        if (kurzor == 7)  frekvenca_nova = frekvenca + 1000000 ;
        if (kurzor == 9)  frekvenca_nova = frekvenca + 100000 ;         // 8 je vejica
        if (kurzor == 10) frekvenca_nova = frekvenca + 10000 ; 
        if (kurzor == 11) frekvenca_nova = frekvenca + 1000 ;
        if (kurzor == 12) frekvenca_nova = frekvenca + 100 ;            // 100 Hz
      }

      if(frekvenca_nova > frekvenca_max) frekvenca_nova = frekvenca;
      if(frekvenca_nova < frekvenca_min) frekvenca_nova = frekvenca;     

      if ((vrstica == vrstica_M) && (kurzor == 6))
      {
        RF_out_pwr_nova = (RF_out_pwr + 1) % 4;
      }
      break;
      
    case 12:      // dol
      if (vrstica == vrstica_F)
      {
        if (kurzor == 4)  frekvenca_nova = frekvenca - 1000000000 ;     // 1 GHz
        if (kurzor == 5)  frekvenca_nova = frekvenca - 100000000 ;
        if (kurzor == 6)  frekvenca_nova = frekvenca - 10000000 ;
        if (kurzor == 7)  frekvenca_nova = frekvenca - 1000000 ;
        if (kurzor == 9)  frekvenca_nova = frekvenca - 100000 ;         // 8 je vejica
        if (kurzor == 10) frekvenca_nova = frekvenca - 10000 ;
        if (kurzor == 11) frekvenca_nova = frekvenca - 1000 ;  
        if (kurzor == 12) frekvenca_nova = frekvenca - 100 ;            // 100 Hz
      }

      if(frekvenca_nova < frekvenca_min) frekvenca_nova = frekvenca;
      if(frekvenca_nova > frekvenca_max) frekvenca_nova = frekvenca;
            
      if ((vrstica == vrstica_M) && (kurzor == 6))
      {
        RF_out_pwr_nova = (RF_out_pwr - 1) % 4;                         // se zatakne, če greš navzdol, ampak BV
      }
      break;

    case 13:      // levo
//      Serial.println("levo");
      kurzor --;

      if ((kurzor == 3) && (vrstica == vrstica_F))                      // da ne more levo in skoči na moč
      {
        kurzor = 6;
        vrstica = vrstica_M;
      }

      if ((vrstica == vrstica_F) && (kurzor == vejica))                 // da preskočimo vejico
      {
        kurzor --;
        vrstica = vrstica_F;
      }
  
      if ((vrstica == vrstica_F) && (kurzor == 13))                     // skoči na moč
      {
        kurzor = 6;
        vrstica = vrstica_M;
      }

      if ((vrstica == vrstica_M) && ((kurzor < 6) || (kurzor > 6)))     // nazaj na frekvenco
      {
        kurzor = 5;
        vrstica = vrstica_F;
      }
//      Serial.print("kurzor ") ; Serial.println(kurzor);
      break;

    case 14:
      break;
  }
}
