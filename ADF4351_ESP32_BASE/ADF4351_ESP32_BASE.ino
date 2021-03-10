/*
 * ESP32   Dev Module
 * ADF je na HSPI
 * VSPI je prost za TFT zaslon
 * 
 *  pin  ESP32     ADF
 *   14  CLK    -> CLK
 *   13  MOSI   -> DATA
 *   27  CS, SS -> LE
 *   26  CE     -> CE
 *  
 *   VSPI     5 SS, CS, CE     - SW za TFT
 *           18 SCK            - HW
 *           19 MISO, SDO      - HW
 *           23 MOSI, SDI, SDA - HW
 *           nn A0, DC, CD, RS - SW
 *              RESET          - SW ali RESET
 *
 *    ADF    27 SS, CS, CE     - SW
 *           14 SCK            - HW
 *           12 MISO, SDO      - HW ni v uporabi
 *           13 MOSI, SDI, SDA - HW
 *           
 *           andrejkregar53@gmail.com
*/


// SPI zadeve
#include <SPI.h>
SPIClass ADF(HSPI);                           // naredimo še en SPI Class in mu damo ime ADF. Fizično je vezan na drugi SPI vmesnik

// ADF pini
#define ADF_LE_Pin  27                        // CE, CS, SS
#define ADF_CE_Pin  26                        // mora biti HIGH

uint64_t frekvenca;                           // nekaj čudnega: pri uint32_t nekatere frekvence ne delajo

#define REF_CLK 25000000                      // izmerjena interna referenca 24.999.864 Hz

#define SHL(x,y) ((uint32_t)1<<y)*x           // za vpis v registre

// REGISTRI
// Register 0:
uint16_t INT = 0;
uint16_t FRAC = 0;

// Register 1:
uint8_t  phase_adj = 0;                       // 0 OFF   1 ON        0 - dela???                                        -   RF Settings
uint8_t  prescaler = 0;                       // 0 4/5   1 8/9       0 - največ 3,6 GHz                                 -   RF Settings
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
uint8_t  RF_out_pwr;                          // 0 -4dBm, 1 -1dBm, 2 +2dBm, 3 +5dBm
                                              // 0,4 mW   0,8 mW   1,6 mW   3,16 mW
uint8_t  Band_Select_CLK = 100;               // 0b00..01 1, 0b00..10 2, 0b11..00 252, 0b11..11 255    100 po SW

// Register 5:
uint8_t  LD_pinmode = 1;                      // 1 digital lock detect

// Register 5:
uint32_t reg[6] = {0, 0, 0, 0, 0, 0};

// enačba 4
uint32_t pfd_freq = (REF_CLK * (1 + Ref_doubler)) / (Ref_counter * ((1 + Ref_div2)));        


///////////// setup /////////////
void setup()
{
  pinMode (ADF_CE_Pin, OUTPUT);
  digitalWrite(ADF_CE_Pin, HIGH);
  
  // SPI_2
  pinMode (ADF_LE_Pin, OUTPUT);
  ADF.begin(14, 12, 13, 27);                  // CLK, MISO, MOSI, SS     privzete vrednosti, razen SS!
  delayMicroseconds(5);                       // dodano 11.4                    Kako je že rekla nuna?
  ADF.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));         // dodano 19.4     
  delay(50);
  pripravi_registre();
  posodobi_vse_registre();                    // pobrišemo staro stanje
  delay(50);
}

///////////// loop /////////////
void loop()
{
  frekvenca = 50000000;
  RF_out_pwr = 2;
  pripravi_registre();
  posodobi_vse_registre();
  while(1);
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
}

void posodobi_vse_registre()
{
  for (int i = 5; i >= 0; i--)
  {
    vpisi_ADF_registre(i);
  }
}
