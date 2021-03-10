SIGNAL GENERATOR ADF4351
ESP32
Push buttons for moving cursor and selecting values are connected to pins 34, 35, 36 and 39. Those pins are GPIs – input only pins and they don’t have internal pull-ups or pull-down resistors, so we must add external pull-up resistors. Few kΩ will be OK. Buttons are connected from pins to ground. The button on pin 39 will move cursor to the left and the button on pin 35 to the right. The underlined digit can be modified with the up button on pin 34 and down with the button on pin 36.
Pull-up resistors of 3,9 kΩ must also be added to pins 21 and 22 (SDA and SCL).
For display you can use any OLED with I2C interface. Of course, you must choose the right constructor.
Last used frequency and output power are stored in EEPROM.
ADF4351
You can find ADF4351 evaluation board on eBay or AliExpress for approx. 20€.
ADF4351 board is connected to ESP32 via HSPI port. VSPI port is free for future use (TFT display, for example).
Connections:
  pin  ESP32     ADF
   14  CLK    -> CLK	SPI
   13  MOSI   -> DAT 	SPI
   27  CS, CE -> LE 	SPI
   25  locked -> MUX	status
   26  OUT    -> LD	green LED
   EN         -> CE	enable output

The microcontroller writes ADF4351 registers using a one-way serial protocol. MISO is not used. The maximum SPI speed is 20 MHz.
If CE is HIGH, ADF4351output is enabled. If you don’t use this signal to control the output, connect it to the EN on ESP32 board.
Unused output must be terminated with 50Ω.
OLED display is connected to default I2C interface on pins 21 and 22 (SDA and SCL).
Frequency step is 0,1 kHz, but tenths of Hz and Hz are not guaranteed to be zero!
Frequency band is from 34,376 Mhz to 3,6 GHz. I do not have neither a need neither an instrument for frequencies above. Upper frequency can be moved to 4,4 GHz by setting prescaler to 1, that’s 8/9. When using this prescaler, the minimum N value is 75, so do this at your own risk!
Built-in reference oscillator is more than adequate for most users. To use an external reference signal, resistor labelled “0” near the oscillator must be removed from the ADF4351 module. But be careful! ADF4351 run on 3,3 V, so amplitude must not exceed this voltage.
ADF4351 is very sensitive to noise in power supply! It is recommended to power it from low noise supply. Do not use USB as power source!
