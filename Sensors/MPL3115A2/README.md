# MPL3115A2 pressure sensor

 
* Basic functionality of MPL3115A2 sensor: reads pressure in kPa and temperature in C
* Pin connection is next:
** [MPL3115A2]  [ESP32]
**  VDD          3.3V
**  GND          GND
**  SDA          IO_4
**  SCL          IO_23
 Refer to datasheet page 3 for placing decoupling capacitors.
* output_data_arduino.txt contains reference sensors readings (actually, my sensor is damaged a bit, so values is much higher than it should be - normal pressure is about 101.3 kPa at sea level)
* comment line 209 to remove pressure data offset