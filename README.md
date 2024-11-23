# Filament dryer

This is a filament dryer that I designed and built.

<div style="display: flex; justify-content: space-around; align-items: center;">
  <img src="images/filament_dryer01.png" width="400">
  &nbsp;
  <img src="images/filament_dryer02.png" width="400">
</div>

Box made from 2mm aluminium sheets and polywood. Electrincics are housed in a 3D printed box.

The dryer is powered by a 12V 2.5A power supply. The heater is a 220V 40W heater. The fan is a 12V 0.2A fan.
The dryer is controlled by an Arduino (Farduino nano) and a DHT22 sensor.

<img src="images/filament_dryer03.png" width="400">
<br/>
<img src="images/filament_dryer05.png" width="400">

The Arduino connected to 4-digit 7-segment display and two buttons - plus and minus.

If buttons are pressed, the temperature can be adjusted. The temperature is displayed on the 4-digit 7-segment display.

After 20 seconds of inactivity, screen swiches to display current parameters - pid, temperature and humidity.

<img src="images/filament_dryer04.png" width="400">

The dryer can be used to dry out filament that has absorbed moisture from the air.
