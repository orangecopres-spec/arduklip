<img src="https://github.com/orangecopres-spec/arduklip/blob/main/arduklip.png?raw=true" alt="arduklip.png"/>




this is the arduklip repo! Tired of having to buy rasperry pi for custom built printers? Well, look no further! arduklip runs in a uno r3 and your linux computer!

how to run:

install klipper and make sure the log file is intact.

#plugs in arduino uno r3
python3 serial_bridge.py
#opens another tab of the terminal
python3 mock_fluidd.py #you do not have to run mock fluidd if you have a macros inserterthat can insert into ~/printer_data/logs/klippy.log

then go to the adresses if you use mock-fluidd go to the adress listed and you will see
<img width="1916" height="1079" alt="image" src="https://github.com/user-attachments/assets/ac8422d3-8bdd-40fb-9c6e-c5095084208b" />



then hook up to avrdude stepper motors. Done!

Octoprint support #beta:

install bridge-serial-octoprint.ino.ino on your arduino uno r3 then go to octoprint and make a profile called o1vo with 200 by 200 by 200 and regular settings then save then connect to it at baud: Auto and Serial: auto and profile o1vo and connect and print a thing!

also on the new relese edit the code at the top until you see a #define SIMULATION_MODE 0   // 1 = simulator, 0 = real hardware turn it to 1 to simulate


