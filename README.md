# 3DRacers RacerOS
3DRacers is the first 3D Printed Racing Game, Smartphone-driven and Arduino-compatible

RacerOS is the Arduino-compatible code that runs inside the ATMega32U4 MCU inside each 3DRacers

![Alt text](/photos/shell-help.jpg?raw=true "3DRacers RacerOS screen")

## The Game

- Race with your friends (up to 1000 players)
- Drive with the Android/Iphone app or the 3D printed remote
- Watch real time score boards and lap times: each car has a sensor that detect specially designed gates.
- Re-program the Arduino core inside each car to unlock new features from the community

More info on: [3DRacers website]

Follow us on Twitter: [@3dracers] and on our community [forum].

Author: Marco D'Alia [@madarco]

## Setup

Just download the Arudino code and load it to your car, like a standard Arduino.

You can use the code as a standard Arduino sketch (use the examples in the examples folder to start), or copy the code to the libraries folder of Arduino:

- Copy the Lib3DRacers to the libraries/Lib3DRacers folder
- Also copy the Lib3DRacersVendors in the libraries/Lib3DRacersVendors folder.

## Car Setup

If it's the first time, you'll have to configure the Servo movement range and center position (ie: if your car doesnt't go straight)

Do that with the /servo1 commands in the console or through the mobile app.

![Alt text](/photos/app-car_setup.jpg?raw=true "3DRacers App car setup")

## Usage

You can connect to the RacersOS console by plugging the car by USB and opening the Arduno console.

If you open the console in the first 5 seconds you'll see the Welcome screen with some usefull debug info:

![Alt text](/photos/shell-welcome.jpg?raw=true "3DRacers welcome screen")

You can also issue some commands:

- /at AT+* : Send BLE commands directly to the Bluetooth 4.0 module. NB: if the module is connected to the App, the data will be sent directly to it as a Byte stream.
- /showinfo : Get actual car configuration: servo setup, battery, connection status
- /motorscheck : Move the servos R/L and the motors F/B for testing
- /servo1 center <angle> : Set servo center rest position, use when you built your own 3DRacers
- /servo1 max <angle> : Set servo max rotation angle to avoid damaging it by pushing the brackets to the car chassis.
- /netdebug : Enable/Disable net packets trace: print to the console the detailed description of each received/sent packet

## Sketch size

If your sketch is too big, you can disable the console code form the Config.h file inside the Lib3DRacers library, this will free an additional 10k of memory.

## The PCB board

3DRacers is Open Source an Open Hardware, check out also the [PCB board sources]

### Development

Want to contribute? Great!

Just fork this repo or create an issue.
Also, follow the community [forum]

License
----

Creative Commons Share-Alike 4.0


**Open Software, Hell Yeah!**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does it's job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [forum]: <http://forum.3dracers.com>
   [Lib3DRacers repository]: <https://github.com/3DRacers/Lib3DRacers>
   [3DRacers website]: <http://www.3dracers.com>
   [@madarco]: <http://twitter.com/madarco>
   [@3dracers]: <http://twitter.com/3dracers>
   [PCB board sources]: <https://github.com/3DRacers/PilotBoard>





