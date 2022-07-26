[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/banner2-direct.svg)](https://stand-with-ukraine.pp.ua)

# EDm_BLE

[![version](https://img.shields.io/badge/version-v0.1.0-blue.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/archive/master.zip)
[![release](https://img.shields.io/badge/release-none-brightgreen.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/releases)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/blob/master/LICENSE.txt)

Monitor the status of your Smart electric drive EV with the Blynk app.  
**This is a work-in-progress project - more features to come...**

Further documentation in the [Wiki](https://github.com/MyLab-odyssey/EDm_BLE/wiki).

---
## Intention
When driving your electric car the knowledge of the real state of charge (SoC) is essential. The usable range is directly related to the SoC of the battery. Some EVs - like the Smart ED - will hide some Ah for back-up and the displayed SoC is more pessimistic than the real charge would suggest.

In the [ED_BMSdiag project](https://github.com/MyLab-odyssey/ED_BMSdiag) I developed a diagnostics tool for the Smart electric drive EV based on an Arduino UNO with a CAN bus shield. Data is send via a serial USB connection to a laptop.  
In every day use this is not a practical use case, so I searched for an easy solution to display the data on a mobile phone. [The Blynk app](http://www.blynk.cc) is a great way for a very flexible dashboard on iOS and on Android phones.

<p align="center">
<img  src="https://github.com/MyLab-odyssey/EDm_BLE/wiki/pictures/EDmDongleProject.jpg" width="480"/>
<p/>

You can display the content you need and arrange different widgets to show the values. Using gauges, value displays and graphs the app gives you the freedom to select the data you really need.

<p align="center">
<img  src="https://github.com/MyLab-odyssey/EDm_BLE/wiki/pictures/BlynkDash1.jpg" width="240"/>
<img  src="https://github.com/MyLab-odyssey/EDm_BLE/wiki/pictures/BlynkDash2.jpg" width="230"/><br>
(rearrange widgets or add a graph)<br>
<p/>

Blynk is able to use a bluetooth low energy connection for data transmission. But adding a BLE interface to the Arduino would be only a small step forward. So I decided to develop my own BLE dongle to connect to the OBD port of the car. A system I can leave plugged in while driving and to be energy efficient to stay without depleting the 12V battery. Therefor the RFduino micro controller is the perfect fit as it is small, very efficient and stable. It runs a well developed RTOS and can be programmed with the Arduino IDE. It has excellent ultra low power features for a deep sleep of the module.

## What is Needed
* The PCB: You can use the Eagle CAD files for ordering the PCB at various online manufacturing services. You will find the board on [OSH Park and can order it there](https://oshpark.com/shared_projects/RnRzi1Kz). As an alternative a local FabLab or MakerSpace may has a rapid prototyping mill for PCBs you could use. I made my prototype this way.

* The parts: get the complete bill of materials ([BOM](https://github.com/MyLab-odyssey/EDm_BLE/blob/master/schematics/EDm_v1.1_BOM.txt)) from my GitHub-repo. Most parts are stock articles from common distributors.
You need skills in soldering SMD parts and a good soldering station (the CAN controller is fine pitch).

## Getting started
* Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and then follow the instructions for [RFduino installation](https://github.com/RFduino/RFduino/blob/master/README.md#installation).
* Download the Repo and copy it to your Arduino folder. Find detailed instructions in my ED_BMSdiag project ([english](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Installation) | [german](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Installation_DE)), as it is very similar.
* Using Blynk is described in the Wiki. You can clone my basic project and then modify it. The access token must be copied into the program code and is then compiled for the RFduino.
* Connect the USB to UART adapter and upload the program.
* Remove the USB programmer and plug the dongle into the OBD port of the car. Start Blynk app and connect to the dongle (detailed instructions here).

## Outlook
Currently only live data will be displayed. I will try to port the complete features of the ED_BMSdiag project, so you can use Blynk to get all information about your vehicle.

## Version history
version  | comment
-------- | --------
v0.1.0   | Initial commit with all readouts as in the vehicle dashboard.
