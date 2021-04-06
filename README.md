# ANT4ROTOR

Authors : Martin Schmoll, Ronan Le Guillou, Christine Azevedo

### Introduction

Code intended for free use as open source project, refer to the included MIT license for legal specifications.

Python program enabling fast mode data acquisition for the ROTOR 2INPOWER commercially available bike crank power-meter. The goal being to acquire detailed data throughout pedalling cycles like for the crank angle or the torque production. 

Based on the openANT (by Gustav Tiger) ANT+ communication protocol's Python library , this project unlocks the low frequency limits (~4Hz) from the default ANT+ protocol to allow acquisition of the ROTOR "Fast Mode" data streaming (~50Hz) through ANT+. This enables data acquisition and monitoring of crank angle, cadence, average power, torque applied on the left and right side independently and more, at frequencies sufficient to analyse the pedalling dynamics inside of a turn rather than having a simple overview of it.


Tested on Raspberry Pi 3B v1.2 with RaspberryOS/Raspbian only.

### Install instructions :

Use these specific recommended packages :

python2

python-usb   	(for python 2, version was 1.0.2-1)

python-serial	(for python 2, version was 3.4-4)


(You can use the requirements.txt file to get the exact packages and versions that were used for the last test)

Move (or copy) the ant-usb-sticks.rules file into /etc/udev/rules.d folder alongside the default 99-com.rules file and reboot your raspberry pi for the new rules to apply.
This file creates a ttyANTX link in /dev so that the ANT communication dongle device can be referred to. 
For debuging purposes, if you have connection problems, the opening of the device port is in the openANT/ant/base/driver.py file, in the USBDriver class, in the open() method. 
However, check first if your Ant+ dongle is detected with lsusb, and if ttyANT3 (for example) appears in the /dev folder (don't forget to reboot to apply the new rules). Otherwise make sure that the udev rules used are appropriate to your system and the device you are using. One pair of idVendor and idProduct in the ant-usb-sticks.rules should match the ones you see on your Ant+ dongle when using the lsusb command. 


Start myOpenANT_Manager.py with python2 and move the crank of your ROTOR device so that it wakes up and can be connected to.

Samplerate should be 40Hz or higher depending of the parameters requested to the ROTOR device. Enjoy !



