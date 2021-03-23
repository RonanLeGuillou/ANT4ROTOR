# ANT4ROTOR

Authors : Martin Schmoll, Ronan Le Guillou, Christine Azevedo

Code intended for free use as open source project, refer to the included MIT license for legal specifications.

Python program enabling fast mode data acquisition for the ROTOR 2INPOWER commercially available bike crank power-meter. The goal being to acquire detailed data throughout pedalling cycles like for the crank angle or the torque production. 

Based on the openANT (by Gustav Tiger) ANT+ communication protocol's Python library , this project unlocks the low frequency limits (~4Hz) from the default ANT+ protocol to allow acquisition of the ROTOR "Fast Mode" data streaming (~50Hz) through ANT+. This enables data acquisition and monitoring of crank angle, cadence, average power, torque applied on the left and right side independently and more, at frequencies sufficient to analyse the pedalling dynamics inside of a turn rather than having a simple overview of it.


Tested for Raspberry Pi 3B only.


Specific packages recommended :

python2

python-usb   (for python 2)
