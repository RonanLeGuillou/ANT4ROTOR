ANT+ Communication for ROTOR 2INPOWER
=======

Requirements
------------

- Python 2.7+
- PyUSB > 1.0a2 (seems to be a bug which makes it segfaults on version before Alpha 2)
- Root access (for installation only)


Manual install
--------------

These should only be necessary to install manually, if you don't want to use the Automatic installation script.

- Install [PyUSB](https://github.com/walac/pyusb).

        pip install pyusb

    *(Or alternatively from [sources available on GitHub](https://github.com/walac/pyusb))*

- Install [udev](http://en.wikipedia.org/wiki/Udev) rules (Only required to avoid running the program as root).

        sudo cp resources/ant-usb-sticks.rules /etc/udev/rules.d
        sudo udevadm control --reload-rules
        sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=0fcf --action=add

Supported devices
-----------------

### ANT USB Sticks

 - [ANTUSB2 Stick](http://www.thisisant.com/developer/components/antusb2/)
 (0fcf:1008: Dynastream Innovations, Inc.)
 - [ANTUSB-m Stick](http://www.thisisant.com/developer/components/antusb-m/)
 (0fcf:1009: Dynastream Innovations, Inc.)
