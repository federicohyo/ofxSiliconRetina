ofxSiliconRetina
=====================================


Introduction
------------
This addon let you interface any Dynamic Vision Sensor to openframeworks.
The Dynamic Vision Sensors are frame-free vision sensors invented, produced and developped by inilabs.com.

License
-------
ofxSiliconRetina is distributed under the [MIT License](https://en.wikipedia.org/wiki/MIT_License), and you might consider using this for your repository. By default, `license.md` contains a copy of the MIT license to which you can add your name and the year.

Installation
------------
Any steps necessary to install your addon. Optimally, this means just dropping the folder into the `openFrameworks/addons/` folder.

Dependencies
------------
This addons includes libcaer and libusb, if missing the one from your operative system. Please compile and copy them into the libs folder.

libcaer can be obained here: https://github.com/inilabs/libcaer
libusb can be obtained here: https://github.com/libusb/libusb

Compatibility
------------
Tested on OF release 0.9.3 

Known issues
------------

Please select the correct device by changing the definition line in ofxSiliconRetina/src/ofxDVS.hpp
/// PLEASE SELECT SENSOR  DVS128 / DAVIS240 / DAVIS346
#define DAVIS240

Version history
------------

### Version 0.11 (25 May 2017):

Implemented frames, imu6 events 

### Version 0.1 (25 May 2017):

Implemented polarity events for DVS128, DAVIS240, DAVIS346B 

