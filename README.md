# rtkgps

Publish GPS coordinates in ROS using the pygnssutils Python 3 library.

## Instalation
python3 -m pip install --upgrade pygnssutils

Fix OpenSSL

Delete crypto.py
pip install pyopenssl --upgrade
(https://stackoverflow.com/questions/73830524/attributeerror-module-lib-has-no-attribute-x509-v-flag-cb-issuer-check)

## Launch
source ROS environment
roslaunch rtkgps rtk.launch
