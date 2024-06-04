# generic can driver

## Setup

1. To set up can port run the following in terminal on the host machine:

PEAK: 
``sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on ``

canusb:
``sudo slcand -o -c -f -s5 /dev/ttyUSB0 can0 ``
``sudo ifconfig can0 up ``

## Errors

1. ``Could not load library dlopen error: /workspaces/vscode-container-workspace/install/lib/libgeneric_can_driver.so: undefined symbol: _ZTVN18generic_can_driver20GenericCanDriverNodeE, at ./src/shared_library.c:99``

        If you're editing something in the generic can driver (or any derived driver) and get an error 
        saying something like the above, try deleting the generic_can_driver folder in ``/build`` and
        ``/install``. Then, rebuild and relaunch

 2. Socket Can refuses to change lifecycle state

        Make sure you have a can bus enabled and that its name is correct in the launch file/params