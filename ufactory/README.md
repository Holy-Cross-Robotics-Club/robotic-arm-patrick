UFACTORY XARM 6
---------------

This directory is for the UFACTORY XARM 6.

Note: This xArm is from a different manufacturer and is unrelated to the
HiWonder xArm in the other directory.

 - `robot.conf` holds IP address for connecting to the xArm. See setup
   instructions below.
 - `shanwan_gamepad.py` a simple proof-of-concept using a cheap shanwan game
   controller to control the joints of the xArm.


# Quick Start

## Configure python

    ~/xarm-python/venv/bin/activate   # or, whatever your venv is
    pip install xarm-python-sdk
    pip install hidapi

## Set up and attach xArm

* Control box (shoebox-sized silver box)
  - Power cord goes to wall power.
  - Large round-locking-connector cable goes to xArm for power.
  - Small round-locking-connector cable goes to xArm for signals.
  - Ethernet cord goes to the beaglebone black.

* Beaglebone Black (small playing-card-sized single-board computer, mounted on
  top of control box)
  - Ethernet cord goes to control box.
  - Make sure the micro-SD card is fully inserted.
  - USB mini cable (with adapter to USB C) goes to laptop.

* Laptop setup
 - USB cable to the beaglebone black. The board should boot within a minute or
   so of being connected to USB, just give it a moment for the blinky blue
   lights to calm down.
 - On laptop it should appear as a network adapter (RNDIS device). Within MacOS
   settings, go to Networks, look for the "AM335x-..." device (if it says
   "Beaglebone" instead, then ensure the micro-SD card is fully inserted in the
   beaglebone, then unplug/replug the usb to reboot). Click Details, find TCP/IP
   settings, and configure:
   - IP address: 192.168.7.1
   - Network mask: 255.255.255.0
   - Router, DNS, IPv6, and all other settings: (leave blank)

* Power-up
 - Plug in control box, turn it on. After a minute it should beep 2 or 3 times indicating it is ready.
 - When beaglebone is connected to USB, it should power up itself.
 - Disengage emergency stop button on control box: twist then pull up.

* Run gamepad demo

    python3 ./shanwan_gamepad.py

* Run examples from ufactory

    git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
    cp robot.conf ./xArm-Python-SDK/example/wrapper/    # copy our robot conf into example folder
    python3 xArm-Python-SDK/example/wrapper/xarm6/2001-move_joint.py

## Details

Control box has only an ethernet interface, so can't be directly connected to a
laptop without some extra hardware or adapters. Here is how it is currently
done, and some alternative options:

1. (current setup) Beaglebone as proxy.
  - Beaglebone runs linux, has ethernet (so it can talk to the control box), and
    plays nicely with a laptop over a USB connection (it appears as an RNDIS
    device.
  - Beaglebone runs a setup script on boot that configures firewall rules. These
    forward all data from the laptop over to the IP address of the control box,
    and vice versa. Only the handfull of known ports (18333, 30000, 30001, etc.)
    are forwarded in this way.
  - The beaglebone is always 192.168.7.2, and it expects the laptop to have a
    similar address. Hence the manual configuration of the laptop's IP address
    for that interface.
  - TODO: beaglebone can be configured to run a dhcp server, which will give the
    laptop with the correct IP address automatically.
  - As configured, the laptop doesn't use the beaglebone for network access
    generally, only for adresses in 192.168.7.*
  - You can can "ssh" to the beaglebone using the username and password printed
    on the label: `ssh beagle@192.168.7.2` 
  - Even if network isn't working properly, on macOS, you should see
    `/dev/tty.usbmodemXXXX` and you can get a console on beaglebone using:
    `screen /dev/tty.usbmodemXXXX 115200`

2. Use a cheap USB-to-Ethernet adapter.
  - Get a cheap adapter to connect laptop USB port directly to ethernet.
  - The control box doesn't run dhcp, so manual network configuration would be
    needed on laptop. If the beaglebone is set up to run dhcp, then it can do
    automatic laptop configuration.
  - Verdict: this option would work just as well as the current beaglebone
    setup, but can't do auto-setup (possible but not yet set up with
    beaglebone).

3. Rasperry Pi as proxy
  - The beaglebone is pretty old. A Rasperry Pi Zero 2 W, or similar RPi, should
    also work in basically the same setup.
  - The Pi Zero 2 W has onboard bluetooth, so that becomes an option. Presumably
    we'd need to set up PAN/PANU to bridge the ethernet traffic over bluetooth.
    Or, make or use a pair of proxy program on the laptop and RPi, to listen on
    port 18333 on the laptop, send the data over bluetooth to RPi, and from
    there to the control box over ethernet. Either way would likely require some
    setup on the laptop.
  - Verdict: strictly better than current beaglebone setup, as it has the option
    of doing bluetooth. 

4. Beaglebone with bluetooth adapter
  - A cheap usb-bluetooth dongle can add bluetooth capability to beaglebone.
  - Verdict: just about the same as Rasperry Pi option, though it's physically a
    bit bigger and more dongles, possibly more power hungry, etc.

