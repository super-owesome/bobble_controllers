
# Getting Started With Your Bobble-Bot
So you've got your Bobble-Bot in hand, and you're not sure what to do next? This 
guide will get you balancing and driving in no time. As an added bonus, we'll show you 
how to stream live data from the robot, and command it via command line and the 
Python API. Let's get started.

## Booting Up
This section should cover starting with the SD card and loading the latest software image.

Picture of dissassembled bot with SD card out goes here.

### Connecting to your Wi-Fi
While you're poking around with the SD card and the RasPi image, it's a good idea to 
go ahead and add your WiFi network configuration. This will allow Bobble-Bot to 
connect up to your network on boot. This will be necessary for streaming data and 
issuing commands from a networked device. You can skip this step if you plan to 
rely only on the Xbox controller for driving.

Edit /etc/wpa_supplicant/wpa_supplicant.conf to add lines like the following:

```
network={
	ssid="YOUR_NETWORK_NAME_HERE"
	psk="your_network_password_here"
}
```

More information about this file can be found [here](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md)




