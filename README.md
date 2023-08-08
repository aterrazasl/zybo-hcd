# zybo-hcd
Implementation of a simple USB host controller driver

Goal:
Create a USB controller driver capable of running outside the linux environment
HCD (Host controller driver) should be light enough to run in small Zynq projects
This particular implementation is based on the Zynq7000 and the Digilent Board Zybo Z7

The HCD is based on the EHCI 1.1 released by Intel and should support as minimum for this first version the following:

-Low/Full speed devices
-High speed devices
-Control and Bulk transfers
-Chapter 9 of USB 2.0 enumeration
-Async Transfers
-64 byes data lenght

Not supported features/future versions
-Split transactions
-USB Hubs
-Sleep/Resume
-Periodic transfers
-Isochronous transfers

HCD should expose USB 2.0 standard request through the API also low level functions to raw queue/enqueue of transfer descriptors
Only one HCD driver instance can be created and the HCD reference can be used by device drivers in order to implement the final device interface

One of the motivations to start this project is to add support to small embedded projects features to interact with USB devices

Common devices used like: Keyboard, mouse and memories are hard to incorporate in small embedded devices, hence the motiviation to create this HCD

As a starting point a Keyboard USB HID driver is implemented


