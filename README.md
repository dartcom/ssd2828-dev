# SSD2828-DEV
A collection of boards to test the SSD2828 RGB to MIPI Converter with the LH154Q01 panel by AUO and the VS035ZSM by BOE. The boards are meant to be ordered as a single unit and then be separated.
![](imgs/3D.png)


# LH154Q01
The LH154Q01 panel has been tested and is working correctly.
![](imgs/IMG_5384.JPG)
The panel does not support packed pixel stream packets, the only way to send pixel data to the display, is through DCS commands. in this demo, a STM32f401 is sending the the write_memory_continue(0x3C) DCS command containing pixel data, through SPI to the SSD2828. 

# VS035ZSM
Work in progress.