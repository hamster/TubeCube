# TubeCube

**TubeCube** is a IV-17 VFC tube driver in a minibadge-cubed form factor.


# Hardware
The PCB stackup consists of 4 boards, each with a different purpose.  All the boards are connected together with the use of solder bridges.  It's likely not the most sturdy method of construction, but it looks pretty clean.

The tube is an [IV-17 VFD](IV-17.pdf).  As of this writing, you can still get these guys off eBay for a few bucks a tube from Ukraine - a far shake cheaper than Nixie tubes of a similar type.  The IV-17 is alpha-numeric with 16 elements, plus two dots.

![IV-17](tube.png)

The tube is fairly forgiving.  You can get good brightness out of it at only 25V on the grid.  The filament is rated for 2.4V @ 47mA.  You really should drive the filament with an AC waveform, but honestly this tube looks just fine on DC.  In my circuit I'm just feeding it 5V-ish with a 68 ohm resistor.  As long as the filament is not glowing, things are going to be alright.

[Schematic is here.](TubeCube.pdf)

The HV5812 is a SPI controllable high voltage switch that can switch up to 20 elements.  You feed it 3 bytes - a set bit corresponds to turning on the corresponding output.  Extra bits are ignored.  In this circuit, I have the grid hooked into one of the outputs so I have to make sure to always drive that switch - the board layout just worked out that way for me.

The boost supply is partially copied from the [Adafruit Ice Tube Clock](https://learn.adafruit.com/ice-tube-clock-kit/design).

![HVPS](HVPS.png)

The supply is driven from the [ATTINY1616](https://www.microchip.com/wwwproducts/en/ATTINY1616) using a PWM channel at about 10khz.  About 22% duty cycle gives me about 24V output, which seems to work pretty well.  You can push the supply up to about 70V if you really want to.  The nice thing about running it from the microcontroller is that you can idle back the supply to save power when nothing is displaying on the tube.  It's not a highly efficient design - I should revisit this later.  Also worth noting is that we're counting on the chip not locking up while it is switching - it's entirely possible that if the switch stays stuck on, you'll smoke the Zener diode.  Sometimes when programming this happens when the chip is reset, which just results in a failed entry into programming mode.

The microcontroller has a UART and I2C connected up.

# Software
Digit driving is done via SPI.  The tube patterns are pre-computed and stored in ASCII order.  Thankfully, I found [a project](github.com/dmadison/Segmented-LED-Display-ASCII) that had a good base to start from on making the patterns.

At bootup, the hardware is configured, the HVPS is idled down, and the tube is blanked.  The board starts sending out a special character and listening to receive the same on RX.  If we don't detect the char coming in, we assume we are at the head end and will be generating the message banners.  In any case, after the system starts up, if we get valid data in the serial port we stop generating our own banners.

When the module receives a character, it displays that to the tube, and pushes out whatever was currently on the tube out to the next tube in line.

To save power, any time the character to display is a non-display char, we turn down the boost supply.  Normally each cube draws about 90mA, but this technique drops it down to 40-ish mA.

The software was started out using the [Atmel START](https://start.atmel.com/) page and then developed in Atmel Studio.  It's kind of nice to have all the boilerplate generated automatically, but for a project like this, it's not absolutely needed.
