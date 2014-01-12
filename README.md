ATtiny4313_TestPCA9543
======================

I used this program to test a PCA9543 I2C bus switch.

The PCA9543 connects up to two downstream I2C channels. I put one on a
Surfboard and connected it to two MCP23008 port expanders.

There's not much to see here. You can write to the single control register
to select one, none, or both downstream channels and you can read the same
control register to see the state of the interrupt pin inputs.

2014-01-12/au
