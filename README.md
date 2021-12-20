# MCP23017_i2c_portexpander

## Usage

Determine your i2c device fd by checking available devices in `/dev` or testing your device with
`i2cset`. Then save your device according to the example `params.yaml`.

### Debuggin with i2cset

To test your device beforhands use `i2cset`. You may want to set `0xff` to pin `A0`. First, set
your ports to output
```
i2cset -r 1 0x20 0x00 0x00
```
Then set your port to high with
```
i2cset -r 1 0x20 0x14 0xff
```
To reset use 
```
i2cset -r 1 0x20 0x14 0x00
```
As a bonus you now know the i2c file descriptor.
 