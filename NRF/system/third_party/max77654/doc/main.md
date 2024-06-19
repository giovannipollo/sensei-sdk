# MAX77654 Driver

Philipp Schilk, 2022

\tableofcontents

## Usage

Create a handle struct (@ref max77654_h), and provide the device configuration and functions to communicate with 
the device.

The following hardware-specific functions have to be provided in the handle at a minimum:

  - read_reg: Reads a register from the PMIC.
  - write_reg: Reads a register from the PMIC.

Call @ref max77654_init to initialize the device.

See [here](./globals_func.html) for a list of functions.

See @ref max77654_h for more details about the handle struct, and see 
@ref max77654_conf for more details abut available config options.

Example Outline:
```c
// Create a handle struct, providing all relevant hardware-specific functions
// and configuration:
struct max77654_h h = {
  .read_regs = read_regs,
  .write_regs = write_regs,
  .adc_read = adc_read,
  .log = drvrlog,
  // Configuration:
  .conf = {
    .cid = 0xA,
    .main_bias_force_enable = true,
    .... many more settings .....
  },
};

// Configure the device:
if (max77654_init(&h) != E_MAX77654_SUCCESS) {
  printf("Driver Init failed!\n");
  return -1;
}
```

