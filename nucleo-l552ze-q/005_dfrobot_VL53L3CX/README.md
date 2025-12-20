# nucleo + VL53L3CX

pin connection:

- SCL → PG14
- SDA → PG13
- XSHUT (Reset) → PA2 (A1) as gpio output
- GPIO1(Interrupt) → PA3 (A0) as gpio input

result:

![alt text](image.png)

## how to

- download the file https://www.st.com/en/embedded-software/stsw-img015.html
- unzip the file
- move `inc` and `src` in `platform` to the project
- drag and drop `core` or `BareDriver` to stm32cubeide
- include `core/inc`
