# Converting Hex Values to LED Patterns

Each byte represents one row of 8 LEDs. I convert hex to binary, where:

- `1` = LED ON (●)
- `0` = LED OFF (○)

## Letter A: `{0x7E, 0x81, 0x81, 0x81, 0xFF, 0x81, 0x81, 0x81}`

```
Row 1: 0x7E = 01111110 = ○●●●●●●○
Row 2: 0x81 = 10000001 = ●○○○○○○●
Row 3: 0x81 = 10000001 = ●○○○○○○●
Row 4: 0x81 = 10000001 = ●○○○○○○●
Row 5: 0xFF = 11111111 = ●●●●●●●●  (crossbar)
Row 6: 0x81 = 10000001 = ●○○○○○○●
Row 7: 0x81 = 10000001 = ●○○○○○○●
Row 8: 0x81 = 10000001 = ●○○○○○○●

Visual result:
 ●●●●●●
●      ●
●      ●
●      ●
●●●●●●●●
●      ●
●      ●
●      ●
```

## Letter B: `{0xFE, 0x81, 0x81, 0xFE, 0x81, 0x81, 0x81, 0xFE}`

```
Row 1: 0xFE = 11111110 = ●●●●●●●○
Row 2: 0x81 = 10000001 = ●○○○○○○●
Row 3: 0x81 = 10000001 = ●○○○○○○●
Row 4: 0xFE = 11111110 = ●●●●●●●○  (middle bar)
Row 5: 0x81 = 10000001 = ●○○○○○○●
Row 6: 0x81 = 10000001 = ●○○○○○○●
Row 7: 0x81 = 10000001 = ●○○○○○○●
Row 8: 0xFE = 11111110 = ●●●●●●●○

Visual result:
●●●●●●●
●      ●
●      ●
●●●●●●●
●      ●
●      ●
●      ●
●●●●●●●
```

## Letter C: `{0x7E, 0x81, 0x80, 0x80, 0x80, 0x80, 0x81, 0x7E}`

```
Row 1: 0x7E = 01111110 = ○●●●●●●○
Row 2: 0x81 = 10000001 = ●○○○○○○●
Row 3: 0x80 = 10000000 = ●○○○○○○○
Row 4: 0x80 = 10000000 = ●○○○○○○○
Row 5: 0x80 = 10000000 = ●○○○○○○○
Row 6: 0x80 = 10000000 = ●○○○○○○○
Row 7: 0x81 = 10000001 = ●○○○○○○●
Row 8: 0x7E = 01111110 = ○●●●●●●○

Visual result:
 ●●●●●●
●      ●
●
●
●
●
●      ●
 ●●●●●●
```

## Letter D: `{0xFE, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFE}`

```
Row 1: 0xFE = 11111110 = ●●●●●●●○
Row 2: 0x81 = 10000001 = ●○○○○○○●
Row 3: 0x81 = 10000001 = ●○○○○○○●
Row 4: 0x81 = 10000001 = ●○○○○○○●
Row 5: 0x81 = 10000001 = ●○○○○○○●
Row 6: 0x81 = 10000001 = ●○○○○○○●
Row 7: 0x81 = 10000001 = ●○○○○○○●
Row 8: 0xFE = 11111110 = ●●●●●●●○

Visual result:
●●●●●●●
●      ●
●      ●
●      ●
●      ●
●      ●
●      ●
●●●●●●●
```

## How I Create These Patterns:

1. **Visualize the letter** in an 8x8 grid
2. **Convert each row to binary** (1 = LED on, 0 = LED off)
3. **Convert binary to hex** for each row
4. **Verify** by converting back to visualize

**Example for letter "A":**

- Draw "A" shape in 8x8 grid
- Top: `01111110` → `0x7E`
- Sides: `10000001` → `0x81`
- Crossbar: `11111111` → `0xFF`

This method ensures the hex values will display the intended letters on your LED matrix!

## Online Tools

### 1. **LED Matrix Editor**

**URL:** https://xantorohara.github.io/led-matrix-editor/

### 2. **Gurgle Apps Matrix Tool**

**URL:** https://gurgleapps.com/tools/matrix

### 3. **Riyas.org Font Generator**

**URL:** https://www.riyas.org/2013/12/online-led-matrix-font-generator-with.html

### 4. **8x8 Matrix Calculator**

**URL:** http://xlr8.at/8x8hexbin/
