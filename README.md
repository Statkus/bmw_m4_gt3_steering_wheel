# BMW M4 GT3 steering wheel

This project is a rough replica of the BMW M4 GT3 wheel, entirely printed in 3D with plastic filament.

## Bill of materials

- PLA or any other filament that you like
- screws:
  - 4 x DIN 912 M2 10mm
  - 1 x DIN 912 M2.5 10mm
  - 28 x DIN 912 M3 6mm
  - 6 x DIN 912 M3 16mm
  - 4 x DIN 912 M3 25mm
  - 6 x DIN 912 M5 55mm
  - 8 x DIN 912 M6 22mm
- nuts:
  - 4 x M2
  - 1 x M2.5
  - 10 x M3
  - 6 x M5
  - 8 x M6
- 28 x M3 inserts OD 5mm length 5mm
- 4 x 12x3mm magnets
- tennis handle grip or similar material for the handles
- 1 x WeAct Black Pill V3.0 (https://stm32-base.org/boards/STM32F401CEU6-WeAct-Black-Pill-V3.0) or any STM32F411CEU6 based board with same form factor
- 12 x pins from deutsch connectors 0460-202-20141
- 1 x 12 pins JST PH male connector
- 1 x 12 pins JST PH female connector
- 5 x EC11 encoders with 15mm plum handle with or without push buttons
- 14 x B3F-4055 tactile switches
- 14 x 10mm caps for B3F-4055 tactile switchs
- 2 x DM1-01P-30-3 switches
- wires to link everything

## Source code

### Usage

STM32CubeMX is used to generate STM32 library source files.

Source code can be build with the command:
```
make all
```

And you can flash the program with:
```
make flash
```

STM32CubeMX generates file in DOS format, to convert files to UNIX format use:
```
make format
```

### Fanatec SPI protocol

The SPI protocol used between the fanatec wheel and wheel base as already been retro engineered. Reference of this protocol can be found on the following GitHub repositories:
- https://github.com/Alexbox364/F_Interface_AL
- https://github.com/darknao/btClubSportWheel
- https://github.com/lshachar/Arduino_Fanatec_Wheel

The pinout of the QR pins can already be found on those repositories.

## 3D files

3D_files/Pin_holder_FINAL.stl and 3D_files/Pin_locker_FINAL.stl have been designed by Alexbox364 and are available on Thingiverse: https://www.thingiverse.com/thing:6271297.

The other parts are designed with FreeCAD.

## TODO

- Add clutch paddles
