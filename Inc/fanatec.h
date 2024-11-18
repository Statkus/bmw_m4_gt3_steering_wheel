/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FANATEC_H
#define __FANATEC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
struct fanatec_data_out_t {
  union {
    struct {
      uint8_t header;
      uint8_t id;
      uint8_t buttons[3];
      int8_t axisX;
      int8_t axisY;
      int8_t encoder;
      uint8_t btnHub[2];
      uint8_t btnPS[2];
      uint8_t garbage[19];
      uint8_t fwvers;
      uint8_t crc;
    };
    uint8_t raw[33];
  };
};

/*
 * Buttons mapping for ClubSport Universal Hub (ID=0x04) on Fanatec Control Panel:
 * buttons[0] 1: D-pad up
 * buttons[0] 2: D-pad left
 * buttons[0] 3: D-pad right
 * buttons[0] 4: D-pad down
 * buttons[0] 5: button 2 on right bottom group and cross/enter button on plate
 * buttons[0] 6: button 1 on right top group and square button on plate
 * buttons[0] 7: button 3 on right top group and round button on plate
 * buttons[0] 8: button 2 on right top group and triangle button on plate
 * buttons[1] 1: right shifter paddle and R1 button on plate
 * buttons[1] 2: button 3 on left top group and R2 button on plate
 * buttons[1] 3: button 3 on right down group and R3 button on plate
 * buttons[1] 4: left shifter paddle and L1 button on plate
 * buttons[1] 5: button 1 on left top group and L2 button on plate
 * buttons[1] 6: button 2 on left top group and L3 button on plate
 * buttons[1] 7: button 1 on left bottom group
 * buttons[1] 8: button 3 on left bottom group and start button on plate
 * buttons[2] 1: button 2 on left top group
 * buttons[2] 2: D-pad button
 * buttons[2] 3: button 1 on right bottom group
 * buttons[2] 4: button 2 on left bottom group
 * buttons[2] 5: button 1 on right top group
 * buttons[2] 6: nothing assigned
 * buttons[2] 7: nothing assigned
 * buttons[2] 8: nothing assigned
 * encoder: +1 -> right, -1 -> left
 * btnHub[0] 1: nothing assigned
 * btnHub[0] 2: nothing assigned
 * btnHub[0] 3: nothing assigned
 * btnHub[0] 4: button 3 on left middle group
 * btnHub[0] 5: button 2 on left middle group
 * btnHub[0] 6: button 1 on left middle group
 * btnHub[0] 7: nothing assigned
 * btnHub[0] 8: nothing assigned
 * btnHub[1] 1: nothing assigned
 * btnHub[1] 2: nothing assigned
 * btnHub[1] 3: nothing assigned
 * btnHub[1] 4: nothing assigned
 * btnHub[1] 5: nothing assigned
 * btnHub[1] 6: nothing assigned
 * btnHub[1] 7: nothing assigned
 * btnHub[1] 8: nothing assigned
 * btnPS[0] 1: left shifter paddle and L1 button on plate
 * btnPS[0] 2: right shifter paddle and R1 button on plate
 * btnPS[0] 3: button 1 on left top group and L2 button on plate
 * btnPS[0] 4: button 3 on left top group and R2 button on plate
 * btnPS[0] 5: button 2 on left top group and L3 button on plate
 * btnPS[0] 6: button 3 on right down group and R3 button on plate
 * btnPS[0] 7: select button on plate
 * btnPS[0] 8: button 3 on left bottom group and start button on plate
 * btnPS[1] 1: button 1 on right top group and square button on plate
 * btnPS[1] 2: button 2 on right top group and triangle button on plate
 * btnPS[1] 3: button 3 on right top group and round button on plate
 * btnPS[1] 4: D-pad up
 * btnPS[1] 5: D-pad down
 * btnPS[1] 6: D-pad left
 * btnPS[1] 7: D-pad right
 * btnPS[1] 8: button 2 on right bottom group and cross/enter button on plate
 *
 * Buttons mapping for ClubSport Universal Hub (ID=0x04) on standard gamepad:
 * buttons[0] 1: D-pad up
 * buttons[0] 2: D-pad left
 * buttons[0] 3: D-pad right
 * buttons[0] 4: D-pad down
 * buttons[0] 5: button 1
 * buttons[0] 6: button 0
 * buttons[0] 7: button 2
 * buttons[0] 8: button 3
 * buttons[1] 1: button 4
 * buttons[1] 2: button 6
 * buttons[1] 3: button 10
 * buttons[1] 4: button 5
 * buttons[1] 5: button 7
 * buttons[1] 6: button 11
 * buttons[1] 7: button 8
 * buttons[1] 8: button 9
 * buttons[2] 1: button 20
 * buttons[2] 2: button 24
 * buttons[2] 3: button 25
 * buttons[2] 4: button 21
 * buttons[2] 5: button 26
 * buttons[2] 6: button 27
 * buttons[2] 7: nothing assigned
 * buttons[2] 8: nothing assigned
 * encoder: +1 -> button 23, -1 -> button 22
 * btnHub[0] 1: nothing assigned
 * btnHub[0] 2: nothing assigned
 * btnHub[0] 3: nothing assigned
 * btnHub[0] 4: button 39
 * btnHub[0] 5: button 40
 * btnHub[0] 6: button 41
 * btnHub[0] 7: nothing assigned
 * btnHub[0] 8: nothing assigned
 * btnHub[1] 1: nothing assigned
 * btnHub[1] 2: nothing assigned
 * btnHub[1] 3: nothing assigned
 * btnHub[1] 4: nothing assigned
 * btnHub[1] 5: nothing assigned
 * btnHub[1] 6: nothing assigned
 * btnHub[1] 7: nothing assigned
 * btnHub[1] 8: button 51
 * btnPS[0] 1: button 5
 * btnPS[0] 2: button 4
 * btnPS[0] 3: button 7
 * btnPS[0] 4: button 6
 * btnPS[0] 5: button 11
 * btnPS[0] 6: button 10
 * btnPS[0] 7: button 8
 * btnPS[0] 8: button 9
 * btnPS[1] 1: button 0
 * btnPS[1] 2: button 3
 * btnPS[1] 3: button 2
 * btnPS[1] 4: D-pad up
 * btnPS[1] 5: D-pad down
 * btnPS[1] 6: D-pad left
 * btnPS[1] 7: D-pad right
 * btnPS[1] 8: button 1
*/

struct fanatec_data_in_t {
  union {
    struct {
      uint8_t header;
      uint8_t id;
      uint8_t disp[3];
      uint16_t leds;
      uint8_t rumble[2];
      uint8_t nothing[23];
      uint8_t crc;
    };
    uint8_t raw[33];
  };
};

/* Exported constants --------------------------------------------------------*/
// CRC lookup table with polynomial of 0x131
extern const uint8_t CRC8_table[256];

/* Exported functions prototypes ---------------------------------------------*/
uint8_t Compute_CRC(const uint8_t* buffer, uint8_t length);

#endif /* __FANATEC_H */
