#ifndef INCLUDE_NEW_PROTOCOL_INCLUDED
#define INCLUDE_NEW_PROTOCOL_INCLUDED

#define HOVER_VALID_HEADER  0xABCD
#define HOVER_CMD_PING      0x01                // Data null        Size 5
#define HOVER_CMD_STATUS    0x02                // Size 7
// Data:
//   Battery voltage (mV) 2 bytes
//   Board temperature (°C) 2 bytes
//   DC current (mA) 2 bytes
//   Flags 1 byte
//     Bit 0: Cruise control active
//     Bit 1: Standstill active
//     Bit 2: Charging
//     Bit 3: Error Motor Left
//     Bit 4: Error Motor Right
//     Bit 5: Error Board
#define HOVER_CMD_MOTORSTAT   0x03              // Size 11
// Data:
//   Speed Left (rpm) 2 bytes
//   Speed Right (rpm) 2 bytes
//   Target Left 2 bytes
//   Target Right 2 bytes
//   Flags Left 1 byte
//     Bit 0: Motor Left enabled
//     Bit 1: Motor Left error
//   Flags Right 1 byte
//     Bit 0: Motor Right enabled
//     Bit 1: Motor Right error
#define HOVER_CMD_MOTORCTRL   0x04              // Size 13
// Data:
//    Motor left target 2 bytes
//    Motor left flags 1 byte
//      Bit 0: Enable
//      Bit 1: Direction
//      Bit 2: Control Type 0: Commutation, 1: Sinusoidal, 2: Field Oriented Control
//      Bit 3: Control Type secondary
//      Bit 4: Control Mode 0: Open, 1: Voltage, 2: Speed, 3: Torque
//      Bit 5: Control Mode secondary	
//    Motor right target 2 bytes
//    Motor right flags 1 byte
//      Bit 0: Enable
//      Bit 1: Direction
//      Bit 2: Control Type 0: Commutation, 1: Sinusoidal, 2: Field Oriented Control
//      Bit 3: Control Type secondary
//      Bit 4: Control Mode 0: Open, 1: Voltage, 2: Speed, 3: Torque
//      Bit 5: Control Mode secondary
#define HOVER_CMD_BUZZER      0x05              
// Data:
//    Frequency 1 byte
//    Pattern 1 byte
#define HOVER_CMD_POWER       0x06              // Size 1
// Data:
//    Power off 1 byte
#define HOVER_CMD_SETTINGS    0x07             
// Data:
//    Max current 2 bytes
//    Max speed 2 bytes
//    Electric brake 1 byte



#endif  /* INCLUDE_NEW_PROTOCOL_INCLUDED */
