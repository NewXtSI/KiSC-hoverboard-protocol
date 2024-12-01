#ifndef INCLUDE_NEW_PROTOCOL_INCLUDED
#define INCLUDE_NEW_PROTOCOL_INCLUDED

#define HOVER_PROTOVERSION  0x01
#define HOVER_VALID_HEADER  (0xABCD + HOVER_PROTOVERSION)

#define HOVER_CMD_PING_SIZE         0
#define HOVER_CMD_PING              0x01                // Data null        Size 5

#define HOVER_CMD_STATUS_SIZE       7
#define HOVER_CMD_STATUS            0x02                // Size 7
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
#define HOVER_CMD_MOTORSTAT_SIZE    10
#define HOVER_CMD_MOTORSTAT         0x03              // Size 11
// Data:
//   Speed Left (rpm) 2 bytes
//   Speed Right (rpm) 2 bytes
//   Target Left 2 bytes
//   Target Right 2 bytes
//   Flags Left 1 byte
//     Bit 0: Motor Left enabled
//     Bit 1: Motor Left error
//     Bit 2: Motor Left cruiseControl
//   Flags Right 1 byte
//     Bit 0: Motor Right enabled
//     Bit 1: Motor Right error
//     Bit 2: Motor Right cruiseControl
#define HOVER_CMD_MOTORCTRL_SIZE    7
#define HOVER_CMD_MOTORCTRL         0x05              // Size 13
// Data:
//    Brake amount      1 byte
//    Motor left target 2 bytes
//    Motor left flags 1 byte
//    1   Bit 0: Enable
//    2   Bit 1: Direction
//    4   Bit 2: Control Type 0: Commutation, 1: Sinusoidal, 2: Field Oriented Control
//    8   Bit 3: Control Type secondary
//    16  Bit 4: Control Mode 0: Open, 1: Voltage, 2: Speed, 3: Torque
//    32  Bit 5: Control Mode secondary
//    64  Bit 6: Cruise control enable
//    Motor right target 2 bytes
//    Motor right flags 1 byte
//      Bit 0: Enable
//      Bit 1: Direction
//      Bit 2: Control Type 0: Commutation, 1: Sinusoidal, 2: Field Oriented Control
//      Bit 3: Control Type secondary
//      Bit 4: Control Mode 0: Open, 1: Voltage, 2: Speed, 3: Torque
//      Bit 5: Control Mode secondary
//      Bit 6: Cruise control enable
#define HOVER_CMD_BUZZER_SIZE       2
#define HOVER_CMD_BUZZER            0x08
// Data:
//    Frequency 1 byte
//    Pattern 1 byte
#define HOVER_CMD_POWER_SIZE        1
#define HOVER_CMD_POWER             0x06              // Size 1
// Data:
//    Power off 1 byte
#define HOVER_CMD_SETTINGS_SIZE     6
#define HOVER_CMD_SETTINGS          0x07
// Data:
//    Max current 2 bytes
//    Max speed 2 bytes
//    Electric brake 1 byte
//    Parking brake 1 byte

typedef struct {
    int16_t target;
    uint8_t  type;
    uint8_t  mode;
    uint8_t  direction;
    uint8_t  enable;
} KiSCMotorCommand;

typedef struct {
    uint8_t             brake;
    KiSCMotorCommand left;
    KiSCMotorCommand right;
} KiSCCommand;

typedef struct {
    int16_t  maxCurrrent;
    int16_t  maxSpeed;
    uint8_t  eBrake;
    uint8_t  parkingbrake;
} KiSCSettings;
#endif  /* INCLUDE_NEW_PROTOCOL_INCLUDED */
