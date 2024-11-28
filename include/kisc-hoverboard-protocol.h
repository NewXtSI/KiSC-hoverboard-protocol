#ifndef INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED
#define INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Commutation,            // 1
    Sinusoidal,
    FieldOrientedControl
} ControlType;              // 1

typedef enum {
    OpenMode,
    Voltage,
    Speed,  // Only with FieldOrientedControl
    Torque  // Only with FieldOrientedControl
} ControlMode;

typedef struct {
    uint16_t enable;            // 1        0
    int16_t pwm;            // 2        1
    ControlType ctrlTyp:8;    // 1        3
    ControlMode ctrlMod:8;    // 1        4
    uint16_t iMotMax;        // 1        5
    uint16_t iDcMax;         // 1        6
    uint16_t nMotMax;       // 2        7
    uint16_t fieldWeakMax;   // 1        9
    uint16_t phaseAdvMax;    // 1        10
    uint16_t cruiseCtrlEna;     // 1        11
    uint16_t nCruiseMotTgt;  // 2        12
} MotorState;

static inline uint16_t calculateStateChecksum(MotorState state) {
    return
        (uint16_t)state.enable ^
        state.pwm ^
        (uint16_t)state.ctrlTyp ^
        (uint16_t)state.ctrlMod ^
        state.iMotMax ^
        state.iDcMax ^
        state.nMotMax ^
        state.fieldWeakMax ^
        state.phaseAdvMax ^
        (uint16_t)state.cruiseCtrlEna ^
        state.nCruiseMotTgt;
}

typedef struct {
    uint16_t freq;
    uint16_t pattern;
} BuzzerState;

static inline uint16_t calculateBuzzerChecksum(BuzzerState state) {
    return state.freq ^ state.pattern;
}

typedef struct {
    uint16_t        start;          // 2        0
    MotorState      left, right;    // 28       2
    BuzzerState     buzzer;         // 2        30
    uint16_t            poweroff;       // 1        32
    uint16_t            led;            // 1        33
    uint16_t            cruiseCtrlAcv;  // 1        34
    uint16_t            standstillAcv;  // 1        35
    uint16_t         electricBrakeAmount;    // 1        36
    uint16_t        checksum;    // 2        37
} SerialCommand;

static inline uint16_t calculateCommandChecksum(SerialCommand command) {
    return command.start ^
           calculateStateChecksum(command.left) ^
           calculateStateChecksum(command.right) ^
           calculateBuzzerChecksum(command.buzzer) ^
           command.poweroff ^
           command.led ^
              command.cruiseCtrlAcv ^
                command.standstillAcv ^
                    command.electricBrakeAmount;
}

typedef struct {
    int16_t   angle;
    int16_t   speed;
    uint8_t   error;
    int16_t   dcLink;
    int16_t   dcPhaA;
    int16_t   dcPhaB;
    int16_t   dcPhaC;
    uint16_t  chops;
    int16_t   id;
    int16_t   iq;
    uint16_t      hallA;
    uint16_t      hallB;
    uint16_t      hallC;
} MotorFeedback;

static inline uint16_t calculateMotorFeedbackChecksum(MotorFeedback feedback) {
    return feedback.angle ^ feedback.speed ^
           feedback.error ^ feedback.dcLink ^
           feedback.dcPhaA ^ feedback.dcPhaB ^
           feedback.dcPhaC ^ feedback.chops ^
           feedback.hallA ^ feedback.hallB ^ feedback.hallC;
}

#define         VALID_HEADER    0x5A5A

typedef struct {
    uint16_t        start;
    MotorFeedback   left, right;
    int16_t         batVoltage;
    int16_t         boardTemp;
    int16_t         timeoutCntSerial;
    uint8_t            cruiseCtrlAcv;
    uint8_t            standstillAcv;
    uint8_t         electricBrakeAmount;
    uint16_t        checksum;
} SerialFeedback;

static inline uint16_t calculateFeedbackChecksum(SerialFeedback feedback) {
    return feedback.start ^
            calculateMotorFeedbackChecksum(feedback.left) ^
            calculateMotorFeedbackChecksum(feedback.right) ^
            feedback.batVoltage ^
            feedback.boardTemp ^
            feedback.timeoutCntSerial ^
            feedback.cruiseCtrlAcv ^
            feedback.standstillAcv ^
            feedback.electricBrakeAmount;

}

#ifdef __cplusplus
}
#endif


#endif  /* INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED */
