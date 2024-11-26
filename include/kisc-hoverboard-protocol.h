#ifndef INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED
#define INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Commutation,
    Sinusoidal,
    FieldOrientedControl
} ControlType;

typedef enum {
    OpenMode,
    Voltage,
    Speed,  // Only with FieldOrientedControl
    Torque  // Only with FieldOrientedControl
} ControlMode;

typedef struct {
    bool enable;
    int16_t pwm;
    ControlType ctrlTyp;
    ControlMode ctrlMod;
    uint8_t iMotMax;
    uint8_t iDcMax;
    uint16_t nMotMax;
    uint8_t fieldWeakMax;
    uint8_t phaseAdvMax;
    bool cruiseCtrlEna;
    int16_t nCruiseMotTgt;
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
    uint8_t freq;
    uint8_t pattern;
} BuzzerState;

static inline uint16_t calculateBuzzerChecksum(BuzzerState state) {
    return state.freq ^ state.pattern;
}

typedef struct {
    uint16_t        start;
    MotorState      left, right;
    BuzzerState     buzzer;
    bool            poweroff;
    bool            led;
    bool            cruiseCtrlAcv;
    bool            standstillAcv;
    uint8_t         electricBrakeAmount;
    uint16_t        checksum;
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
    bool      hallA;
    bool      hallB;
    bool      hallC;
} MotorFeedback;

static inline uint16_t calculateMotorFeedbackChecksum(MotorFeedback feedback) {
    return feedback.angle ^ feedback.speed ^
           feedback.error ^ feedback.dcLink ^
           feedback.dcPhaA ^ feedback.dcPhaB ^
           feedback.dcPhaC ^ feedback.chops ^
           feedback.hallA ^ feedback.hallB ^ feedback.hallC;
}

typedef struct {
    uint16_t start;
    MotorFeedback   left, right;
    int16_t         batVoltage;
    int16_t         boardTemp;
    int16_t         timeoutCntSerial;
    bool            cruiseCtrlAcv;
    bool            standstillAcv;
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
