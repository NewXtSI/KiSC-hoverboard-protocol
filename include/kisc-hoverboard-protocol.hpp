#ifndef INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED
#define INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED

#include <stdint.h>

namespace kisc {
namespace protocol {

enum class ControlType : uint8_t {
    Commutation,
    Sinusoidal,
    FieldOrientedControl
};

enum class ControlMode : uint8_t {
    OpenMode,
    Voltage,
    Speed,  // Only with FieldOrientedControl
    Torque  // Only with FieldOrientedControl
};

}  // namespace protocol
}  // namespace kisc

namespace kisc {
namespace protocol {
namespace serial {

struct MotorState {
    bool enable = false;
    int16_t pwm = 0;
    ControlType ctrlTyp = ControlType::FieldOrientedControl;
    ControlMode ctrlMod = ControlMode::OpenMode;
    uint8_t iMotMax = 15;                  // [A] Maximum motor current limit
    uint8_t iDcMax = 17;                   // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
    uint16_t nMotMax = 1000;               // [rpm] Maximum motor speed limit
    uint8_t fieldWeakMax = 10;             // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
    uint8_t phaseAdvMax = 40;              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
    bool cruiseCtrlEna = false;
    int16_t nCruiseMotTgt = 0;
};

inline uint16_t calculateChecksum(MotorState state) {
    return
        uint16_t(state.enable) ^
        state.pwm ^
        uint16_t(state.ctrlTyp) ^
        uint16_t(state.ctrlMod) ^
        state.iMotMax ^
        state.iDcMax ^
        state.nMotMax ^
        state.fieldWeakMax ^
        state.phaseAdvMax ^
        uint16_t(state.cruiseCtrlEna) ^
        state.nCruiseMotTgt;
}

struct BuzzerState {
    uint8_t freq = 0;
    uint8_t pattern = 0;
};

inline uint16_t calculateChecksum(BuzzerState state) {
    return state.freq ^ state.pattern;
}

struct Command {
    static constexpr uint16_t VALID_HEADER = 0xAAAA;
    static constexpr uint16_t INVALID_HEADER = 0xFFFF;

    uint16_t start;

    MotorState left, right;

    BuzzerState buzzer;

    bool poweroff = false;
    bool led = false;

    uint16_t checksum;
};

inline uint16_t calculateChecksum(Command command) {
    return command.start ^
           calculateChecksum(command.left) ^
           calculateChecksum(command.right) ^
           calculateChecksum(command.buzzer) ^
           command.poweroff ^
           command.led;
}

struct MotorFeedback {
    int16_t   angle = 0;
    int16_t   speed = 0;
    uint8_t   error = 0;
    int16_t   dcLink = 0;
    int16_t   dcPhaA = 0;
    int16_t   dcPhaB = 0;
    int16_t   dcPhaC = 0;
    uint16_t  chops = 0;
    int16_t   id = 0;
    int16_t   iq = 0;
    bool      hallA = false;
    bool      hallB = false;
    bool      hallC = false;
};

inline uint16_t calculateChecksum(MotorFeedback feedback) {
    return feedback.angle ^ feedback.speed ^
           feedback.error ^ feedback.dcLink ^
           feedback.dcPhaA ^ feedback.dcPhaB ^
           feedback.dcPhaC ^ feedback.chops ^
           feedback.hallA ^ feedback.hallB ^ feedback.hallC;
}

struct Feedback {
    static constexpr uint16_t VALID_HEADER = 0xAAAA;
    static constexpr uint16_t INVALID_HEADER = 0xFFFF;

    uint16_t start;

    MotorFeedback left, right;

    int16_t   batVoltage = 0;
    int16_t   boardTemp = 0;

    int16_t timeoutCntSerial = 0;

    uint16_t checksum;
};

inline uint16_t calculateChecksum(Feedback feedback) {
    return feedback.start ^
           calculateChecksum(feedback.left) ^
           calculateChecksum(feedback.right) ^
           feedback.batVoltage ^
           feedback.boardTemp ^
           feedback.timeoutCntSerial;
}
}
}
}

#endif  /* INCLUDE_KISC_HOVERBOARD_PROTOCOL_INCLUDED */
