// Optional browser DevTools snippet for the existing rocket GCS.
// Open workswithoutcmd copy.html, connect serial, stop simulation, then run this
// file in DevTools Sources > Snippets. It adds no controls or layout changes.
// Connect GCS serial to the wireless receiver, not the rocket for radio control.
// Examples (one at a time, wait for DONE/FAIL after the acceptance ACK):
//   servoCommands.spin()         // legacy unsupported positional command (ERR)
//   servoCommands.stop()         // legacy unsupported positional command (ERR)
//   servoCommands.reverse()      // legacy unsupported positional command (ERR)
//   servoCommands.test(4)        // PWM4,TEST90: nominal90deg
//   servoCommands.test(5)        // PWM5,TEST90
//   servoCommands.pulse(4, 1200) // firmware checks its configured pulse limits
//   servoCommands.safe(4)        // PWM4,SAFE: configured90deg
//   servoCommands.status()       // STATUS,SERVO
// Existing connection/simulation/pending-command guards and ACK handling apply.
// Acknowledgement establishes commanded output, not physical movement.
(() => {
  'use strict';
  if (typeof VEHICLE === 'undefined' || VEHICLE.ADDRESS !== '23:R' ||
      typeof sendFlightCommand !== 'function') {
    throw new Error('Run this snippet in the existing rocket GCS page.');
  }
  function command(channel, argument) {
    if (![2,3,4,5].includes(channel)) throw new TypeError('Servo channel must be 2, 3, 4 or 5.');
    return sendFlightCommand('PWM' + channel, argument);
  }
  window.servoCommands = Object.freeze({
    spin() { return command(4, 'SPIN'); },
    stop() { return command(4, 'STOP'); },
    reverse() { return command(4, 'REVERSE'); },
    test(channel) { return command(channel, 'TEST90'); },
    safe(channel) { return command(channel, 'SAFE'); },
    pulse(channel, microseconds) {
      if (!Number.isInteger(microseconds) || microseconds < 0 || microseconds > 0xFFFFFFFF)
        throw new TypeError('Pulse must be an unsigned integer in microseconds.');
      return command(channel, String(microseconds));
    },
    status() { return sendFlightCommand('STATUS', 'SERVO'); }
  });
})();
