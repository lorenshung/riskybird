package chipyard.config

// Extracted from chipyard 7346121:
//   generators/chipyard/src/main/scala/config/fragments/PeripheralFragments.scala
// The original still lives in the backend until its pin moves to the plain
// tip; keep the two in step until then.

import org.chipsalliance.cde.config.{Config}

/**
  * Logical peripherals required by the RiskyBird drone sensor/actuator base.
  *
  * This deliberately does not bind FPGA package pins. Board harness binders
  * must be added only after the connector-to-package mapping and I/O voltage
  * compatibility have been verified.
  */
class WithRiskyBirdDronePeriphery extends Config(
  new WithPWM(address = 0x10051000) ++
  new WithPWM(address = 0x10050000) ++
  new WithGPIO(address = 0x10010000, width = 3) ++ // PMW3901 CS, reset, LED_N
  new WithSPI(address = 0x10031000))
