package chipyard.iobinders

// Extracted from chipyard 7346121:
//   generators/chipyard/src/main/scala/iobinders/IOBinders.scala
// The original still lives in the backend until its pin moves to the plain
// tip; keep the two in step until then.

import chisel3._

class WithOspiPunchthrough extends OverrideIOBinder({
  (system: ospi.CanHavePeripheryOspi) => {
    val ports = system.ospi_sensor.map { sensorMV =>
      val sensor: ospi.SensorIO = sensorMV // unwrap the InModuleBody ModuleValue via the implicit
      val io_ospi = IO(chiselTypeOf(sensor)).suggestName("ospi_sensor")
      io_ospi <> sensor
      OspiPort(() => io_ospi)
    }.toSeq
    (ports, Nil)
  }
})
