package chipyard.iobinders

// Extracted from chipyard 7346121:
//   generators/chipyard/src/main/scala/iobinders/IOBinders.scala
// Compiled into Chipyard by the overlay plugin, hardware/chipyard/project/RiskyBird.scala.

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
