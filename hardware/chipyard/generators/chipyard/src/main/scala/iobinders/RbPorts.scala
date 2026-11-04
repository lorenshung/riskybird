package chipyard.iobinders

// Extracted from chipyard 7346121:
//   generators/chipyard/src/main/scala/iobinders/Ports.scala
// Compiled into Chipyard by the overlay plugin, hardware/chipyard/project/RiskyBird.scala.

case class OspiPort        (val getIO: () => ospi.SensorIO)
    extends Port[ospi.SensorIO]
