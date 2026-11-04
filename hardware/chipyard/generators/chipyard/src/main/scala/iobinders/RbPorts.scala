package chipyard.iobinders

// Extracted from chipyard 7346121:
//   generators/chipyard/src/main/scala/iobinders/Ports.scala
// The original still lives in the backend until its pin moves to the plain
// tip; keep the two in step until then.

case class OspiPort        (val getIO: () => ospi.SensorIO)
    extends Port[ospi.SensorIO]
