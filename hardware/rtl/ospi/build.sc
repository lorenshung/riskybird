import mill._
import mill.scalalib._

// Standalone mill build for the OSPI HM01B0 camera-capture RTL.
// Versions intentionally match Chipyard (chisel 6.7.0 / scala 2.13.16 / chiseltest 6.0.0)
// so the move to sbt at Chipyard-integration time is clean.
object ospi extends ScalaModule {
  def scalaVersion = "2.13.16"

  // Use the repo root as the module root: sources in ./src, tests in ./test/src
  def millSourcePath = super.millSourcePath / os.up

  def ivyDeps = Agg(
    ivy"org.chipsalliance::chisel:6.7.0"
  )
  def scalacPluginIvyDeps = Agg(
    ivy"org.chipsalliance:::chisel-plugin:6.7.0"
  )
  def scalacOptions = Seq("-Ymacro-annotations", "-deprecation", "-feature")

  object test extends ScalaTests with TestModule.ScalaTest {
    def ivyDeps = Agg(
      ivy"edu.berkeley.cs::chiseltest:6.0.0"
    )
    def scalacPluginIvyDeps = Agg(
      ivy"org.chipsalliance:::chisel-plugin:6.7.0"
    )
    def scalacOptions = Seq("-Ymacro-annotations")
  }
}
