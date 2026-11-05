// RiskyBird's sbt overlay for a plain Chipyard checkout.
//
// tools/backend-patches links this file into backends/chipyard/project/ (see
// backends/overlays). An AutoPlugin there is triggered for every project of
// Chipyard's build and adds the RiskyBird sources under hardware/ to the
// projects named below, without touching build.sbt. Its settings are applied
// before each project's own, so the stock definitions keep working on top of
// them (a plain .sbt drop-in does not get that ordering, and `set` in
// Chipyard's make flow rebuilds the session without settings appended at load).
//
// Packages and class names are the ones the RiskyBird Chipyard fork used, so
// configs keep their fully qualified names
// (chipyard.fpga.arty200t.RocketArty200TDroneFullDDRConfig, ...).
//
// An overlay root may also SHADOW a stock source file: an overlay file whose
// path relative to its root matches a stock file of that project replaces it.
// The fork reworked a few stock files in place (the Arty100T harness, binders
// and configs, and fpga-shells' Arty100T shell and MIG, for the TE0712-100T);
// those are the only shadows. `sbt riskybirdShadows` lists them.

import sbt._
import sbt.Keys._

object RiskyBird extends AutoPlugin {
  override def trigger = allRequirements
  override def requires = plugins.JvmPlugin

  // project id -> (stock source root relative to Chipyard, overlay roots and
  // excluded paths relative to the RiskyBird checkout)
  case class Overlay(stock: String, roots: Seq[String], excluded: Seq[String] = Nil)

  val overlays: Map[String, Overlay] = Map(
    "chipyard" -> Overlay("generators/chipyard/src/main/scala", Seq(
      "hardware/chipyard/generators/chipyard/src/main/scala",
      "hardware/gemmini/chipyard",
      "hardware/rtl/esplink",
      "hardware/rtl/uartdma",
      "hardware/rtl/ospi/src",
      "hardware/rtl/ospi/chipyard",
      "hardware/rtl/fcrocc",
    ), Seq(
      "hardware/rtl/ospi/src/infra/Elaborate.scala", // standalone mill entry point
      "hardware/rtl/fcrocc/prototype",               // superseded prototype, not built
      "hardware/rtl/fcrocc/sim",
      "hardware/rtl/fcrocc/asm",
    )),
    "chipyard_fpga" -> Overlay("fpga/src/main/scala", Seq("hardware/chipyard/fpga/src/main/scala")),
    "fpga_shells" -> Overlay("fpga/fpga-shells/src/main/scala", Seq("hardware/fpga-shells/src/main/scala")),
    "gemmini" -> Overlay("generators/gemmini/src/main/scala", Seq("hardware/gemmini/src/main/scala")),
    "firechip" -> Overlay("generators/firechip/chip/src/main/scala",
      Seq("hardware/chipyard/generators/firechip/chip/src/main/scala")),
  )

  // FcRoCC's blackbox resources: hardware/rtl/fcrocc/vsrc/* -> /vsrc/* on the classpath.
  val resourceRoot = "hardware/rtl/fcrocc"
  val resourceKeep = "hardware/rtl/fcrocc/vsrc"

  // This file is a symlink to <root>/hardware/chipyard/project/RiskyBird.scala.
  def root(chipyard: File): File =
    (chipyard / "project" / "RiskyBird.scala").toPath.toRealPath()
      .getParent.getParent.getParent.getParent.toFile

  def shadowed(chipyard: File, o: Overlay): Seq[File] = {
    val stock = chipyard / o.stock
    val rb = root(chipyard)
    (stock ** "*.scala").get.filter { f =>
      val rel = stock.toPath.relativize(f.toPath)
      o.roots.exists(r => (rb / r).toPath.resolve(rel).toFile.isFile)
    }
  }

  object autoImport {
    val riskybirdShadows = taskKey[Seq[File]]("stock Chipyard sources replaced by a RiskyBird overlay file")
  }
  import autoImport._

  private def overlayOf = Def.setting(overlays.get(thisProject.value.id))

  override def projectSettings: Seq[Def.Setting[_]] = Seq(
    Compile / unmanagedSourceDirectories ++= {
      val rb = root((ThisBuild / baseDirectory).value)
      overlayOf.value.toSeq.flatMap(_.roots.map(rb / _))
    },
    Compile / unmanagedSources := {
      val sources = (Compile / unmanagedSources).value
      overlayOf.value match {
        case None => sources
        case Some(o) =>
          val cy = (ThisBuild / baseDirectory).value
          val rb = root(cy)
          val gone = shadowed(cy, o).map(_.getCanonicalFile).toSet
          val excluded = o.excluded.map(e => (rb / e).toPath)
          sources.filterNot { f =>
            val p = f.getAbsoluteFile.toPath
            gone(f.getCanonicalFile) || excluded.exists(p.startsWith)
          }
      }
    },
    Compile / unmanagedResourceDirectories ++= {
      if (thisProject.value.id == "chipyard") Seq(root((ThisBuild / baseDirectory).value) / resourceRoot)
      else Nil
    },
    Compile / unmanagedResources := {
      val resources = (Compile / unmanagedResources).value
      if (thisProject.value.id != "chipyard") resources
      else {
        val rb = root((ThisBuild / baseDirectory).value)
        val fc = (rb / resourceRoot).toPath
        val keep = (rb / resourceKeep).toPath
        resources.filter { f =>
          val p = f.getAbsoluteFile.toPath
          !p.startsWith(fc) || p.startsWith(keep)
        }
      }
    },
    riskybirdShadows := {
      val cy = (ThisBuild / baseDirectory).value
      overlayOf.value.toSeq.flatMap(o => shadowed(cy, o))
    },
  )
}
