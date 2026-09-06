PMW3901 optical-flow diagnostic
###############################

The application uses a target-supplied ``riskybird,pmw3901`` devicetree
contract for its SPI bus, software chip select, reset, and LED control. It
initializes the sensor and streams motion bursts.

.. code-block:: console

   ./rb build pmw3901_test --target esp32c6
   ./rb run pmw3901_test --target esp32c6 --port /dev/ttyACM0

FPGA software compilation selects the logical DroneLogic peripheral set:

.. code-block:: console

   ./rb build pmw3901_test --target arty100t \
     --config RocketArty100TDroneLogicConfig

The FPGA SPI/GPIO package pins are not yet routed or verified, so physical
FPGA execution remains disabled.
