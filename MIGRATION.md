# PCB-only migration draft

This branch starts from the authoritative riskybirdv3-layout revision
f2ef0775cd7f9e3b502e52b2644797742ad05169. Board designs are unchanged.

Active temporal model tooling moved to lorenshung/ModelBlaster on
migration/unified-draft-20260927 under riskybird/dronet_temporal/.
Architecture source belongs to the same draft branch of lorenshung/chipyard.
The camera image converter and historical FPGA/model reports moved to
lorenshung/riskybird-dev's migration/unified-draft-20260927 branch.

The historical docs retained here describe old workspaces. Historical Arty DMA
bitstreams remain retrievable from f2ef077; their hashes are recorded in the
superproject's docs/history/pcb-development/bitstreams.json. New release images
must be built and published with their complete source manifest.

This existing personal fork is the review destination. Creating a separate lab
PCB repository and changing the lab superproject are deferred until review.
