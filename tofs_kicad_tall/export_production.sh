#!/usr/bin/env bash
# JLC production export for the VL53L5CX ToF breakout (2-layer: F.Cu/B.Cu).
# Output -> production/cli_<git-short-hash>/ AND promoted to top-level production/:
#   <name>_gerbers.zip   gerbers + PTH/NPTH excellon + job file
#   <name>_bom.csv       Comment,Designator,Footprint,Quantity,MPN,LCSC Part #
#   <name>_cpl.csv       Designator,Mid X,Mid Y,Layer,Rotation
# BOM/CPL exclude DNP + "exclude from BOM/pos" parts (e.g. the breakaway tab).
set -uo pipefail
cd "$(dirname "$0")"
NAME=VL53L5CX_ToF_Breakout_Tall
HASH=$(git rev-parse --short HEAD 2>/dev/null || echo nohash)
OUT=production/cli_$HASH
rm -rf "$OUT"; mkdir -p "$OUT/gerbers"
echo "== export dir: $OUT (HEAD $HASH)"

echo "== ERC / DRC (informational; non-fatal)"
kicad-cli sch erc --severity-error --format json -o "$OUT/erc.json" $NAME.kicad_sch 2>&1 | grep -i "found\|error" || true
kicad-cli pcb drc --schematic-parity --severity-error --format json -o "$OUT/drc.json" $NAME.kicad_pcb 2>&1 | grep -i "found\|error" || true

echo "== Gerbers (2-layer) + drill"
kicad-cli pcb export gerbers --check-zones --no-protel-ext -o "$OUT/gerbers" \
  -l F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,F.Paste,B.Paste,Edge.Cuts \
  $NAME.kicad_pcb >/dev/null || { echo "GERBER EXPORT FAILED"; exit 1; }
kicad-cli pcb export drill -o "$OUT/gerbers/" --format excellon --excellon-units mm \
  --drill-origin absolute --excellon-zeros-format decimal --excellon-separate-th \
  $NAME.kicad_pcb >/dev/null || { echo "DRILL EXPORT FAILED"; exit 1; }
(cd "$OUT/gerbers" && zip -q "../${NAME}_gerbers.zip" ./*)

echo "== BOM + CPL"
kicad-cli sch export bom -o "$OUT/${NAME}_bom.csv" --exclude-dnp \
  --fields 'Value,Reference,Footprint,${QUANTITY},MPN,LCSC Part' \
  --labels 'Comment,Designator,Footprint,Quantity,MPN,LCSC Part #' \
  --group-by 'Footprint,LCSC Part' --ref-range-delimiter '' $NAME.kicad_sch >/dev/null \
  || { echo "BOM EXPORT FAILED"; exit 1; }
kicad-cli pcb export pos -o "$OUT/pos-raw.csv" --format csv --units mm --side both --exclude-dnp \
  $NAME.kicad_pcb >/dev/null || { echo "POS EXPORT FAILED"; exit 1; }
python3 - "$OUT" "$NAME" <<'EOF'
import csv, sys
out, name = sys.argv[1:]
bom = list(csv.DictReader(open(f'{out}/{name}_bom.csv')))
for row in bom:
    if ',' in row['Comment']: row['Comment'] = row['MPN']
with open(f'{out}/{name}_bom.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(bom[0]), quoting=csv.QUOTE_ALL)
    w.writeheader(); w.writerows(bom)
refs = {r for row in bom for r in row['Designator'].split(',')}
nolcsc = [row['Designator'] for row in bom if not row['LCSC Part #']]
pos = [r for r in csv.DictReader(open(f'{out}/pos-raw.csv')) if r['Ref'] in refs]
with open(f'{out}/{name}_cpl.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['Designator','Mid X','Mid Y','Layer','Rotation'])
    for r in pos: w.writerow([r['Ref'], r['PosX'], r['PosY'], r['Side'].capitalize(), r['Rot']])
missing = refs - {r['Ref'] for r in pos}
print(f"BOM: {len(bom)} lines, {len(refs)} placed parts; CPL: {len(pos)} rows")
if nolcsc: print("WARNING: BOM lines without an LCSC part:", nolcsc)
if missing: print("WARNING: in BOM but not in CPL:", sorted(missing))
EOF
rm -f "$OUT/pos-raw.csv"

cp "$OUT/${NAME}_gerbers.zip" "production/${NAME}-gerbers.zip"
cp "$OUT/${NAME}_bom.csv"     "production/${NAME}-bom.csv"
cp "$OUT/${NAME}_cpl.csv"     "production/${NAME}-cpl.csv"
rm -rf production/gerbers && cp -r "$OUT/gerbers" production/gerbers
echo "== promoted to canonical: production/${NAME}-{gerbers.zip,bom.csv,cpl.csv} (from $OUT)"
echo "== done. files in $OUT:"; ls -1 "$OUT"
