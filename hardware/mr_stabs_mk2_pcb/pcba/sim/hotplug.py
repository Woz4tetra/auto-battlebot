"""Hot-plug transient sim of VBATT in ngspice."""

import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = """* hotplug {name}
Vpack p 0 17.4
Rint p a 0.04
S1 a b ctl 0 sw
.model sw SW(Ron=1m Roff=1G Vt=0.5 Vh=0.1)
Vctl ctl 0 PWL(0 0 1u 0 1.001u 1)
Llead b c 0.4u
Rlead c vb 0.015
Ccer vb 0 6.1u
Rbleed vb 0 1k  ; starts the board discharged (otherwise the DC point charges it through Roff)
{bulk}
.tran 2n 60u
.control
run
meas tran vpk MAX v(vb)
wrdata {csv} v(vb)
.endc
.end
"""
BULK = "Rb vb nb 0.34\nLb nb nb2 3n\nCb nb2 0 100u"
out = {}
for name, bulk in (("ceramic_only", ""), ("with_bulk", BULK)):
    csv = os.path.join(HERE, f"hotplug_{name}.csv")
    deck = os.path.join(HERE, f"hotplug_{name}.cir")
    open(deck, "w").write(BASE.format(name=name, bulk=bulk, csv=csv))
    r = subprocess.run(["ngspice", "-b", deck], capture_output=True, text=True)
    line = next(ln for ln in r.stdout.splitlines() if ln.strip().startswith("vpk"))
    out[name] = round(float(line.split("=")[1].split()[0]), 2)
json.dump(out, open(os.path.join(HERE, "hotplug.json"), "w"), indent=1)
print(out)
