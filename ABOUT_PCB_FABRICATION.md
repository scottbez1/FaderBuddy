## PCB Fabrication
The FaderBuddy PCB is designed for JLCPCB SMT assembly -- all surface-mount components are placed by the factory, but the through-hole daisy-chain pin headers are omitted by default. You'll need to order and solder the pin headers separately (or add them to your JLCPCB assembly order):

| Item | Qty | Notes |
|------|-----|-------|
| 5 pin right-angle male pin headers 0.1" spacing | 1 each | Included with Bezek Labs FaderBuddy boards, can be purchased separately from electronics suppliers like [LCSC - 40-pin break-apart headers](https://www.lcsc.com/product-detail/C429956.html)|
| 5 pin right-angle female pin headers 0.1" spacing | 1 each | Included with Bezek Labs FaderBuddy boards, can be purchased separately from electronics suppliers like [LCSC](https://www.lcsc.com/product-detail/C2935995.html) |


Use the files from the [latest release](https://github.com/scottbez1/FaderBuddy/releases) when ordering!

> [!CAUTION]
> The files below are auto-generated from the current (untested) design files, and are provided for design reference ONLY. They are NOT considered stable for manufacturing. Use the latest stable release linked above! 

Latest auto-generated (untested and likely broken!) artifacts⚠️:
- Review
  - [Schematic](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-schematic.pdf)
  - [Interactive BOM](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-ibom.html)
  - [PCB Packet](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-pcb-packet.pdf)
- Ordering (Configured for JLCPCB)
  - 1.6mm, any color, HASL lead-free
  - [Untested gerbers](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-jlc/gerbers.zip)
  - [Untested BOM csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-jlc/bom.csv)
  - [Untested CPL (POS) csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/fader_buddy_main-jlc/pos.csv)

### Assembly

The PCB comes fully assembled from Bezek Labs LLC and JLCPCB. The only soldering required is attaching the PCB to the fader itself and the optional daisy-chaining headers, which are all through-hole connections:

- **2 connections** for the motor
- **4 connections** for the fader potentiometer
- 2 optional mechanical-only connections (I recommend skipping these)
- **5 connections** for the female daisy-chaining pin headers
- **5 connections** for the male daisy-chaining pin headers

No fine-pitch or SMD soldering is required.