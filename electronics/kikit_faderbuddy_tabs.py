"""
KiKit tabs plugin for the fader_buddy_main panel.

KiKit's built-in "fixed" tabs are spread evenly along each board edge, which
doesn't work here: the FaderBuddy outline has only a few narrow windows where a
mousebite cut won't collide with a via, tooling hole, LED or test pad. This
plugin places tabs at explicit, hand-picked positions instead.

Used from fader_buddy_main-kikit_panelize.json as:

    "tabs": {
        "type": "plugin",
        "code": "electronics/kikit_faderbuddy_tabs.py.FaderBuddyTabs",
        ...
    }

Positions are offsets in mm from the board's top-left corner (the untranslated
board outline spans x 53..112, y 41.5..58.5).
"""

from kikit.annotations import TabAnnotation
from kikit.common import (
    fromMm,
    shpBBoxBottom,
    shpBBoxLeft,
    shpBBoxRight,
    shpBBoxTop,
)
from kikit.panelize_ui_impl import dummyFramingSubstrate
from kikit.plugin import TabsPlugin
from kikit.substrate import SubstrateNeighbors

# Tabs on the left/right (17mm) edges, as an offset in mm from the board's top
# edge. A single centered tab: the straight part of the edge runs y 43.5..56.5
# (2mm corner radii), and at y=50 the cut stays clear of the H1/H2 tooling holes
# and of the two GND vias near the right edge.
SIDE_TAB_OFFSETS_MM = [8.5]

# Tabs on the top/bottom (59mm) edges, joining the rows of the panel, as an
# offset in mm from the board's left edge. These are the only two windows along
# those edges with no copper or holes close to the outline:
#   x=60.0 (offset 7.0)  -- under the "Install fader here" silkscreen, clear 55.7..64.5
#   x=92.2 (offset 39.2) -- under the 5-pin headers, clear 85.0..100.6
# Everything in between is blocked by D1, D2, TP2, TP3, J4/J5 mounting pads and
# a via sitting 0.5mm from the edge.
ROW_TAB_OFFSETS_MM = [7.0, 39.2]


class FaderBuddyTabs(TabsPlugin):
    def buildTabAnnotations(self, panel):
        hwidth = self.preset["tabs"]["hwidth"]
        vwidth = self.preset["tabs"]["vwidth"]

        # The frame doesn't exist yet when tabs are built, so stand-in
        # substrates are needed for the boards facing it -- same trick KiKit's
        # own tab builders use.
        ghosts = dummyFramingSubstrate(panel.substrates, self.preset)
        neighbors = SubstrateNeighbors(panel.substrates + ghosts)

        sides = [
            (neighbors.leftC, shpBBoxLeft, (1, 0), SIDE_TAB_OFFSETS_MM, hwidth),
            (neighbors.rightC, shpBBoxRight, (-1, 0), SIDE_TAB_OFFSETS_MM, hwidth),
            (neighbors.topC, shpBBoxTop, (0, 1), ROW_TAB_OFFSETS_MM, vwidth),
            (neighbors.bottomC, shpBBoxBottom, (0, -1), ROW_TAB_OFFSETS_MM, vwidth),
        ]

        for substrate in panel.substrates:
            for query, edgeOf, direction, offsets, width in sides:
                edge = edgeOf(substrate.bounds())
                for neighbor, shadow in query(substrate):
                    for offset in offsets:
                        pos = edge.min + fromMm(offset)
                        # Only place a tab where the neighbour actually faces
                        # this part of the edge
                        if not any(s.min <= pos <= s.max for s in shadow.intervals):
                            continue
                        origin = (edge.x, pos) if direction[0] else (pos, edge.x)
                        substrate.annotations.append(
                            TabAnnotation(None, origin, direction, width))
