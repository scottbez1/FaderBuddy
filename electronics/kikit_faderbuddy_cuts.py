"""
KiKit cuts plugin for the fader_buddy_main panel.

The panel has a vertical backbone between its two columns, and
`"vbonecut": true` is needed so the backbone is perforated where it meets the
top and bottom rails -- otherwise the spine stays attached to the rails and
neither can be snapped off. KiKit, however, builds the backbone as one segment
per row and cuts *every* segment end, which also perforates the spine at each
row boundary and leaves it far less rigid than intended.

This plugin renders the cuts exactly like the built-in "mousebites" type, but
first drops the backbone cuts that aren't at the very top or bottom of the
spine.

Used from fader_buddy_main-kikit_panelize.json as:

    "cuts": {
        "type": "plugin",
        "code": "electronics/kikit_faderbuddy_cuts.py.FaderBuddyCuts",
        ...
    }
"""

from kikit.common import fromMm
from kikit.plugin import CutsPlugin

# Cut endpoints come straight out of KiKit's geometry, so they land on exact
# values; this is just guarding against integer rounding.
TOL = fromMm(0.01)


class FaderBuddyCuts(CutsPlugin):
    def renderTabCuts(self, panel, cuts):
        # Tab cuts honour the configured offset, same as the built-in type
        self._mousebites(panel, cuts, self.preset["cuts"]["offset"])

    def renderOtherCuts(self, panel, cuts):
        # Frame and backbone cuts ignore the offset, same as the built-in type
        self._mousebites(panel, self._dropInnerBackboneCuts(cuts), 0)

    def _mousebites(self, panel, cuts, offset):
        properties = self.preset["cuts"]
        panel.makeMouseBites(
            list(cuts),
            properties["drill"],
            properties["spacing"],
            offset,
            properties["prolong"],
        )

    def _dropInnerBackboneCuts(self, cuts):
        """
        Keep only the topmost and bottommost backbone cuts (where the spine
        meets the rails), and every non-backbone cut untouched.
        """
        cuts = list(cuts)
        backbone = [c for c in cuts if self._isBackboneCut(c)]
        if not backbone:
            return cuts
        ys = [c.coords[0][1] for c in backbone]
        top, bottom = min(ys), max(ys)
        return [
            c
            for c in cuts
            if not self._isBackboneCut(c)
            or abs(c.coords[0][1] - top) < TOL
            or abs(c.coords[0][1] - bottom) < TOL
        ]

    def _isBackboneCut(self, cut):
        """
        A backbone cut runs horizontally across the spine, so it is exactly
        vbackbone long. The frame cuts are the width of a rail (plus its
        spacing), so they don't collide with this as long as the two differ.
        """
        width = self.preset["layout"]["vbackbone"]
        (x1, y1), (x2, y2) = cut.coords[0], cut.coords[-1]
        return abs(y1 - y2) < TOL and abs(abs(x1 - x2) - width) < TOL
