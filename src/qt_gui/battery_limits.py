#!/usr/bin/env python3
#
# Per-drone battery thresholds, read from each aircraft's Paparazzi airframe
#
#   <section name="BAT">
#     <define name="LOW_BAT_LEVEL"    value="10.5" unit="V"/>   <!-- plan to land -->
#     <define name="CRITIC_BAT_LEVEL" value="9.9"  unit="V"/>   <!-- land now -->
#   </section>
#
import logging
import urllib.request
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)

# Fallback, used when the airframe is unreachable or defines no BAT section.
# Gens Ace Soaring 2700mAh 3S (11.1V nominal, 12.6V full).
DEFAULT_LOW_V  = 3.5 * 3   # 10.5V: plan to land
DEFAULT_CRIT_V = 3.3 * 3   # 9.9V: land now


FULL_CELL_V = 4.2          # a LiPo cell, fully charged
DEFAULT_CELLS = 3          # the lab packs are 3S


class BatteryLimits:
    """Land-soon ('low') and land-now ('crit') pack voltages for one drone,
    plus its cell count so voltages can be shown per cell -- 4.2 V full,
    3.5 V low reads the same whatever the pack size."""

    def __init__(self, low=DEFAULT_LOW_V, crit=DEFAULT_CRIT_V,
                 cells=DEFAULT_CELLS, source='default'):
        self.low, self.crit, self.source = float(low), float(crit), source
        self.cells = max(1, int(cells))

    def per_cell(self, v):
        """Pack voltage -> volts per cell (None stays None)."""
        return None if v is None else v / self.cells

    @property
    def low_per_cell(self):
        return self.low / self.cells

    @property
    def crit_per_cell(self):
        return self.crit / self.cells

    def state(self, v):
        """Classify a pack voltage: 'unknown' | 'bad' | 'warn' | 'ok'."""
        if v is None:
            return 'unknown'
        if v < self.crit:
            return 'bad'
        if v < self.low:
            return 'warn'
        return 'ok'

    def __repr__(self):
        return (f'BatteryLimits(low={self.low:.1f}V, crit={self.crit:.1f}V, '
                f'from {self.source})')


def _bat_defines(airframe_url):
    """{DEFINE_NAME: float} of the airframe's <section name="BAT">."""
    with urllib.request.urlopen(airframe_url) as f:
        tree = ET.parse(f)
    out = {}
    for section in tree.iter('section'):
        if (section.get('name') or '').upper() != 'BAT':
            continue
        for d in section.iter('define'):
            name, value = d.get('name'), d.get('value')
            if not name or value is None:
                continue
            try:
                out[name.upper()] = float(value)
            except ValueError:
                pass          # non-numeric define (an expression): ignore
    return out


def from_airframe(conf):
    """Battery limits of the aircraft described by `conf` (a PprzConfig),
    read from its airframe file. Always returns usable limits: any problem
    (no airframe, unreadable, no BAT section, inconsistent values) falls back
    to the defaults with a warning."""
    ac_id = getattr(conf, 'id', '?')
    url = getattr(conf, 'airframe', None)
    if not url:
        logger.warning(f'aircraft {ac_id}: no airframe in its config; '
                       'using default battery thresholds')
        return BatteryLimits()
    try:
        defines = _bat_defines(url)
    except Exception as e:
        logger.warning(f'aircraft {ac_id}: cannot read airframe {url} ({e}); '
                       'using default battery thresholds')
        return BatteryLimits()

    low, crit = defines.get('LOW_BAT_LEVEL'), defines.get('CRITIC_BAT_LEVEL')
    if low is None and crit is None:
        logger.warning(f'aircraft {ac_id}: no BAT thresholds in the airframe; '
                       'using default battery thresholds')
        return BatteryLimits()
    low = DEFAULT_LOW_V if low is None else low
    crit = DEFAULT_CRIT_V if crit is None else crit
    if not (0 < crit < low):
        logger.warning(f'aircraft {ac_id}: inconsistent BAT thresholds in the '
                       f'airframe (low={low}, critic={crit}); using defaults')
        return BatteryLimits()
    maxv = defines.get('MAX_BAT_LEVEL')
    cells = round(maxv / FULL_CELL_V) if maxv else round(low / 3.5)
    cells = min(max(int(cells), 1), 12)
    limits = BatteryLimits(low, crit, cells, source='airframe')
    logger.info(f'aircraft {ac_id}: battery thresholds {limits}')
    return limits
