#!/usr/bin/env python3

# Resolve flight plan block names to block ids, and jump to them over
# the Ivy bus (ground JUMP_TO_BLOCK, relayed to the aircraft by the
# Paparazzi server). This is what lets the operator window run the
# motors/takeoff sequence without going back to the GCS

import logging
import urllib.request
import xml.etree.ElementTree as ET

from pprzlink.message import PprzMessage

logger = logging.getLogger(__name__)


MOTORS_CANDIDATES  = ('start motors', 'start engine', 'start engines', 'motors on')
TAKEOFF_CANDIDATES = ('takeoff', 'take off', 'take-off')
LAND_CANDIDATES    = ('land here', 'land', 'landing')

class FlightPlanBlocks:
    """The block name -> id table of one aircraft's flight plan.

    Parsed from the generated flight plan xml advertised in the CONFIG
    message, where the <block> document order matches the onboard
    block ids (same table the GCS uses for its own block buttons).
    """
    def __init__(self, conf):
        self.names = []          # index = block id
        try:
            with urllib.request.urlopen(conf.flight_plan) as f:
                tree = ET.parse(f)
            self.names = [b.get('name', '') for b in tree.iter('block')]
            logger.debug(f'aircraft {conf.id}: flight plan blocks {self.names}')
        except Exception as e:
            logger.warning(f'aircraft {conf.id}: cannot read flight plan '
                           f'{conf.flight_plan} ({e}); block jumps disabled')

    def find(self, candidates):
        """Block id of the first candidate name in the plan, else None."""
        lowered = [n.lower() for n in self.names]
        for cand in candidates:
            if cand in lowered:
                return lowered.index(cand)
        return None


def jump_to_block(ivy, ac_id, block_id):
    msg = PprzMessage('ground', 'JUMP_TO_BLOCK')
    msg['ac_id'] = str(ac_id)
    msg['block_id'] = block_id
    ivy.send(msg)
