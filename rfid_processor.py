"""
rfid_processor.py — RFID tag dispatcher
========================================
Consumes tags from state.rfid_queue and forwards them to the SequenceEngine.

All sequence behaviour (what happens when a tag is read) is now defined in the
profile JSON under "sequences".  No Python changes are needed to add or modify
sequences — edit the profile, restart.

To add a new action type (e.g. "activate_gripper"), add one method to
core/sequence_engine.py and register it there.
"""

import logging

import config

logger = logging.getLogger(__name__)


async def rfid_processor(state, engine):
    """Thin dispatcher: reads tags from the queue and passes to the engine.

    Args:
        state:  AMRState shared object
        engine: SequenceEngine instance (core/sequence_engine.py)
    """
    logger.info("[RFID Processor] Started.")
    while True:
        tag = await state.rfid_queue.get()
        state.latest_rfid_tag = tag
        # Homepost presence tracking (drives DO_AT_HOME / gates DI_CALL_START).
        # Reading the homepost tag while NOT driving (parked, pushed there in
        # MANUAL, etc.) marks the AGV at homepost; while RUNNING the
        # arrive_homepost sequence's go_home action sets it instead, so the
        # tag being read again as the AGV drives OFF homepost doesn't re-set
        # it. Any other tag means the AGV is somewhere else.
        if tag == config.HOMEPOST_RFID_TAG:
            if state.current_mode not in ("running", "reversing") and not state.at_homepost:
                state.at_homepost = True
                logger.info("[RFID] Homepost tag read while stopped — at_homepost = True")
                if state.active_branch is not None:
                    logger.info("[RFID] Branch %s cleared at homepost", state.active_branch)
                    state.active_branch = None
        else:
            if state.at_homepost:
                state.at_homepost = False
                logger.info("[RFID] Tag %s read — at_homepost = False", tag)
            if state.leaving_home:
                state.leaving_home = False
                logger.info("[RFID] Tag %s read — left homepost, arrive_homepost re-armed", tag)
        logger.debug("[RFID] Tag read: %s", tag)
        await engine.on_rfid_tag(tag)
