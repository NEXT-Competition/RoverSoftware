"""Commands recognised by keyword, before the language model is asked anything.

Two reasons this exists, and only the first one matters.

**Stopping must not depend on a model.** LM Studio may be closed, the machine may
be swapping, the model may be mid-download, the prompt may take 800 ms. None of
that can be allowed to sit between an operator saying "stop" and a rover
stopping. So the transcript is matched against a small set of literal phrases
first, and a hit dispatches immediately — no HTTP request, no JSON parsing, no
network at all. If the model is down, stop is the one thing that still works.

**The common cases shouldn't cost a round trip.** "rover two", "teleop", "show me
the camera" are most of what actually gets said during a match, they are
unambiguous, and recognising them locally makes the interface feel immediate in
a way a 700 ms model hop never will. The model is for the sentences that need
understanding — "send rover2 to bucket A then come back" — not for the ones that
are effectively buttons.

--- Why this is a whitelist of phrases and not a parser ---
Every rule here must be *certain*. A fast path that fires on a maybe is worse
than no fast path, because it silently bypasses the model that would have got it
right. Anything that isn't a confident match returns None and falls through.
"""

from __future__ import annotations

import re
from typing import Any, Dict, Optional, Tuple

from .vocabulary import Vocabulary, normalise

# Phrases that mean STOP. Matched as whole phrases against the normalised
# transcript, and deliberately generous: a false stop costs a match, a missed
# stop costs a rover. Whisper's usual manglings ("stopped", "stop it") are here
# because they are what a shouted word actually transcribes as.
_STOP_PHRASES = (
    "stop", "stop it", "stopped", "stop stop", "stop now", "all stop",
    "halt", "hold", "hold it", "freeze", "abort", "kill it", "cut it",
    "e stop", "estop", "emergency stop", "emergency", "shut it down",
)

# ...and the ones that mean stop EVERYTHING, not just the selected rover.
_STOP_ALL = ("all stop", "stop all", "stop everyone", "stop everything",
             "stop all rovers", "everybody stop", "all rovers stop",
             "emergency stop all", "kill everything", "shut it down")

# Releasing a latch is not urgent, but it sits next to stop in an operator's
# head and it is unambiguous, so it goes here too.
_CLEAR_PHRASES = ("clear", "clear e stop", "clear estop", "clear the e stop",
                  "clear the stop", "release", "reset the stop", "unstop",
                  "clear it", "resume")

# Lead-ins an operator puts in front of a bare rover name or mode: "switch to
# rover two", "put it in waypoint mode". Stripped by longest-first prefix match
# rather than by an optional regex group — an optional group backtracks into
# matching nothing, which quietly turns "switch to rover 3" into a lookup for
# the literal phrase "switch to rover 3". Longest first so "switch to" is
# consumed before "switch".
_LEAD_INS = tuple(sorted((
    "switch to", "switch", "select", "go to", "go", "talk to", "use", "control",
    "take", "change to", "change", "put it in", "put it into", "put it",
    "put", "set it to", "set to", "set", "into", "in", "to", "give me",
    "i want", "let me have",
), key=len, reverse=True))

# Trailing filler that means nothing: "waypoint mode", "rover two please".
_TRAILERS = ("mode", "please", "now", "thanks", "ok", "okay")

# Camera. The one command nobody phrases the same way twice.
_CAMERA_RE = re.compile(
    r"^(?:(?:pull\s+up|show|open|bring\s+up|give\s+me|let\s+me\s+see)\s+)"
    r"(?:me\s+)?(?:the\s+|its\s+)?"
    r"(?:(.+?)(?:'?s)?\s+)?"
    r"(?:camera|fpv|feed|video|view)\s*$"
)
_CAMERA_BARE = ("camera", "fpv", "camera feed", "fpv feed", "video", "the camera")

# Verbs that mean "run this routine". Longest first, same reason as _LEAD_INS.
# Deliberately narrow: these are the words that only ever precede the name of
# something to run, so "run collect" is certain in a way "do collect" is not.
_RUN_VERBS = tuple(sorted((
    "run", "start", "execute", "launch", "begin", "play", "perform",
), key=len, reverse=True))


def match(text: str, vocab: Vocabulary) -> Optional[Tuple[str, Dict[str, Any]]]:
    """Recognise a command without a model, or return None to fall through.

    Returns `(intent_name, args)` in exactly the shape the model would have
    produced, so the executor cannot tell — and must not care — which path a
    command arrived by.
    """
    said = normalise(text)
    if not said:
        return None

    # 1. STOP. First, unconditionally, before anything else can match.
    if said in _STOP_ALL:
        return "estop", {"robot": "all"}
    if said in _STOP_PHRASES:
        return "estop", {}
    # A stop with a rover named: "stop rover2", "rover 2 stop".
    for phrase in ("stop", "halt", "abort", "e stop", "estop", "emergency stop"):
        for candidate in (_strip_prefix(said, phrase), _strip_suffix(said, phrase)):
            if candidate is not None:
                rid = vocab.resolve_robot(candidate)
                if rid:
                    return "estop", {"robot": candidate}

    if said in _CLEAR_PHRASES:
        return "clear_estop", {}

    # 2. Camera, before the bare-word rules below can eat "camera".
    if said in _CAMERA_BARE:
        return "show_camera", {}
    cam = _CAMERA_RE.match(said)
    if cam:
        who = (cam.group(1) or "").strip()
        # "show me the camera" leaves who="", meaning the selected rover. A word
        # that isn't a rover ("show me the front camera") is not a confident
        # match, so fall through to the model rather than guessing which rover.
        if not who:
            return "show_camera", {}
        if vocab.resolve_robot(who):
            return "show_camera", {"robot": who}
        return None

    # 3. "run the collect routine". A verb the operator only ever says before
    #    the name of something to run, plus a name that resolves against the
    #    routines actually loaded on the rover being addressed — nothing else
    #    can match this shape, so it does not need a model to disambiguate it.
    #
    #    Before the rover and mode rules: "start" is not a rover or a mode, and
    #    a routine somebody named "waypoint" should still be reachable by the
    #    sentence that names it explicitly.
    named = _routine_named(said, vocab)
    if named:
        return "start_routine", named

    # 4. A bare rover name, or "switch to <rover>". Checked before modes so a
    #    fleet with a rover named "auto" doesn't lose its own name to an alias.
    #
    #    Both the whole phrase and the lead-in-stripped one are tried, because a
    #    rover could legitimately be called "go" and stripping unconditionally
    #    would make it unaddressable.
    for candidate in _candidates(said):
        if vocab.resolve_robot(candidate):
            return "select_robot", {"robot": candidate}

    # 5. A bare mode name. Only when the phrase resolves to a mode on its own —
    #    "align to the bucket" has an object in it and belongs to the model.
    for candidate in _candidates(said):
        if vocab.resolve_mode(candidate):
            return "set_mode", {"mode": candidate}

    # 6. A routine's bare name, last of all, so it can never take a word off a
    #    rover or a mode. A routine the operator programmed and named is a
    #    button on their screen; saying its name should press it. Starting one
    #    still needs a tap to confirm (intents.py), which is what makes it safe
    #    to recognise a single word this way.
    if vocab.resolve_routine(vocab.selected, said):
        return "start_routine", {"routine": said}

    return None


def _routine_named(said: str, vocab: Vocabulary) -> Optional[Dict[str, Any]]:
    """The args for "run the collect routine", or None if that isn't what it is.

    Carries what was SAID, not the resolved id: the executor re-resolves it
    against the rover the intent finally lands on. It also carries the rover
    whenever one was named, because an autonomous routine started on the wrong
    machine is the worst thing this file could produce.

    The rover goes before the verb — "rover2 run collect" — because "run
    collect on rover2" puts a rover where a place name goes, and telling those
    two apart is exactly the job the model exists for.
    """
    for rest, robot in _robot_prefixes(said, vocab):
        for verb in _RUN_VERBS:
            body = _strip_prefix(rest, verb)
            if body is None:
                continue
            for article in ("the ", "a ", "an ", "my "):
                if body.startswith(article):
                    body = body[len(article):].strip()
                    break
            if not body:
                return None  # "run the routine" — which one? Ask the model.
            if vocab.resolve_routine(robot or vocab.selected, body):
                return {"routine": body, **({"robot": robot} if robot else {})}
    return None


def _robot_prefixes(said: str, vocab: Vocabulary) -> Tuple[Tuple[str, Optional[str]], ...]:
    """`(remainder, robot)` for the phrase as-is, and with a leading rover name
    stripped off it. Literal first, so a rover is only assumed when the plain
    reading found nothing."""
    out: list = [(said, None)]
    words = said.split()
    for take in (2, 1):  # "rover 2 run collect", then "rover2 run collect"
        if len(words) > take:
            rid = vocab.resolve_robot(" ".join(words[:take]))
            if rid and rid != "*":
                out.append((" ".join(words[take:]), rid))
                break
    return tuple(out)


def _candidates(said: str) -> Tuple[str, ...]:
    """The phrase, and what's left of it once filler is removed.

    Returned in order of decreasing literalness so an exact name always beats a
    stripped one, and de-duplicated so a phrase with no filler isn't looked up
    twice.
    """
    out = [said]
    body = said
    for lead in _LEAD_INS:
        if body.startswith(lead + " "):
            body = body[len(lead):].strip()
            break
    for trailer in _TRAILERS:
        if body.endswith(" " + trailer):
            body = body[: -len(trailer)].strip()
            break
    # Strip a leading article last: "the camera" is handled above, but "go to
    # the waypoint mode" needs it gone after the lead-in.
    for article in ("the ", "a ", "an ", "its ", "it "):
        if body.startswith(article):
            body = body[len(article):].strip()
            break
    if body and body not in out:
        out.append(body)
    return tuple(out)


def _strip_prefix(said: str, word: str) -> Optional[str]:
    return said[len(word):].strip() if said.startswith(word + " ") else None


def _strip_suffix(said: str, word: str) -> Optional[str]:
    return said[: -len(word)].strip() if said.endswith(" " + word) else None


def is_stop(text: str) -> bool:
    """Does this transcript contain a stop, anywhere in it?

    Used as a last-ditch check on transcripts the fast path did not fully match
    — "uh, stop, stop the rover" is not one of the phrases above, but it is
    unmistakably a stop and must not be sent to a model to think about.
    """
    said = normalise(text)
    return any(re.search(rf"\b{re.escape(p)}\b", said) for p in
               ("stop", "halt", "abort", "estop", "emergency", "freeze"))
