"""Commanding the fleet in words — by voice, by typing, or by another AI.

Three faces, one throat. Speech recognition (stt.py), a local language model
(llm.py) and the MCP server (basestation/mcp_server.py) all converge on the same
`CommandExecutor` (executor.py), which turns an *intent* into the exact action
dicts `basestation/app.py::handle_action` already accepts.

That convergence is the whole design, and the reason is safety rather than
tidiness. A 4-billion-parameter model running on a base station in a noisy pit
WILL occasionally hallucinate. It must not be able to invent a command — only to
choose one from a list the dashboard could already have issued, addressed to a
robot that exists, with a place that has been saved. Everything the model emits
is validated against live fleet state before a byte reaches the radio.

Three rules the rest of this package exists to enforce:

  1. **E-stop never goes through the model.** fastpath.py matches "stop" and its
     synonyms on the raw transcript, before the LLM is asked anything. A model
     that is slow, wrong, or not running at all cannot delay a stop.
  2. **The executor whitelists.** An intent whose name is not in INTENTS, or
     whose robot is not in the fleet, is refused — it is never passed through
     hopefully. See intents.py.
  3. **Dangerous verbs need a human.** Firing, arming, jogging and raw driving
     return a pending confirmation instead of executing; a person taps it. See
     `Authority` in intents.py.
"""

from .executor import CommandExecutor, Outcome
from .intents import INTENTS, Authority, IntentError, plan
from .vocabulary import Vocabulary

__all__ = [
    "CommandExecutor",
    "Outcome",
    "INTENTS",
    "Authority",
    "IntentError",
    "plan",
    "Vocabulary",
]
