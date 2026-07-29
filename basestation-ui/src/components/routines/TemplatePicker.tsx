// The empty state: four working routines to open, instead of a blank grid.
//
// This is the first thing a competitor sees in this tab, and "No routines yet"
// with a New button was a dead end — it explains how to make a box, not what a
// routine is for. Each card here opens a complete, runnable example that
// demonstrates one idea, and every one of them is parsed by the real robot
// validator in tests/test_routine_templates.py, so none of them can greet
// someone with a wall of errors on a document they didn't write.

import { TEMPLATES } from "../../routines/templates.ts";
import { addRoutine, addRoutineFromTemplate } from "../../state/routines.ts";

export function TemplatePicker() {
  return (
    <div class="templates">
      <div class="templates-head">
        <span class="eyebrow">Start from</span>
        <p class="templates-blurb">
          Each of these is a complete routine you can run and then edit. They
          are copied when you open them, so changing one never affects the next.
        </p>
      </div>

      <div class="template-grid">
        {TEMPLATES.map((t) => (
          <button
            key={t.key}
            type="button"
            class="template-card"
            onClick={() => addRoutineFromTemplate(t.spec)}
          >
            <span class="template-title">{t.title}</span>
            <span class="template-blurb">{t.blurb}</span>
            <span class="template-foot">
              <span class="template-steps">
                {t.spec.states?.length ?? 0} steps
              </span>
              {/* The one thing a template cannot guess is where your field is,
                  so it says so rather than shipping someone else's coordinates. */}
              {t.needs && <span class="template-needs">{t.needs}</span>}
            </span>
          </button>
        ))}
      </div>

      <button type="button" class="btn ghost small" onClick={addRoutine}>
        Or start from an empty routine
      </button>
    </div>
  );
}
