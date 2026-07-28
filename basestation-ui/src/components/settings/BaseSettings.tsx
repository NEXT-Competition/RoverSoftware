// Base station tab: settings that belong to this console, not to any robot.
//
// Everything here is local — no radio round trip — so it applies the moment it
// is changed and persists to the base station's settings file.

import { settingsResult } from "../../net/ws.ts";
import { BASE_GROUPS } from "../../settings/schema.ts";
import { SettingField } from "./Field.tsx";

export function BaseSettings() {
  const result = settingsResult.value;
  const rejected = result?.rejected ?? {};
  const restart = new Set(result?.restart ?? []);
  const saveError = result?.save_error ?? null;

  return (
    <>
      {saveError && (
        <p class="banner error">
          Applied, but not saved: {saveError}. The change is live now and will
          be lost when the base station restarts.
        </p>
      )}
      {restart.size > 0 && (
        <p class="banner warn">
          Saved, but the running service keeps its current value until it is
          restarted.
        </p>
      )}

      {BASE_GROUPS.map((group) => (
        <section class="group open" key={group.title}>
          <div class="group-head static">
            <span class="group-title">{group.title}</span>
          </div>
          <div class="group-body">
            {group.blurb && <p class="group-blurb">{group.blurb}</p>}
            {group.fields.map((field) => (
              <SettingField
                key={field.path}
                field={field}
                rejected={rejected[field.path]}
                restartPending={restart.has(field.path)}
              />
            ))}
          </div>
        </section>
      ))}
    </>
  );
}
