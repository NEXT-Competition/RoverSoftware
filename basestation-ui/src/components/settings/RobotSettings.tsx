// Robot tab: the selected robot's tunable parameters, grouped.
//
// The config is FETCHED, not pushed — it is ~2.4 KB over a radio shared with
// telemetry, so it arrives only when this tab asks for it (on open, and on the
// Refresh button). Until then the page says so rather than rendering a form
// full of zeroes that would look like real values.

import { useEffect, useState } from "preact/hooks";
import { conn, robotConfigs, robots } from "../../net/ws.ts";
import { ROBOT_GROUPS } from "../../settings/schema.ts";
import {
  configTarget,
  refreshRobotConfig,
  targetRobot,
} from "../../state/settings.ts";
import { SettingField } from "./Field.tsx";

function GroupCard(
  { title, blurb, children, open, onToggle }: {
    title: string;
    blurb?: string;
    children: preact.ComponentChildren;
    open: boolean;
    onToggle: () => void;
  },
) {
  return (
    <section class={`group${open ? " open" : ""}`}>
      <button type="button" class="group-head" onClick={onToggle}>
        <span class="group-title">{title}</span>
        <span class="chevron" aria-hidden="true" />
      </button>
      {open && (
        <div class="group-body">
          {blurb && <p class="group-blurb">{blurb}</p>}
          {children}
        </div>
      )}
    </section>
  );
}

export function RobotSettings() {
  const rid = targetRobot.value;
  const entry = rid ? robotConfigs.value[rid] : undefined;
  const rejected = entry?.result?.rejected ?? {};
  const restart = new Set(entry?.result?.restart ?? []);
  const saveError = entry?.result?.save_error ?? null;
  // Only the first group starts open: eleven expanded groups is a wall of
  // sliders, and the ones you want are rarely at the top.
  const [open, setOpen] = useState<Record<string, boolean>>({
    [ROBOT_GROUPS[0].title]: true,
  });

  // Ask once per robot when the tab mounts, the target changes, or the socket
  // comes back. Guarded on having no cached config so revisiting the tab
  // doesn't spend radio airtime re-fetching something we already have.
  useEffect(() => {
    if (rid && !entry && conn.value === "live") refreshRobotConfig();
  }, [rid, conn.value]);

  if (!rid) {
    return <p class="hint pad">No robot selected — pick one on the driving view first.</p>;
  }

  return (
    <>
      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Robot</span>
          <select
            class="field-select"
            value={rid}
            onChange={(e) =>
              configTarget.value = (e.target as HTMLSelectElement).value}
          >
            {robots.value.map((r) => (
              <option key={r.robot_id} value={r.robot_id}>{r.robot_id}</option>
            ))}
          </select>
        </div>
        <button type="button" class="btn ghost small" onClick={refreshRobotConfig}>
          Refresh
        </button>
      </div>

      {restart.size > 0 && (
        <p class="banner warn">
          {restart.size} change{restart.size === 1 ? "" : "s"} saved but not
          live — restart the robot service to apply.
        </p>
      )}
      {saveError && (
        <p class="banner error">
          Applied, but not saved on the robot: {saveError}. The change is live
          now and will be lost on the next reboot.
        </p>
      )}

      {!entry
        ? (
          <p class="hint pad">
            Fetching {rid}'s configuration… If it stays empty, the robot is
            offline or running a build without live tuning.
          </p>
        )
        : ROBOT_GROUPS.map((group) => (
          <GroupCard
            key={group.title}
            title={group.title}
            blurb={group.blurb}
            open={!!open[group.title]}
            onToggle={() =>
              setOpen((prev) => ({ ...prev, [group.title]: !prev[group.title] }))}
          >
            {group.fields.map((field) => (
              <SettingField
                key={field.path}
                field={field}
                rejected={rejected[field.path]}
                restartPending={restart.has(field.path)}
              />
            ))}
          </GroupCard>
        ))}
    </>
  );
}
