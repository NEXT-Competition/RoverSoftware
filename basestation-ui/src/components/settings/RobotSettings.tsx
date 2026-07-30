// Robot tab: the selected robot's tunable parameters, grouped.
//
// The config is FETCHED, not pushed — it is ~2.4 KB, so it arrives only when
// this tab asks for it (on open, and on the Refresh button). Until then the
// page says so rather than rendering a form full of zeroes that would look like
// real values. It travels over the robot's WiFi link and never over the radio,
// so a rover out of WiFi range reports that instead (see the banner below);
// driving and telemetry are unaffected by any of this.

import { useState } from "preact/hooks";
import { robotConfigs, robotDocuments, robots } from "../../net/ws.ts";
import { type Group, robotGroupsFor } from "../../settings/schema.ts";
import { PidGraph } from "./PidGraph.tsx";

/**
 * The tuning path of the PID loop this group owns, or null.
 *
 * Found from the fields themselves — a group with an `<x>.kp` in it is a group
 * about the loop `<x>` — rather than from a table of group titles. A title is
 * prose somebody will reword; the gain path is the same string the robot names
 * its trace with (robot/control/waypoint.py::pid_traces), so matching on it is
 * what guarantees the graph and the knobs beneath it are the same loop.
 */
function loopIn(group: Group): string | null {
  const gain = group.fields.find((f) => f.path.endsWith(".kp"));
  return gain ? gain.path.slice(0, -3) : null;
}
import { useRadioFetch } from "../../state/fetch.ts";
import {
  configTarget,
  refreshRobotConfig,
  refreshRobotFields,
  targetRobot,
} from "../../state/settings.ts";
import { SettingField } from "./Field.tsx";
import { Waiting } from "./Waiting.tsx";

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
  // Not "the robot is offline" — it may well be driving. Config does not travel
  // over the radio, so this is specifically "no WiFi link to it".
  const unreachable = entry?.result?.error ?? null;
  // The groups this robot actually has. A stock build's are the hand-written
  // ones; a build running its own layout describes its actuators and those
  // groups are generated from the description. See settings/schema.ts.
  const documents = rid ? robotDocuments.value[rid] : undefined;
  const groups = robotGroupsFor(documents?.fields);
  // Only the first group starts open: eleven expanded groups is a wall of
  // sliders, and the ones you want are rarely at the top.
  const [open, setOpen] = useState<Record<string, boolean>>({
    [groups[0].title]: true,
  });

  // Ask per robot when the tab mounts, the target changes, or the socket comes
  // back. Guarded on having no cached config so revisiting the tab doesn't
  // spend radio airtime re-fetching something we already have, and retried a
  // couple of times because the snapshot arrives in pieces and a piece lost to
  // a busy radio otherwise leaves this page blank for good. The field
  // descriptors ride along: without them a custom layout's actuators would
  // arrive as values with nothing to render them.
  const fetch = useRadioFetch(rid && `${rid}:config`, !!entry, refreshRobotConfig);
  useRadioFetch(
    rid && `${rid}:fields`,
    !!documents?.fields_rev,
    refreshRobotFields,
  );

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

      {unreachable && <p class="banner error">{unreachable}</p>}
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
          <Waiting
            what="configuration"
            robot={rid}
            fetch={fetch}
            note={
              <p class="hint">
                A build without live tuning also answers nothing here.
              </p>
            }
          />
        )
        : groups.map((group) => (
          <GroupCard
            key={group.title}
            title={group.title}
            blurb={group.blurb}
            open={!!open[group.title]}
            onToggle={() =>
              setOpen((prev) => ({ ...prev, [group.title]: !prev[group.title] }))}
          >
            {/* A group that owns a closed loop leads with what that loop is
                doing. Above the gains rather than below them: you read the
                curve, then reach for the knob — and the two cannot be about
                different loops, because the graph is identified by the same
                path prefix the gain fields carry. */}
            {loopIn(group) && (
              <PidGraph
                robotId={rid}
                loop={loopIn(group)!}
                unit={loopIn(group)!.startsWith("nav.") ? "°" : ""}
              />
            )}
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
