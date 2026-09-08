/* Shared browser side of rover-web.  Every page loads this; nothing else. */
"use strict";

const Rover = {
  /* One reconnecting WebSocket.  `onMessage` receives parsed robotd and brain
     lines, relayed verbatim by rover-web. */
  connect(onMessage) {
    const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
    let socket = null;
    let backoff = 500;

    const open = () => {
      socket = new WebSocket(url);
      socket.onopen = () => {
        backoff = 500;
        Rover.setDot("ws", true);
      };
      socket.onclose = () => {
        socket = null;
        Rover.setDot("ws", false);
        setTimeout(open, backoff);
        backoff = Math.min(backoff * 2, 5000);
      };
      socket.onerror = () => socket && socket.close();
      socket.onmessage = (event) => {
        let message;
        try {
          message = JSON.parse(event.data);
        } catch {
          return;
        }
        if (onMessage) onMessage(message);
      };
    };
    open();

    return (payload) => {
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
        return true;
      }
      return false;
    };
  },

  setDot(name, up) {
    const el = document.querySelector(`[data-dot="${name}"]`);
    if (el) el.dataset.up = up ? "true" : "false";
  },

  /* The STOP form posts on its own.  This only adds feedback and keeps the
     page from navigating away; if it throws, the plain form still submits. */
  wireStop() {
    const form = document.querySelector("form.stop");
    if (!form) return;
    const note = form.querySelector(".note");
    form.addEventListener("submit", (event) => {
      event.preventDefault();
      if (note) note.textContent = "sending e-stop...";
      fetch("/estop", { method: "POST" })
        .then((r) => r.json())
        .then((body) => {
          if (note) {
            note.textContent = body.delivered
              ? "e-stop sent to robotd"
              : "robotd is not connected - use the inline switch";
            note.className = body.delivered ? "note" : "note warn";
          }
        })
        .catch(() => {
          if (note) {
            note.textContent = "rover-web unreachable - use the inline switch";
            note.className = "note warn";
          }
        });
    });
  },

  /* The stop-flag bit table, from /status rather than a copy here.  Resolves
     to {} if rover-web cannot be reached, so callers never wait on it. */
  stopFlagTable() {
    return fetch("/status")
      .then((r) => r.json())
      .then((s) => s.stop_flags || {})
      .catch(() => ({}));
  },

  /* Names of the set bits of `state.rover.stop_flags`, in bit order. */
  flagNames(mask, table) {
    return Object.keys(table)
      .filter((name) => (mask & table[name]) !== 0)
      .sort((a, b) => table[a] - table[b]);
  },

  /* The state strip every page shows: what the controller last reported, from
     the robotd `state` line, never interpreted beyond formatting.  Returns an
     update function; feed it each `state` message. */
  stateReadout(el) {
    if (!el) return () => {};
    let table = {};
    Rover.stopFlagTable().then((t) => {
      table = t;
    });
    const fmt = (value, digits, unit) =>
      value === null || value === undefined ? "-" : value.toFixed(digits) + unit;
    return (state) => {
      const rover = state.rover || {};
      const flags = Rover.flagNames(rover.stop_flags || 0, table);
      el.textContent = [
        "hdg " + fmt(rover.heading_deg, 0, "°"),
        "fw " + (rover.fw || "stock"),
        "hb " + (rover.hb_ok ? "ok" : "LOST"),
        "flags " + (flags.length ? flags.join("|") : "-"),
        "front " + (state.front_m === null || state.front_m === undefined ? "none" : fmt(state.front_m, 2, " m")),
        "bat " + fmt((state.battery || {}).pack_v, 1, " V") + " " + ((state.battery || {}).pct ?? "-") + "%",
        "budget " + fmt((state.budget || {}).motion_s, 1, " s"),
        state.ready ? "ready" : "not ready" + (state.reason ? " (" + state.reason + ")" : ""),
      ].join(" · ");
    };
  },

  /* Recovery panel for the clearable faults.  Every name it can offer is a
     ClearableFault robotd accepts through POST /clear: estop_sw from the
     state's own flag, low_battery from the controller's stop-flag bit, and
     obstacle_latched from robotd's not-ready reason.  A clear never re-arms.

     Returns an update function; feed it each robotd `state` line. */
  faultPanel(el) {
    if (!el) return () => {};
    let table = null;
    let shown = "";
    Rover.stopFlagTable().then((t) => {
      table = t;
    });

    const send = (names, note) => {
      note.textContent = "clearing " + names.join(", ") + "...";
      fetch("/clear", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ faults: names }),
      })
        .then((r) => r.json())
        .then((body) => {
          note.textContent = body.delivered
            ? "clear sent for " + body.faults.join(", ")
            : "robotd is not connected";
          note.className = body.delivered ? "note" : "note warn";
        })
        .catch(() => {
          note.textContent = "rover-web unreachable";
          note.className = "note warn";
        });
    };

    return (state) => {
      if (table === null) return;
      const names = [];
      if (state.estop_sw) names.push("estop_sw");
      const mask = (state.rover && state.rover.stop_flags) || 0;
      if (table.lowbat && (mask & table.lowbat) !== 0) names.push("low_battery");
      if (!state.ready && /obstacle/.test(state.reason || "")) names.push("obstacle_latched");
      const key = names.join(",");
      if (key === shown) return;
      shown = key;
      el.textContent = "";
      el.hidden = names.length === 0;
      if (names.length === 0) return;

      const title = document.createElement("p");
      title.className = "note";
      title.textContent = "latched — motion is refused until cleared";
      el.appendChild(title);
      const note = document.createElement("p");
      note.className = "note";

      for (const name of names) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "plain";
        button.textContent = "clear " + name;
        button.addEventListener("click", () => send([name], note));
        el.appendChild(button);
      }
      if (names.length > 1) {
        const all = document.createElement("button");
        all.type = "button";
        all.className = "plain";
        all.textContent = "clear all";
        all.addEventListener("click", () => send(names.slice(), note));
        el.appendChild(all);
      }
      el.appendChild(note);
    };
  },

  /* Poll /status so a page shows a dead robotd or brain rather than looking
     healthy while nothing is listening. */
  pollStatus(periodMs = 2000) {
    const tick = () =>
      fetch("/status")
        .then((r) => r.json())
        .then((s) => {
          Rover.setDot("robotd", s.robotd);
          Rover.setDot("brain", s.brain);
          Rover.setDot("teleop", s.teleop);
        })
        .catch(() => {
          Rover.setDot("robotd", false);
          Rover.setDot("brain", false);
        });
    tick();
    setInterval(tick, periodMs);
  },

  /* Screen Wake Lock, so the face page on a phone does not dim mid-turn.
     Convenience only; nothing safety-related depends on it. */
  keepAwake() {
    if (!("wakeLock" in navigator)) return;
    let lock = null;
    const acquire = () =>
      navigator.wakeLock
        .request("screen")
        .then((l) => {
          lock = l;
          l.addEventListener("release", () => {
            lock = null;
          });
        })
        .catch(() => {});
    acquire();
    document.addEventListener("visibilitychange", () => {
      if (document.visibilityState === "visible" && lock === null) acquire();
    });
  },

  appendLog(el, text, keep = 60) {
    if (!el) return;
    el.textContent += text + "\n";
    const lines = el.textContent.split("\n");
    if (lines.length > keep) el.textContent = lines.slice(-keep).join("\n");
    el.scrollTop = el.scrollHeight;
  },
};
