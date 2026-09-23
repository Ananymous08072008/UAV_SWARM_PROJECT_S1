/* Dashboard client: WebSocket stream (with polling fallback), panels and operator actions. */
(function () {
  "use strict";

  var swarmMap = new SwarmMap("map");
  var network = new SwarmNetwork("network");
  var lastState = null;
  var selectedUav = null;
  var logSeen = 0;

  var IMPORTANT = ["POI_ASSIGNED", "POI_COMPLETED", "POI_ADDED", "POI_RELEASED", "RELAY_ASSIGNED",
    "RELAY_RELEASED", "UAV_FAILED", "UAV_DISCONNECTED", "UAV_RECONNECTED", "FAULT_DETECTED",
    "RECONFIGURATION", "RECOVERY_COMPLETE", "LINK_DEGRADED", "LINK_RESTORED", "OBSTACLE_ADDED",
    "OBSTACLE_REMOVED", "PREEMPTION", "RTH_STARTED", "HANDOVER_STARTED", "SAFETY_VIOLATION",
    "MISSION_RECALL", "TRIGGER_FIRED", "TRIGGER_REJECTED", "SIM_STARTED", "SIM_STOPPED"];

  function fmt(value, digits) {
    if (value === null || value === undefined) return "-";
    return typeof value === "number" ? value.toFixed(digits === undefined ? 2 : digits) : value;
  }
  function pct(value) { return value === null || value === undefined ? "-" : Math.round(value * 100) + "%"; }

  function setHeader(state) {
    document.getElementById("h-scenario").textContent = state.scenario.name;
    document.getElementById("h-mode").textContent = state.scenario.mode;
    document.getElementById("h-time").textContent =
      Math.round(state.world.t_s) + " / " + Math.round(state.world.duration_s) + " s";
    document.getElementById("h-pois").textContent =
      state.metrics.pois_completed + "/" + state.metrics.pois_total;
  }

  function setConnection(up) {
    var el = document.getElementById("h-conn");
    el.textContent = up ? "live" : "offline";
    el.className = "pill " + (up ? "up" : "down");
  }

  function renderMetrics(state) {
    var m = state.metrics, d = m.data || {};
    var cards = [
      ["PoIs done", m.pois_completed + "/" + m.pois_total],
      ["UAVs connected", pct(m.connectivity_ratio)],
      ["route PDR", fmt(m.mean_route_pdr)],
      ["latency", fmt(m.mean_latency_ms, 0) + " <small>ms</small>"],
      ["imagery live", pct(d.live_ratio)],
      ["delivered", fmt(d.delivered_mb, 0) + " <small>Mb</small>"],
      ["incidents", m.incidents + (m.incidents_open ? " <small>(" + m.incidents_open + " open)</small>" : "")],
      ["mean recovery", fmt(m.mean_recovery_time_s, 1) + " <small>s</small>"],
      ["relay changes", m.relay_changes],
      ["UAVs lost", m.uavs_lost],
      ["safety events", m.safety_violations]
    ];
    document.getElementById("metrics").innerHTML = cards.map(function (c) {
      return '<div class="metric"><div class="k">' + c[0] + '</div><div class="v">' + c[1] + "</div></div>";
    }).join("");
  }

  function renderTable(state) {
    var body = document.querySelector("#uav-table tbody");
    body.innerHTML = state.world.uavs.map(function (u) {
      var battClass = u.battery_pct < 20 ? "bad" : (u.battery_pct < 35 ? "warn" : "");
      var route = u.connected ? u.route.join(" > ") : "-";
      return '<tr data-uav="' + u.uav_id + '"' + (u.uav_id === selectedUav ? ' class="selected"' : "") + ">" +
        "<td>" + u.name + (u.health === "FAILED" ? " (lost)" : "") + "</td>" +
        '<td class="role ' + u.role + '">' + u.role + "</td>" +
        "<td>" + u.mode + "</td>" +
        "<td>" + (u.assigned_poi || "-") + "</td>" +
        '<td class="' + battClass + '">' + u.battery_pct.toFixed(0) + "</td>" +
        "<td>" + route + "</td>" +
        '<td class="' + (u.connected ? "" : "bad") + '">' + (u.connected ? u.pdr.toFixed(2) : "no link") + "</td>" +
        "<td>" + (u.latency_ms === null ? "-" : u.latency_ms.toFixed(0)) + "</td>" +
        "<td>" + u.z_m.toFixed(0) + "</td></tr>";
    }).join("");
    Array.prototype.forEach.call(body.querySelectorAll("tr"), function (row) {
      row.addEventListener("click", function () { select(parseInt(row.dataset.uav, 10)); });
    });
  }

  function renderIncidents(state) {
    var el = document.getElementById("incidents");
    var incidents = (state.swarm.incidents || []).slice(-4).reverse();
    el.innerHTML = incidents.map(function (i) {
      var cls = i.recovered_s !== null ? "ok" : (i.closed_reason ? "" : "open");
      var status = i.recovered_s !== null
        ? "recovered in " + fmt(i.recovery_time_s, 1) + " s"
        : (i.closed_reason || "in progress");
      return '<div class="incident ' + cls + '"><b>' + i.cause.replace(/_/g, " ") + "</b>" +
        (i.uav_id ? " (UAV-" + i.uav_id + ")" : "") + " at " + fmt(i.onset_s, 0) + " s - " +
        (i.detection_time_s !== null ? "detected in " + fmt(i.detection_time_s, 1) + " s, " : "") +
        status + "</div>";
    }).join("") || '<div class="incident ok">no faults yet</div>';
  }

  function appendEvents(events) {
    if (!events || !events.length) return;
    var log = document.getElementById("log");
    var importantOnly = document.getElementById("filter-important").checked;
    events.forEach(function (e) {
      if (e.seq <= logSeen) return;
      logSeen = e.seq;
      if (importantOnly && IMPORTANT.indexOf(e.type) === -1) return;
      var row = document.createElement("div");
      row.className = e.severity;
      row.innerHTML = '<span class="t">' + e.t_s.toFixed(1) + "s</span> " + e.type + " - " + e.message;
      log.appendChild(row);
    });
    while (log.childElementCount > 400) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
  }

  function select(uavId) {
    selectedUav = uavId;
    swarmMap.selected = uavId;
    var uav = lastState && lastState.world.uavs.filter(function (u) { return u.uav_id === uavId; })[0];
    document.getElementById("selected-uav").textContent = uav
      ? "selected: " + uav.name + " (" + uav.role + (uav.assigned_poi ? " -> " + uav.assigned_poi : "") + ")"
      : "no UAV selected";
    if (lastState) { renderTable(lastState); swarmMap.update(lastState); }
  }
  swarmMap.onSelect = select;

  function render(state) {
    lastState = state;
    setHeader(state);
    renderMetrics(state);
    renderTable(state);
    renderIncidents(state);
    swarmMap.update(state);
    network.update(state);
    document.getElementById("btn-download").classList
      .toggle("ready", !!(state.scenario && state.scenario.finished));
  }

  // ----------------------------------------------------------------- actions
  function inject(action, params) {
    return fetch("/api/inject", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: action, params: params || {} })
    }).then(function (r) { return r.json(); });
  }

  var obstacleCount = 0;
  var ACTIONS = {
    degrade: function () { inject("degrade_link", { uav_id: "critical_relay", quality: 0.15 }); },
    restore: function () {
      (lastState ? lastState.world.uavs : []).forEach(function (u) {
        if (u.radio_health < 1) inject("restore_link", { uav_id: u.uav_id });
      });
    },
    fail: function () { inject("fail_uav", { uav_id: selectedUav || "critical_relay", reason: "operator injected" }); },
    obstacle: function () {
      obstacleCount += 1;
      inject("add_obstacle", { id: "DEBRIS-OP" + obstacleCount, center_m: "backbone_midpoint",
                               size_m: 140, height_m: 70, attenuation_db: 40 });
    },
    "clear-obstacles": function () {
      ((lastState && lastState.env.obstacles) || []).forEach(function (o) { inject("remove_obstacle", { id: o.id }); });
    },
    "urgent-poi": function () {
      var area = lastState ? lastState.scenario.area_m : [0, 0, 500, 500];
      var x = Math.round(area[0] + (area[2] - area[0]) * (0.35 + 0.5 * Math.random()));
      var y = Math.round(area[1] + (area[3] - area[1]) * (0.35 + 0.5 * Math.random()));
      inject("add_poi", { id: "POI-OP" + Date.now().toString().slice(-4), position_m: [x, y],
                          priority: 5, survey_time_s: 60 });
    },
    drain: function () {
      if (!selectedUav) { alert("Select a UAV row first."); return; }
      inject("set_battery", { uav_id: selectedUav, battery_pct: 20 });
    }
  };
  Array.prototype.forEach.call(document.querySelectorAll(".controls button"), function (button) {
    button.addEventListener("click", function () { ACTIONS[button.dataset.action](); });
  });

  /* The run's logs and metrics as one zip. A hidden link rather than fetch():
     the response is a plain GET with an attachment disposition, so the browser
     saves it without leaving the page or buffering the archive as a blob. */
  document.getElementById("btn-download").addEventListener("click", function () {
    var link = document.createElement("a");
    link.href = "/api/export";
    link.download = "";
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  });

  // ------------------------------------------------------------- live stream
  function connect() {
    var url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
    var socket = new WebSocket(url);
    var ping = null;
    socket.onopen = function () {
      setConnection(true);
      ping = setInterval(function () { if (socket.readyState === 1) socket.send("ping"); }, 5000);
    };
    socket.onmessage = function (event) {
      var message = JSON.parse(event.data);
      if (message.type === "state") { render(message.state); appendEvents(message.events); }
    };
    socket.onclose = function () {
      setConnection(false);
      if (ping) clearInterval(ping);
      setTimeout(connect, 2000);
    };
    socket.onerror = function () { socket.close(); };
  }

  // one immediate REST fetch so the page is populated before the first push
  fetch("/api/state").then(function (r) { return r.json(); }).then(function (data) {
    if (data.state && data.state.world) { render(data.state); }
    return fetch("/api/events?since=0");
  }).then(function (r) { return r.json(); }).then(function (data) { appendEvents(data.events); })
    .catch(function () { /* the simulation may not have published yet */ });

  connect();
})();
