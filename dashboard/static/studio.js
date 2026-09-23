/* Mission studio controller.

   Two views on one page: the builder (design a mission) and the live view
   (watch and drive it). Every request is scoped to a session id, so two people
   on the same server never touch each other's world.

   The websocket carries three message types:
     state  - snapshot + new events + control status
     status - control status only (the simulation is paused or idle)
     reset  - the session restarted; drop the log and the charts and resync */
(function () {
  "use strict";

  var QUIET = ["UAV_COMMANDED", "ROUTE_CHANGED", "POI_SURVEY_STARTED",
               "UAV_ARRIVED", "POI_CREATED", "UAV_SPAWNED"];

  var limits = null, builder = null, swarmMap = null, network = null, charts = null;
  var sessionId = null, socket = null, lastState = null, selectedUav = null;
  var reconnectTimer = null;

  function $(id) { return document.getElementById(id); }
  function fmt(v, d) { return v === null || v === undefined ? "-" : Number(v).toFixed(d === undefined ? 2 : d); }
  function pct(v) { return v === null || v === undefined ? "-" : Math.round(v * 100) + "%"; }

  /* ----------------------------------------------------------------- builder */
  function initBuilder() {
    return fetch("/api/limits").then(function (r) { return r.json(); }).then(function (data) {
      limits = data;
      $("uav-count").max = limits.max_uavs;
      $("uav-count").min = limits.min_uavs;
      $("duration").min = limits.min_duration_s;
      $("duration").max = limits.max_duration_s;

      builder = new BuilderMap("builder-map", limits);
      builder.onChange = renderPoiList;
      builder.onNotify = function (msg) {
        $("builder-note").textContent = msg;
        setTimeout(function () { $("builder-note").textContent = ""; }, 3000);
      };
      builder.invalidate();
      renderPoiList([]);
      refreshSessions();
      setInterval(refreshSessions, 5000);
    });
  }

  function renderPoiList(pois) {
    var host = $("poi-list");
    $("poi-count").textContent = pois.length + " placed";
    if (!pois.length) {
      host.innerHTML = '<p class="note">No PoIs yet. Click the map to place one, ' +
                       'or launch with the standard six.</p>';
      return;
    }
    host.innerHTML = pois.map(function (p, i) {
      return '<div class="poi-row">' +
        '<b>' + p.id + '</b>' +
        '<span class="coords">' + p.x_m + ', ' + p.y_m + ' m</span>' +
        '<select data-poi="' + i + '" class="prio">' +
          [1, 2, 3, 4, 5].map(function (n) {
            return '<option value="' + n + '"' + (n === p.priority ? " selected" : "") +
                   ">p" + n + "</option>";
          }).join("") +
        "</select>" +
        '<button data-remove="' + i + '" class="tiny">x</button>' +
      "</div>";
    }).join("");
    Array.prototype.forEach.call(host.querySelectorAll(".prio"), function (sel) {
      sel.addEventListener("change", function () {
        builder.setPriority(parseInt(sel.dataset.poi, 10), parseInt(sel.value, 10));
      });
    });
    Array.prototype.forEach.call(host.querySelectorAll("[data-remove]"), function (btn) {
      btn.addEventListener("click", function () {
        builder.remove(parseInt(btn.dataset.remove, 10));
      });
    });
  }

  function refreshSessions() {
    if (!$("builder-view") || $("builder-view").hidden) return;
    fetch("/api/sessions").then(function (r) { return r.json(); }).then(function (data) {
      $("capacity").textContent = data.sessions.length + " / " + data.capacity;
      var host = $("session-list");
      if (!data.sessions.length) {
        host.innerHTML = '<p class="note">Nothing running yet.</p>';
        return;
      }
      host.innerHTML = data.sessions.map(function (s) {
        return '<div class="session-row">' +
          '<span class="pill ' + s.state + '">' + s.state + "</span>" +
          "<b>" + s.scenario + "</b>" +
          '<span class="hint">' + s.uav_count + " UAVs, " + s.poi_count + " PoIs, t=" +
            fmt(s.t_s, 0) + "s</span>" +
          '<button data-join="' + s.id + '" class="tiny">watch</button>' +
        "</div>";
      }).join("");
      Array.prototype.forEach.call(host.querySelectorAll("[data-join]"), function (btn) {
        btn.addEventListener("click", function () { openSession(btn.dataset.join); });
      });
    }).catch(function () { /* server restarting; the next tick retries */ });
  }

  function launch() {
    var spec = {
      name: "mission",
      uav_count: parseInt($("uav-count").value, 10),
      duration_s: parseFloat($("duration").value),
      speed: parseFloat($("speed").value),
      seed: parseInt($("seed").value, 10),
      mode: $("mode").value,
      faults: $("faults").checked
    };
    if (builder.pois.length) spec.pois = builder.pois;
    $("launch-error").textContent = "";
    $("launch").disabled = true;
    fetch("/api/sessions", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(spec)
    }).then(function (r) {
      return r.json().then(function (body) {
        if (!r.ok) throw new Error(body.detail || "could not start the mission");
        return body;
      });
    }).then(function (status) {
      openSession(status.id);
    }).catch(function (err) {
      $("launch-error").textContent = err.message;
    }).then(function () { $("launch").disabled = false; });
  }

  /* --------------------------------------------------------------- live view */
  function openSession(id) {
    sessionId = id;
    $("builder-view").hidden = true;
    $("live-view").hidden = false;
    $("live-session").textContent = "session " + id;
    // Put the session in the URL so it can be shared with somebody else or
    // survive a page refresh - several people watching one mission is the point.
    if (window.history && history.replaceState) {
      history.replaceState(null, "", "?session=" + id);
    }
    if (!swarmMap) {
      swarmMap = new SwarmMap("map");
      swarmMap.onSelect = select;
      network = new SwarmNetwork("network");
      // Joining a mission where every UAV has already landed gives a very tight
      // bounding box; without a ceiling, fit() zooms until the labels fill the
      // panel. Cap it and let the graph open out as the swarm spreads.
      network.cy.maxZoom(2.5);
      charts = new ChartBank("charts");
      bindLiveControls();
    }
    charts.reset();
    network.fitted = false;          // re-fit for this session's world
    setTimeout(function () { network.cy.resize(); network.fitted = false; }, 80);
    $("log").innerHTML = "";
    lastState = null;
    selectedUav = null;
    setTimeout(function () { swarmMap.map.invalidateSize(); }, 60);
    hydrate();
    connect();
  }

  /* One immediate REST read so the view is populated before the first push,
     instead of sitting blank for a frame. Same trick as the single-simulation
     dashboard. */
  function hydrate() {
    var id = sessionId;
    fetch("/api/sessions/" + id + "/state").then(function (r) { return r.json(); })
      .then(function (data) {
        if (id !== sessionId) return;                  // switched away while loading
        if (data.status) applyStatus(data.status);
        if (data.state && data.state.world) render(data.state);
        return fetch("/api/sessions/" + id + "/history?every=2");
      })
      .then(function (r) { return r ? r.json() : null; })
      .then(function (data) {
        if (data && id === sessionId) charts.seed(data.samples);
        return fetch("/api/sessions/" + id + "/events?since=0");
      })
      .then(function (r) { return r ? r.json() : null; })
      .then(function (data) { if (data && id === sessionId) appendEvents(data.events); })
      .catch(function (err) {
        // The simulation may simply not have published yet, but a render fault
        // lands here too - say so rather than leaving a silently blank page.
        console.error("initial load failed", err);
      });
  }

  function backToBuilder() {
    if (socket) { socket.onclose = null; socket.close(); socket = null; }
    clearTimeout(reconnectTimer);
    sessionId = null;
    $("live-view").hidden = true;
    $("builder-view").hidden = false;
    if (window.history && history.replaceState) history.replaceState(null, "", location.pathname);
    builder.invalidate();
    refreshSessions();
  }

  function connect() {
    if (socket) { socket.onclose = null; socket.close(); }
    var proto = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(proto + "://" + location.host + "/ws/" + sessionId);
    var ping = null;
    socket.onopen = function () {
      // Keeps a reverse proxy from dropping the socket as idle.
      ping = setInterval(function () { if (socket.readyState === 1) socket.send("ping"); }, 5000);
    };
    socket.onmessage = function (ev) {
      var msg = JSON.parse(ev.data);
      if (msg.type === "reset") {
        charts.reset();
        $("log").innerHTML = "";
        lastState = null;
        return;
      }
      if (msg.status) applyStatus(msg.status);
      if (msg.type === "state") {
        render(msg.state);
        appendEvents(msg.events || []);
      }
    };
    socket.onclose = function () {
      if (ping) clearInterval(ping);
      if (!sessionId) return;
      reconnectTimer = setTimeout(connect, 1500);   // survive a server restart
    };
    socket.onerror = function () { socket.close(); };
  }

  function applyStatus(status) {
    $("h-state").textContent = status.state;
    $("h-state").className = "pill " + status.state;
    $("h-mode").textContent = status.mode;
    $("h-scenario").textContent = status.scenario;
    $("h-time").textContent = fmt(status.t_s, 0) + " / " + fmt(status.duration_s, 0) + " s";
    $("btn-pause").textContent = status.state === "paused" ? "Resume" : "Pause";
    $("btn-pause").disabled = (status.state === "finished" || status.state === "error");
    // The run is over and the session will be reaped when idle, so draw attention
    // to the only thing that keeps its data.
    $("btn-download").classList.toggle("ready", status.state === "finished");
    if (String(parseInt($("live-speed").value, 10)) !== String(Math.round(status.speed))) {
      $("live-speed").value = Math.round(status.speed);
      $("live-speed-value").textContent = Math.round(status.speed) + "x";
    }
    if (status.error) $("builder-note").textContent = status.error;
  }

  function control(action, extra) {
    var body = { action: action };
    if (extra) Object.keys(extra).forEach(function (k) { body[k] = extra[k]; });
    return fetch("/api/sessions/" + sessionId + "/control", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body)
    }).then(function (r) {
      return r.json().then(function (b) {
        if (!r.ok) throw new Error(b.detail || action + " failed");
        return b;
      });
    }).then(applyStatus).catch(function (err) {
      $("selected-uav").textContent = err.message;
    });
  }

  /* Take the run away as a file: summary, timeseries, events and the mission
     spec. Studio sessions write nothing to disk and are reaped once idle, so
     this is the only way the data outlives the tab.

     A hidden link rather than fetch(): the response is a normal GET with an
     attachment disposition, so the browser saves it without leaving the page,
     and without holding the whole archive in memory as a blob first. */
  function downloadRun() {
    if (!sessionId) return;
    var link = document.createElement("a");
    link.href = "/api/sessions/" + sessionId + "/export";
    link.download = "";
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  }

  function bindLiveControls() {
    $("btn-pause").addEventListener("click", function () {
      control($("btn-pause").textContent === "Pause" ? "pause" : "resume");
    });
    $("btn-restart").addEventListener("click", function () { control("restart"); });
    $("btn-stop").addEventListener("click", function () { control("stop"); });
    $("btn-download").addEventListener("click", downloadRun);
    $("btn-new").addEventListener("click", backToBuilder);
    $("live-speed").addEventListener("input", function () {
      $("live-speed-value").textContent = $("live-speed").value + "x";
    });
    $("live-speed").addEventListener("change", function () {
      control("speed", { speed: parseFloat($("live-speed").value) });
    });

    var actions = {
      degrade: function () { inject("degrade_link", { uav_id: selectedUav || "critical_relay", quality: 0.15 }); },
      restore: function () { inject("restore_link", {}); },
      fail: function () { inject("fail_uav", { uav_id: selectedUav || "critical_relay", reason: "operator injected" }); },
      debris: function () { inject("add_obstacle", { id: "DEBRIS-OP", center_m: "backbone_midpoint", size_m: 140, height_m: 60 }); },
      clear: function () { inject("remove_obstacle", {}); },
      urgent: function () { inject("add_poi", { priority: 5, survey_time_s: 40 }); },
      drain: function () {
        if (!selectedUav) { $("selected-uav").textContent = "Select a UAV row first."; return; }
        inject("set_battery", { uav_id: selectedUav, battery_pct: 20 });
      }
    };
    Array.prototype.forEach.call(document.querySelectorAll(".control-panel [data-act]"), function (btn) {
      btn.addEventListener("click", function () { actions[btn.dataset.act](); });
    });
    $("important-only").addEventListener("change", function () {
      $("log").classList.toggle("show-all", !$("important-only").checked);
    });
  }

  function inject(action, params) {
    return fetch("/api/sessions/" + sessionId + "/inject", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: action, params: params })
    }).then(function (r) {
      if (!r.ok) return r.json().then(function (b) {
        $("selected-uav").textContent = b.detail || "action rejected";
      });
    });
  }

  /* ---------------------------------------------------------------- rendering */
  function renderMetrics(state) {
    var m = state.metrics, d = m.data || {};
    var cards = [
      ["PoIs done", m.pois_completed + " / " + m.pois_total, ""],
      ["UAVs connected", pct(m.connectivity_ratio), ""],
      ["Route PDR", fmt(m.mean_route_pdr), ""],
      ["Latency", fmt(m.mean_latency_ms, 0), "ms"],
      ["Imagery live", pct(d.live_ratio), ""],
      ["Delivered", fmt(d.delivered_mb, 0), "Mb"],
      ["Buffered", fmt(d.buffered_mb, 0), "Mb"],
      ["Lost", fmt(d.lost_mb, 1), "Mb"],
      ["Incidents", m.incidents + (m.incidents_open ? " (" + m.incidents_open + " open)" : ""), ""],
      ["Mean recovery", fmt(m.mean_recovery_time_s, 0), "s"],
      ["Relay changes", m.relay_changes, ""],
      ["UAVs lost", m.uavs_lost, ""],
      ["Safety events", m.safety_violations, ""],
      ["Net components", state.swarm.network ? state.swarm.network.components : "-", ""]
    ];
    // style.css styles .metric/.k/.v - not .card - so match it.
    $("metrics").innerHTML = cards.map(function (c) {
      var unit = c[2] ? ' <small>' + c[2] + "</small>" : "";
      return '<div class="metric"><div class="k">' + c[0] + "</div>" +
             '<div class="v">' + c[1] + unit + "</div></div>";
    }).join("");
  }

  function renderTable(state) {
    var body = document.querySelector("#uav-table tbody");
    body.innerHTML = state.world.uavs.map(function (u) {
      var route = u.connected ? u.route.join(" > ") : '<span class="bad">no link</span>';
      return '<tr data-uav="' + u.uav_id + '"' + (u.uav_id === selectedUav ? ' class="selected"' : "") + ">" +
        "<td>" + u.name + (u.health === "LOST" ? ' <span class="bad">(lost)</span>' : "") + "</td>" +
        '<td class="role ' + u.role + '">' + u.role + "</td>" +
        "<td>" + u.mode + "</td>" +
        "<td>" + (u.assigned_poi || "-") + "</td>" +
        "<td>" + fmt(u.battery_pct, 0) + "</td>" +
        "<td>" + route + "</td>" +
        "<td>" + fmt(u.pdr) + "</td>" +
        "<td>" + fmt(u.latency_ms, 0) + "</td>" +
        "<td>" + fmt(u.z_m, 0) + "</td>" +
        "<td>" + fmt(u.ground_speed_mps, 1) + "</td>" +
        "<td>" + fmt(u.heading_deg, 0) + "</td>" +
        "<td>" + fmt(u.radio_health, 2) + "</td>" +
      "</tr>";
    }).join("");
    Array.prototype.forEach.call(body.querySelectorAll("tr"), function (row) {
      row.addEventListener("click", function () { select(parseInt(row.dataset.uav, 10)); });
    });
  }

  function renderIncidents(state) {
    var incidents = (state.swarm.incidents || []).slice(-4).reverse();
    $("incidents").innerHTML = incidents.map(function (i) {
      var status = i.recovered_s !== null && i.recovered_s !== undefined
        ? "recovered in " + fmt(i.recovery_time_s, 0) + "s"
        : (i.detected_s !== null && i.detected_s !== undefined ? "in progress" : "undetected");
      return '<div class="incident' + (i.recovered_s === null || i.recovered_s === undefined ? " open" : "") + '">' +
        "<b>" + i.cause + "</b> " +
        (i.uav_id !== null && i.uav_id !== undefined ? "(UAV-" + String(i.uav_id).padStart(2, "0") + ") " : "") +
        "at " + fmt(i.onset_s, 0) + "s - detected in " + fmt(i.detection_time_s, 1) + "s, " + status +
      "</div>";
    }).join("");
  }

  function appendEvents(events) {
    var log = $("log");
    events.forEach(function (e) {
      var row = document.createElement("div");
      row.className = "line" + (QUIET.indexOf(e.type) >= 0 ? " quiet" : "");
      row.innerHTML = '<span class="t">' + fmt(e.t_s, 1) + "s</span> " +
                      '<span class="ty">' + e.type + "</span> " +
                      (e.detail || e.message || "");
      log.appendChild(row);
    });
    while (log.childElementCount > 400) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
  }

  function select(uavId) {
    selectedUav = selectedUav === uavId ? null : uavId;
    if (swarmMap) swarmMap.selected = selectedUav;
    var uav = lastState && lastState.world.uavs.filter(function (u) { return u.uav_id === selectedUav; })[0];
    $("selected-uav").textContent = uav
      ? uav.name + " - " + uav.role + " / " + uav.mode + ", battery " + fmt(uav.battery_pct, 0) +
        "%, " + fmt(uav.ground_speed_mps, 1) + " m/s, heading " + fmt(uav.heading_deg, 0) +
        ", radio " + fmt(uav.radio_health, 2) + ", flown " + fmt(uav.distance_travelled_m, 0) + " m"
      : "no UAV selected";
    if (lastState) renderTable(lastState);
  }

  function render(state) {
    lastState = state;
    $("h-pois").textContent = state.metrics.pois_completed + "/" + state.metrics.pois_total;
    renderMetrics(state);
    renderTable(state);
    renderIncidents(state);
    charts.update(state);
    swarmMap.update(state);
    network.update(state);
  }

  /* -------------------------------------------------------------------- boot */
  function bindBuilderControls() {
    ["uav-count", "duration", "speed"].forEach(function (id) {
      var el = $(id);
      el.addEventListener("input", function () {
        var suffix = id === "duration" ? " s" : (id === "speed" ? "x" : "");
        $(id + "-value").textContent = el.value + suffix;
      });
    });
    $("clear-pois").addEventListener("click", function () { builder.clear(); });
    $("launch").addEventListener("click", launch);
  }

  window.addEventListener("resize", function () { if (charts) charts.draw(); });

  function requestedSession() {
    var match = /[?&]session=([a-z0-9]+)/i.exec(location.search);
    return match ? match[1] : null;
  }

  bindBuilderControls();
  initBuilder().then(function () {
    // Opening a shared link goes straight to that mission, unless it has expired.
    var wanted = requestedSession();
    if (!wanted) return;
    return fetch("/api/sessions/" + wanted).then(function (r) {
      if (r.ok) openSession(wanted);
      else $("builder-note").textContent = "That simulation has expired - build a new one.";
    });
  }).catch(function (err) {
    // Surfaced in both views: $("launch-error") is invisible once the live view
    // is open, and an openSession fault would otherwise vanish without trace.
    console.error("studio boot failed", err);
    $("launch-error").textContent = "Could not start: " + err.message;
    $("builder-note").textContent = "Could not start: " + err.message;
  });
})();
