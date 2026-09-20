/* Live Leaflet map: UAVs, PoIs, relay links, obstacles and the disaster zone.
   Local ENU metres are converted with the same flat-earth formula the Python
   side uses, so the map, Mission Planner and the simulation always agree. */
(function (global) {
  "use strict";
  var EARTH_R = 6378137.0;

  var ROLE_COLOR = {
    SURVEY: "#4fc3f7", RELAY: "#b083f0", IDLE: "#8b98a8",
    BACKUP: "#52d18b", RETURNING: "#f2c14e", CHARGING: "#6b7a8d"
  };
  var POI_COLOR = {
    PENDING: "#8b98a8", ASSIGNED: "#4fc3f7", IN_PROGRESS: "#f2c14e", COMPLETED: "#52d18b"
  };

  function qualityColor(pdr) {
    if (pdr >= 0.85) return "#52d18b";
    if (pdr >= 0.6) return "#b8d64a";
    if (pdr >= 0.45) return "#f2c14e";
    return "#ff6b6b";
  }

  function SwarmMap(elementId) {
    // scrollWheelZoom off: the wheel scrolls the page, +/- or double-click zooms the map
    this.map = L.map(elementId, { zoomControl: true, attributionControl: true, scrollWheelZoom: false,
                                  zoomSnap: 0.25, zoomDelta: 0.5 });   // fractional zoom frames the area tightly
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19, attribution: "&copy; OpenStreetMap"
    }).addTo(this.map);
    this.layers = {};
    ["zone", "obstacles", "links", "plan", "pois", "uavs", "gcs"].forEach(function (name) {
      this.layers[name] = L.layerGroup().addTo(this.map);
    }, this);
    this.map.setView([0, 0], 3);
    this.fitted = false;
    this.selected = null;
    this.onSelect = null;
  }

  SwarmMap.prototype.toLatLon = function (x, y) {
    var o = this.origin;
    var lat = o.lat_deg + (y / EARTH_R) * 180 / Math.PI;
    var lon = o.lon_deg + (x / (EARTH_R * Math.cos(o.lat_deg * Math.PI / 180))) * 180 / Math.PI;
    return [lat, lon];
  };

  SwarmMap.prototype.update = function (state) {
    var self = this;
    this.origin = state.scenario.origin;
    Object.keys(this.layers).forEach(function (k) { self.layers[k].clearLayers(); });

    var zone = state.env.disaster_zone || [];
    if (zone.length > 2) {
      L.polygon(zone.map(function (p) { return self.toLatLon(p[0], p[1]); }),
        { color: "#f2984e", weight: 1, dashArray: "6 6", fill: false })
        .bindTooltip("disaster area").addTo(this.layers.zone);
    }

    (state.env.obstacles || []).forEach(function (o) {
      var pts = (o.polygon_latlon && o.polygon_latlon.length)
        ? o.polygon_latlon
        : o.polygon_m.map(function (p) { return self.toLatLon(p[0], p[1]); });
      L.polygon(pts, { color: "#ff6b6b", weight: 1, fillOpacity: 0.28 })
        .bindTooltip(o.id + " - " + o.height_m + " m, -" + o.attenuation_db + " dB")
        .addTo(self.layers.obstacles);
    });

    var nodes = state.env.node_positions || {};
    var routeEdges = {};
    (state.swarm.routes || []).forEach(function (r) {
      for (var i = 0; i + 1 < r.path.length; i++) {
        var a = r.path[i], b = r.path[i + 1];
        routeEdges[Math.min(a, b) + "-" + Math.max(a, b)] = true;
      }
    });
    (state.env.links || []).forEach(function (lk) {
      if (!lk.up) return;
      var a = nodes[String(lk.a)], b = nodes[String(lk.b)];
      if (!a || !b) return;
      var onRoute = routeEdges[Math.min(lk.a, lk.b) + "-" + Math.max(lk.a, lk.b)];
      L.polyline([self.toLatLon(a[0], a[1]), self.toLatLon(b[0], b[1])], {
        color: qualityColor(lk.pdr), weight: onRoute ? 3 : 1.5,
        opacity: onRoute ? 0.9 : 0.35, dashArray: lk.obstructed ? "4 5" : null
      }).bindTooltip("link " + lk.a + "-" + lk.b + ": PDR " + lk.pdr.toFixed(2) +
        ", " + lk.distance_m.toFixed(0) + " m, " + lk.latency_ms.toFixed(0) + " ms")
        .addTo(self.layers.links);
    });

    ((state.swarm.relays || {}).chains || []).forEach(function (chain) {
      chain.points.forEach(function (p) {
        L.circleMarker(self.toLatLon(p[0], p[1]), {
          radius: 5, color: "#b083f0", weight: 1, fillOpacity: 0, dashArray: "2 3"
        }).bindTooltip("planned relay for " + chain.terminal).addTo(self.layers.plan);
      });
    });

    state.world.pois.forEach(function (poi) {
      var marker = L.circleMarker([poi.lat_deg, poi.lon_deg], {
        radius: 5 + poi.priority, color: POI_COLOR[poi.status] || "#8b98a8",
        weight: 2, fillOpacity: poi.status === "COMPLETED" ? 0.55 : 0.2
      });
      marker.bindTooltip(poi.poi_id + " (p" + poi.priority + ") " + poi.status +
        " " + Math.round(poi.progress_ratio * 100) + "%");
      marker.addTo(self.layers.pois);
    });

    var gcs = L.circleMarker([state.world.gcs_lat_deg, state.world.gcs_lon_deg],
      { radius: 8, color: "#e6edf3", fillColor: "#e6edf3", fillOpacity: 0.9, weight: 2 });
    gcs.bindTooltip("GCS", { permanent: true, direction: "right", className: "map-label" });
    gcs.addTo(this.layers.gcs);

    state.world.uavs.forEach(function (uav) {
      var failed = uav.health === "FAILED";
      var marker = L.circleMarker([uav.lat_deg, uav.lon_deg], {
        radius: uav.uav_id === self.selected ? 9 : 6,
        color: uav.uav_id === self.selected ? "#ffffff" : (failed ? "#ff6b6b" : ROLE_COLOR[uav.role]),
        fillColor: failed ? "#ff6b6b" : ROLE_COLOR[uav.role],
        fillOpacity: uav.airborne ? 0.95 : 0.35, weight: 2
      });
      marker.bindTooltip(uav.name + " " + uav.role +
        (uav.assigned_poi ? " -> " + uav.assigned_poi : "") +
        " | " + uav.battery_pct.toFixed(0) + "% | " +
        (uav.connected ? "PDR " + uav.pdr.toFixed(2) + " / " + uav.hop_count + " hops" : "NO LINK"));
      marker.on("click", function () { if (self.onSelect) self.onSelect(uav.uav_id); });
      marker.addTo(self.layers.uavs);
    });

    if (!this.fitted && state.scenario.area_latlon) {
      this.map.fitBounds(state.scenario.area_latlon, { padding: [10, 10] });
      this.fitted = true;
    }
  };

  global.SwarmMap = SwarmMap;
  global.qualityColor = qualityColor;
  global.ROLE_COLOR = ROLE_COLOR;
})(window);
