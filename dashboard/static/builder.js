/* Mission builder map: click inside the area to drop a Point of Interest.

   Uses the same flat-earth conversion as map.js and the Python side, but in
   reverse - a click gives lat/lon, the simulation wants local ENU metres. */
(function (global) {
  "use strict";
  var EARTH_R = 6378137.0;
  var PRIORITY_COLOR = { 1: "#8b98a8", 2: "#4fc3f7", 3: "#52d18b", 4: "#f2c14e", 5: "#ff6b6b" };

  function BuilderMap(elementId, limits) {
    this.limits = limits;
    this.origin = limits.origin;
    this.area = limits.area;
    this.pois = [];
    this.onChange = null;

    this.map = L.map(elementId, { zoomControl: true, scrollWheelZoom: false,
                                  zoomSnap: 0.25, zoomDelta: 0.5 });
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19, attribution: "&copy; OpenStreetMap"
    }).addTo(this.map);
    this.layer = L.layerGroup().addTo(this.map);

    var a = this.area;
    var sw = this.toLatLon(a.x_min_m, a.y_min_m);
    var ne = this.toLatLon(a.x_max_m, a.y_max_m);
    this.bounds = L.latLngBounds(sw, ne);
    L.rectangle(this.bounds, { color: "#f2c14e", weight: 1, dashArray: "5 5",
                               fill: false, interactive: false }).addTo(this.map);
    L.marker(this.toLatLon(0, 0), { interactive: false }).addTo(this.map).bindTooltip("GCS", {
      permanent: true, direction: "right", className: "gcs-tip"
    });
    this.map.fitBounds(this.bounds, { padding: [20, 20] });

    var self = this;
    this.map.on("click", function (e) { self.addAt(e.latlng); });
  }

  BuilderMap.prototype.toLatLon = function (x, y) {
    var o = this.origin;
    return [o.lat_deg + (y / EARTH_R) * 180 / Math.PI,
            o.lon_deg + (x / (EARTH_R * Math.cos(o.lat_deg * Math.PI / 180))) * 180 / Math.PI];
  };

  BuilderMap.prototype.toMetres = function (latlng) {
    var o = this.origin;
    return {
      x_m: Math.round((latlng.lng - o.lon_deg) * Math.PI / 180 * EARTH_R
                      * Math.cos(o.lat_deg * Math.PI / 180)),
      y_m: Math.round((latlng.lat - o.lat_deg) * Math.PI / 180 * EARTH_R)
    };
  };

  BuilderMap.prototype.addAt = function (latlng) {
    if (this.pois.length >= this.limits.max_pois) {
      this.notify("At most " + this.limits.max_pois + " PoIs.");
      return;
    }
    var p = this.toMetres(latlng);
    var a = this.area;
    // Server-side validation would reject this anyway; refusing here explains why.
    if (p.x_m < a.x_min_m || p.x_m > a.x_max_m || p.y_m < a.y_min_m || p.y_m > a.y_max_m) {
      this.notify("Place PoIs inside the dashed mission area.");
      return;
    }
    this.pois.push({ id: "POI-" + (this.pois.length + 1), x_m: p.x_m, y_m: p.y_m,
                     priority: 3, survey_time_s: 45 });
    this.redraw();
  };

  BuilderMap.prototype.remove = function (index) {
    this.pois.splice(index, 1);
    this.pois.forEach(function (p, i) { p.id = "POI-" + (i + 1); });
    this.redraw();
  };

  BuilderMap.prototype.setPriority = function (index, priority) {
    this.pois[index].priority = priority;
    this.redraw();
  };

  BuilderMap.prototype.clear = function () { this.pois = []; this.redraw(); };

  BuilderMap.prototype.notify = function (message) {
    if (this.onNotify) this.onNotify(message);
  };

  BuilderMap.prototype.redraw = function () {
    var self = this;
    this.layer.clearLayers();
    this.pois.forEach(function (p, i) {
      var marker = L.circleMarker(self.toLatLon(p.x_m, p.y_m), {
        radius: 7 + p.priority, color: "#0d1117", weight: 2,
        fillColor: PRIORITY_COLOR[p.priority] || "#52d18b", fillOpacity: 0.9
      }).addTo(self.layer);
      marker.bindTooltip(p.id + " (p" + p.priority + ")", { direction: "top" });
      marker.on("click", function (e) { L.DomEvent.stop(e); self.remove(i); });
    });
    if (this.onChange) this.onChange(this.pois);
  };

  BuilderMap.prototype.invalidate = function () {
    var self = this;
    setTimeout(function () {
      self.map.invalidateSize();
      if (self.bounds) self.map.fitBounds(self.bounds, { padding: [20, 20] });
    }, 50);
  };

  global.BuilderMap = BuilderMap;
})(window);
