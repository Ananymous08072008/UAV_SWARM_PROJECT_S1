/* Live time-series strips.

   The dashboard used to show only instantaneous numbers, which hides the whole
   point of the system: connectivity dips when a relay dies and climbs back as
   the swarm re-plans. These strips keep that history on screen while it happens.

   Drawn on a plain canvas rather than a charting library so the page keeps
   working with no internet connection - the Leaflet map already depends on a
   CDN, the charts should not add a second one. */
(function (global) {
  "use strict";

  var MAX_POINTS = 600;          // ~10 minutes of mission time at one sample/second

  function Strip(canvas, options) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.label = options.label;
    this.color = options.color || "#4fc3f7";
    this.min = options.min !== undefined ? options.min : 0;
    this.max = options.max !== undefined ? options.max : 1;
    this.format = options.format || function (v) { return v.toFixed(2); };
    this.autoscale = !!options.autoscale;
    this.points = [];            // [t_s, value]
  }

  Strip.prototype.reset = function () { this.points = []; };

  Strip.prototype.push = function (t, value) {
    if (value === null || value === undefined || isNaN(value)) return;
    var last = this.points[this.points.length - 1];
    if (last && t <= last[0]) return;               // ignore repeats and rewinds
    this.points.push([t, value]);
    if (this.points.length > MAX_POINTS) this.points.shift();
  };

  Strip.prototype.bounds = function () {
    if (!this.autoscale) return [this.min, this.max];
    var lo = Infinity, hi = -Infinity;
    this.points.forEach(function (p) { lo = Math.min(lo, p[1]); hi = Math.max(hi, p[1]); });
    if (lo === Infinity) return [this.min, this.max];
    if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }
    var pad = (hi - lo) * 0.15;
    return [Math.max(0, lo - pad), hi + pad];
  };

  Strip.prototype.draw = function () {
    var c = this.canvas, ctx = this.ctx;
    // Match the backing store to the CSS size so lines stay sharp on HiDPI.
    var ratio = global.devicePixelRatio || 1;
    var w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) return;
    if (c.width !== w * ratio || c.height !== h * ratio) {
      c.width = w * ratio; c.height = h * ratio;
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);

    var pad = { l: 4, r: 4, t: 14, b: 4 };
    var pw = w - pad.l - pad.r, ph = h - pad.t - pad.b;
    var range = this.bounds(), lo = range[0], hi = range[1];

    ctx.strokeStyle = "rgba(255,255,255,0.07)";
    ctx.lineWidth = 1;
    [0, 0.5, 1].forEach(function (f) {
      var y = pad.t + ph * f;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
    });

    var latest = this.points.length ? this.points[this.points.length - 1][1] : null;
    ctx.fillStyle = "#8b98a8";
    ctx.font = "10px ui-monospace, Consolas, monospace";
    ctx.fillText(this.label, pad.l, 10);
    if (latest !== null) {
      ctx.fillStyle = this.color;
      var text = this.format(latest);
      ctx.fillText(text, w - pad.r - ctx.measureText(text).width, 10);
    }

    if (this.points.length < 2) return;
    var t0 = this.points[0][0], t1 = this.points[this.points.length - 1][0];
    var span = Math.max(1e-6, t1 - t0);
    var self = this;
    function px(p) { return pad.l + ((p[0] - t0) / span) * pw; }
    function py(p) { return pad.t + ph - ((p[1] - lo) / Math.max(1e-9, hi - lo)) * ph; }

    ctx.beginPath();
    ctx.moveTo(px(this.points[0]), pad.t + ph);
    this.points.forEach(function (p) { ctx.lineTo(px(p), py(p)); });
    ctx.lineTo(px(this.points[this.points.length - 1]), pad.t + ph);
    ctx.closePath();
    ctx.fillStyle = this.color.replace(")", ", 0.12)").replace("rgb", "rgba");
    if (ctx.fillStyle === this.color) {   // hex colour: fall back to globalAlpha
      ctx.save(); ctx.globalAlpha = 0.12; ctx.fillStyle = this.color; ctx.fill(); ctx.restore();
    } else { ctx.fill(); }

    ctx.beginPath();
    this.points.forEach(function (p, i) {
      if (i === 0) ctx.moveTo(px(p), py(p)); else ctx.lineTo(px(p), py(p));
    });
    ctx.strokeStyle = this.color;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    var lastPoint = this.points[this.points.length - 1];
    ctx.beginPath();
    ctx.arc(px(lastPoint), py(lastPoint), 2.5, 0, Math.PI * 2);
    ctx.fillStyle = this.color;
    ctx.fill();
  };

  /* A row of strips fed from the live state payload. */
  function ChartBank(containerId) {
    var host = document.getElementById(containerId);
    this.strips = [];
    var specs = [
      { key: "connectivity", label: "UAVS CONNECTED", color: "#52d18b", min: 0, max: 1,
        format: function (v) { return Math.round(v * 100) + "%"; } },
      { key: "pdr", label: "ROUTE PDR", color: "#4fc3f7", min: 0, max: 1,
        format: function (v) { return v.toFixed(2); } },
      { key: "latency", label: "LATENCY ms", color: "#f2c14e", autoscale: true,
        format: function (v) { return Math.round(v) + " ms"; } },
      { key: "delivery", label: "IMAGERY DELIVERED %", color: "#b083f0", min: 0, max: 1,
        format: function (v) { return Math.round(v * 100) + "%"; } }
    ];
    var self = this;
    specs.forEach(function (spec) {
      var wrap = document.createElement("div");
      wrap.className = "strip";
      var canvas = document.createElement("canvas");
      wrap.appendChild(canvas);
      host.appendChild(wrap);
      var strip = new Strip(canvas, spec);
      strip.key = spec.key;
      self.strips.push(strip);
    });
  }

  ChartBank.prototype.reset = function () {
    this.strips.forEach(function (s) { s.reset(); });
    this.draw();
  };

  /* Seed from /history so joining a mission part-way shows the curve that led
     to now, instead of starting blank at whatever second you arrived. */
  ChartBank.prototype.seed = function (samples) {
    var byKey = {};
    this.strips.forEach(function (s) { byKey[s.key] = s; });
    (samples || []).forEach(function (row) {
      Object.keys(byKey).forEach(function (key) {
        if (row[key] !== undefined) byKey[key].push(row.t_s, row[key]);
      });
    });
    this.draw();
  };

  ChartBank.prototype.update = function (state) {
    var m = state.metrics || {};
    var t = state.world ? state.world.t_s : m.t_s;
    if (t === undefined || t === null) return;
    var data = m.data || {};
    var values = {
      connectivity: m.connectivity_ratio,
      pdr: m.mean_route_pdr,
      latency: m.mean_latency_ms,
      delivery: data.delivery_ratio
    };
    this.strips.forEach(function (s) { s.push(t, values[s.key]); });
    this.draw();
  };

  ChartBank.prototype.draw = function () {
    this.strips.forEach(function (s) { s.draw(); });
  };

  global.ChartBank = ChartBank;
})(window);
