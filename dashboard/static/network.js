/* Communication graph (Cytoscape): GCS + airborne UAVs, edges = usable links.
   Node positions mirror the real geometry, so the graph reads like the map. */
(function (global) {
  "use strict";

  function SwarmNetwork(elementId) {
    this.cy = cytoscape({
      container: document.getElementById(elementId),
      elements: [],
      style: [
        { selector: "node", style: {
            "background-color": "data(color)", "label": "data(label)", "color": "#e6edf3",
            "font-size": 9, "text-valign": "top", "text-margin-y": -2, "width": 16, "height": 16,
            "border-width": 2, "border-color": "data(border)"
        }},
        { selector: "node[?gcs]", style: { "shape": "round-rectangle", "width": 34, "height": 18 } },
        { selector: "edge", style: {
            "line-color": "data(color)", "width": "data(width)", "curve-style": "straight",
            "opacity": "data(opacity)", "line-style": "data(style)"
        }}
      ],
      layout: { name: "preset" }, userZoomingEnabled: true, userPanningEnabled: true
    });
    this.fitted = false;
  }

  SwarmNetwork.prototype.update = function (state) {
    var nodes = state.env.node_positions || {};
    var byId = {};
    state.world.uavs.forEach(function (u) { byId[u.uav_id] = u; });

    var routeEdges = {};
    (state.swarm.routes || []).forEach(function (r) {
      for (var i = 0; i + 1 < r.path.length; i++) {
        routeEdges[Math.min(r.path[i], r.path[i + 1]) + "-" + Math.max(r.path[i], r.path[i + 1])] = true;
      }
    });

    var elements = [];
    Object.keys(nodes).forEach(function (key) {
      var id = parseInt(key, 10), pos = nodes[key], uav = byId[id];
      elements.push({
        data: {
          id: "n" + id, label: id === 0 ? "GCS" : (uav ? uav.name : "UAV-" + id),
          color: id === 0 ? "#e6edf3" : (global.ROLE_COLOR[uav ? uav.role : "IDLE"] || "#8b98a8"),
          border: uav && !uav.connected ? "#ff6b6b" : "#2a323e",
          gcs: id === 0 ? 1 : 0
        },
        position: { x: pos[0] * 0.28, y: -pos[1] * 0.28 }
      });
    });
    (state.env.links || []).forEach(function (lk) {
      if (!lk.up || !nodes[String(lk.a)] || !nodes[String(lk.b)]) return;
      var onRoute = routeEdges[Math.min(lk.a, lk.b) + "-" + Math.max(lk.a, lk.b)];
      elements.push({ data: {
        id: "e" + lk.a + "_" + lk.b, source: "n" + lk.a, target: "n" + lk.b,
        color: global.qualityColor(lk.pdr), width: onRoute ? 1 + 4 * lk.pdr : 1,
        opacity: onRoute ? 0.95 : 0.3, style: lk.obstructed ? "dashed" : "solid"
      }});
    });

    this.cy.json({ elements: elements });
    this.cy.layout({ name: "preset" }).run();
    if (!this.fitted && elements.length) { this.cy.fit(undefined, 30); this.fitted = true; }
  };

  global.SwarmNetwork = SwarmNetwork;
})(window);
