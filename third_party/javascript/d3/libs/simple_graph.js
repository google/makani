/* A D3 library to plot 2D charts that can be zoomed/panned.
 * This is a modified version based on
 * https://gist.github.com/stepheneb/1182434
 */
SimpleGraph = function(properties, obj) {
  this.obj = obj;
  this.chart = document.getElementById(properties.selector.slice(1));
  var cx = this.chart.clientWidth;
  var cy = properties.aspect_ratio ?
    this.chart.clientWidth / properties.aspect_ratio
    : this.chart.clientHeight;

  var xLimits = properties.x_attrs.domain ?
    properties.x_attrs.domain : [0.0, 30.0];
  this.invertX = properties.x_attrs.invert ?
    properties.x_attrs.invert : false;
  this.numXTicks = properties.x_attrs.num_ticks ?
    properties.x_attrs.num_ticks : 10;

  var yLimits = properties.y_attrs.domain ?
    properties.y_attrs.domain : [0.0, 10.0];
  this.invertY = properties.y_attrs.invert ?
    properties.y_attrs.invert : false;
  this.numYTicks = properties.y_attrs.num_ticks ?
    properties.y_attrs.num_ticks : 10;

  this.interactive = properties.interactive ? properties.interactive : false;

  var padding_scale_x = cx / 1000 + 1;
  var padding_scale_y = cy / 1000 + 1;
  var padding = {
    "top":    (properties.title  ? 30 : 5) * padding_scale_y,
    "right":  15 * padding_scale_x,
    "bottom": (properties.x_attrs.label ? 60 : 20) * padding_scale_y,
    "left":   (properties.y_attrs.label ? 54 : 36) * padding_scale_x
  };

  this.size = {
    "width":  cx - padding.left - padding.right,
    "height": cy - padding.top  - padding.bottom
  };

  // x-scale
  this.x = d3.scale.linear()
      .range([0, this.size.width])
      .nice();
  this.setXDomain(xLimits).nice();

  // drag x-axis logic
  this.downx = Math.NaN;

  // y-scale
  this.y = d3.scale.linear()
      .range([0, this.size.height])
      .nice();
  this.setYDomain(yLimits).nice();

  // drag y-axis logic
  this.downy = Math.NaN;

  this.dragged = this.selected = null;

  this.canvas = d3.select(this.chart).append("svg")
      .attr("width",  cx)
      .attr("height", cy)
      .append("g")
      .attr("transform", "translate(" + padding.left + "," + padding.top + ")");

  this.plot = this.canvas.append("rect")
      .attr("width", this.size.width)
      .attr("height", this.size.height)
      .style("fill", "#EEEEEE")
      .attr("pointer-events", "all");

  this.updateScale();

  this.grid = this.canvas.append("g")
      .attr("id", "grid")

  this.svg = this.canvas.append("svg")
      .attr("top", 0)
      .attr("left", 0)
      .attr("width", this.size.width)
      .attr("height", this.size.height)
      .attr("viewBox", "0 0 "+this.size.width+" "+this.size.height);
      // .attr("class", "line");

  // add Chart Title
  if (properties.title) {
    this.canvas.append("text")
        .attr("class", "axis")
        .text(properties.title)
        .attr("x", this.size.width/2)
        .attr("dy","-0.8em")
        .style("text-anchor","middle");
  }

  // Add the x-axis label
  if (properties.x_attrs.label) {
    this.canvas.append("text")
        .attr("class", "axis")
        .text(properties.x_attrs.label)
        .attr("x", this.size.width/2)
        .attr("y", this.size.height)
        .attr("dy","2.4em")
        .style("text-anchor","middle");
  }

  // add y-axis label
  if (properties.y_attrs.label) {
    this.canvas.append("g").append("text")
        .attr("class", "axis")
        .text(properties.y_attrs.label)
        .style("text-anchor","middle")
        // Place the ylabel right against the left border of the padding.
        .attr("transform","translate(" + (-padding.left + 20) + ", " +
              this.size.height/2+") rotate(-90)");
  }

  if (this.interactive) {
    var self = this;
    d3.select(this.chart)
        .on("mousemove.drag", self.mousemove())
        .on("touchmove.drag", self.mousemove())
        .on("mouseup.drag",   self.mouseup())
        .on("touchend.drag",  self.mouseup());
  }

  this.redraw()();
};

//
// SimpleGraph methods
//

SimpleGraph.prototype.setXDomain = function(limits) {
  return this.x.domain(this.invertX ? [limits[1], limits[0]] : limits);
}

SimpleGraph.prototype.setYDomain = function(limits) {
  // Our notion of "inverted" corresponds to d3's default ordering of the
  // bounds.
  return this.y.domain(this.invertY ? limits : [limits[1], limits[0]]);
}

SimpleGraph.prototype.getXLimits = function() {
  var domain = this.x.domain();
  return this.invertX ? [domain[1], domain[0]] : domain
}

SimpleGraph.prototype.getYLimits = function() {
  var domain = this.y.domain();
  return this.invertY ? domain : [domain[1], domain[0]]
}

SimpleGraph.prototype.updateScale = function() {
  if (this.interactive) {
    // Change the zooming scale accordingly.
    this.plot.call(d3.behavior.zoom().x(this.x).y(this.y)
                     .on("zoom", this.redraw()));
  }
};

SimpleGraph.prototype.update = function() {
  if (this.obj.refresh) {
    this.obj.refresh();
  }

  if (d3.event && d3.event.keyCode) {
    d3.event.preventDefault();
    d3.event.stopPropagation();
  }
}

SimpleGraph.prototype.mousemove = function() {
  var self = this;
  return function() {
    // p is where the mouse is in the canvas's coordinate.
    // Note p[1] is 0 at the top of the canvas and largest at the bottom.
    var p = d3.mouse(self.canvas[0][0]),
        t = d3.event.changedTouches;
    if (self.dragged) {
      self.dragged.y = self.y.invert(Math.max(0, Math.min(self.size.height, p[1])));
      self.update();
    };
    // downx is the position of the mousedown event along the X axis.
    if (!isNaN(self.downx)) {
      d3.select(self.chart).style("cursor", "ew-resize");
      // rupx is the position of the mouse in the X axis coordination.
      var rupx = self.x.invert(p[0]),
          x_limits = self.getXLimits(),
          xaxis1 = x_limits[0],
          xaxis2 = x_limits[1],
          xextent = xaxis2 - xaxis1;
      if (rupx != 0) {
        var changex, new_limits;
        changex = (self.downx - xaxis1) / (rupx - xaxis1);
        new_limits = [xaxis1, xaxis1 + (xextent * changex)];
        self.setXDomain(new_limits)
        self.redraw()();
        self.updateScale();
      }
      d3.event.preventDefault();
      d3.event.stopPropagation();
    };
    // downy is the position of the mousedown event along the Y axis.
    if (!isNaN(self.downy)) {
      d3.select(self.chart).style("cursor", "ns-resize");
      // rupy is the position of the mouse in the Y axis coordination.
      var rupy = self.y.invert(p[1]),
          y_limits = self.getYLimits(),
          yaxis1 = y_limits[0],
          yaxis2 = y_limits[1],
          yextent = yaxis2 - yaxis1;
      if (rupy != 0) {
        var changey, new_limits;
        changey = (self.downy - yaxis1) / (rupy - yaxis1);
        new_limits = [yaxis1, yaxis1 + (yextent * changey)];
        self.setYDomain(new_limits);
        self.redraw()();
        self.updateScale();
      }
      d3.event.preventDefault();
      d3.event.stopPropagation();
    }
  }
};

SimpleGraph.prototype.mouseup = function() {
  var self = this;
  return function() {
    document.onselectstart = function() { return true; };
    d3.select(self.chart).style("cursor", "auto");
    d3.select(self.chart).style("cursor", "auto");
    if (!isNaN(self.downx)) {
      self.redraw()();
      self.downx = Math.NaN;
      d3.event.preventDefault();
      d3.event.stopPropagation();
    };
    if (!isNaN(self.downy)) {
      self.redraw()();
      self.downy = Math.NaN;
      d3.event.preventDefault();
      d3.event.stopPropagation();
    }
    if (self.dragged) {
      self.dragged = null
    }
  }
}

SimpleGraph.prototype.redraw = function() {
  var self = this;
  return function() {
    var tx = function(d) {
      return "translate(" + self.x(d) + ",0)";
    },
    ty = function(d) {
      return "translate(0," + self.y(d) + ")";
    },
    stroke = function(d) {
      return d ? "#ccc" : "#666";
    },
    fx = self.x.tickFormat(self.numXTicks),
    fy = self.y.tickFormat(self.numYTicks);

    // Regenerate x-ticks…
    var gx = self.grid.selectAll("g.x")
        .data(self.x.ticks(self.numXTicks), String)
        .attr("transform", tx);

    gx.select("text")
        .text(fx);

    var gxe = gx.enter().insert("g", "a")
        .attr("class", "x")
        .attr("transform", tx);

    gxe.append("line")
        .attr("stroke", stroke)
        .attr("y1", 0)
        .attr("y2", self.size.height);

    var x_text = gxe.append("text")
        .attr("class", "axis")
        .attr("y", self.size.height)
        .attr("dy", "1em")
        .attr("text-anchor", "middle")
        .text(fx)
        .style("cursor", "ew-resize");

    if (self.interactive) {
      x_text
        .on("mouseover", function(d) { d3.select(this).style("font-weight", "bold");})
        .on("mouseout",  function(d) { d3.select(this).style("font-weight", "normal");})
        .on("mousedown.drag",  self.xaxis_drag())
        .on("touchstart.drag", self.xaxis_drag());
    }

    gx.exit().remove();

    // Regenerate y-ticks…
    var gy = self.grid.selectAll("g.y")
        .data(self.y.ticks(self.numYTicks), String)
        .attr("transform", ty);

    gy.select("text")
        .text(fy);

    var gye = gy.enter().insert("g", "a")
        .attr("class", "y")
        .attr("transform", ty)
        .attr("background-fill", "#FFEEB6");

    gye.append("line")
        .attr("stroke", stroke)
        .attr("x1", 0)
        .attr("x2", self.size.width);

    var y_text = gye.append("text")
        .attr("class", "axis")
        .attr("x", -3)
        .attr("dy", ".35em")
        .attr("text-anchor", "end")
        .text(fy)
        .style("cursor", "ns-resize");

    if (self.interactive) {
      y_text
        .on("mouseover", function(d) { d3.select(this).style("font-weight", "bold");})
        .on("mouseout",  function(d) { d3.select(this).style("font-weight", "normal");})
        .on("mousedown.drag",  self.yaxis_drag())
        .on("touchstart.drag", self.yaxis_drag());
    }

    gy.exit().remove();
    self.updateScale();
    self.update();
  }
}

SimpleGraph.prototype.xaxis_drag = function() {
  var self = this;
  return function(d) {
    document.onselectstart = function() { return false; };
    var p = d3.mouse(self.canvas[0][0]);
    self.downx = self.x.invert(p[0]);
  }
};

SimpleGraph.prototype.yaxis_drag = function(d) {
  var self = this;
  return function(d) {
    document.onselectstart = function() { return false; };
    var p = d3.mouse(self.canvas[0][0]);
    self.downy = self.y.invert(p[1]);
  }
};

function getAxisScale(scale, range) {
  if (scale == 'time') {
    return d3.time.scale().range(range);
  } else if (scale == 'linear') {
    return d3.scale.linear().nice().range(range).nice();
  } else if (scale == 'log') {
    return d3.scale.log().range(range);
  }
}
