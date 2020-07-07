/*
 * The Drag-and-Drop Zoomable tree from D3.
 * Adapted from:
 * http://www.robschmuecker.com/d3-js-drag-and-drop-zoomable-tree/
 */

// start
function dndTree(json) {
    var treeData = json;
    
    // Calculate total nodes, max label length
    treeData.order = 0;
    if(treeData.children)
        treeData.max_peer_children = treeData.children.length;
    else
        treeData.max_peer_children = 0;
    var totalNodes = 0;
    var maxLabelLength = {};
    // variables for drag/drop
    var selectedNode = null;
    var draggingNode = null;
    var level_scale = 3.2;
    var level_offset = 20;
    // panning variables
    var panSpeed = 200;
    var panBoundary = 20; // Within 20px from edges will pan when dragging.
    // Misc. variables
    var i = 0;
    var duration = 750;
    var root;

    // size of the diagram
    var viewerWidth = graph_width;
    var viewerHeight = graph_height;

    var tree = d3.layout.tree()
        .size([viewerHeight, viewerWidth]);

    // define a d3 diagonal projection for use by the node paths later on.
    var diagonal = d3.svg.diagonal()
        .projection(function(d) {
            return [d.y, d.x];
        });

    // A recursive helper function for performing some setup by walking through all nodes

    function visit(parent, visitFn, childrenFn) {
        if (!parent) return;

        visitFn(parent);

        var children = childrenFn(parent);
        if (children) {
            var count = children.length;
            for (var i = 0; i < count; i++) {
                children[i].level = parent.level + 1;
                visit(children[i], visitFn, childrenFn);
            }
        }
    }

    function buildLabelLengths(d) {
        totalNodes++;
        if(!(d.level in maxLabelLength)) {
            maxLabelLength[d.level] = 0;
        }
        maxLabelLength[d.level] = Math.max(d.name.length, maxLabelLength[d.level]);
        var max_grandchildren = 0;
        if(d.children){
            d.num_children = d.children.length;
            for(var i = 0; i < d.num_children; i++) {
                d.children[i].order = i;
                if(d.children[i].children)
                    max_grandchildren = Math.max(d.children[i].children.length, max_grandchildren);
            }
            for(var i = 0; i < d.num_children; i++) {
                d.children[i].max_peer_children = max_grandchildren;
            }
        } else {
            d.num_children = 0;
        }
    }

    function getChildren(d) {
        return d.children && d.children.length > 0 ? d.children : null;
    }

    function accumulateLabelLengths(maxLabelLength){
      var accumLabelLength = {};
      var maxLevel = Object.keys(maxLabelLength).length;
      for (var i=0; i <= maxLevel; i++){
          if(i in maxLabelLength){
              if((i-1) in maxLabelLength){
                  accumLabelLength[i] = accumLabelLength[i-1]+maxLabelLength[i];
              } else {
                  accumLabelLength[i] = maxLabelLength[i];
              }
          } else {
                  accumLabelLength[i] = 0;
          }
      }
      return accumLabelLength;
    }

    // Call visit function to establish maxLabelLength, maxLevel, and num_children
    treeData.level = 0;
    visit(treeData, buildLabelLengths, getChildren);

    var accumLabelLength = accumulateLabelLengths(maxLabelLength);


    // sort the tree according to the node names
    function sortTreeByName(tree) {
        tree.sort(function(a, b) {
            return b.name.toLowerCase() < a.name.toLowerCase() ? 1 : -1;
        });
    }

    function sortTreeByNumChildren(tree) {
        tree.sort(function(a, b) {
            return b.num_children > a.num_children ? 1 : -1;
        });
    }

    function sortTree() {
        // first sort the tree by name
        sortTreeByName(tree);
        // then sort the tree by number of children
        // sortTreeByNumChildren(tree);
    }

    // Sort the tree initially incase the JSON isn't in a sorted order.
    sortTree();

    // TODO: Pan function, can be better implemented.

    function pan(domNode, direction) {
        var speed = panSpeed;
        if (panTimer) {
            clearTimeout(panTimer);
            translateCoords = d3.transform(svgGroup.attr("transform"));
            if (direction == 'left' || direction == 'right') {
                translateX = direction == 'left' ? translateCoords.translate[0] + speed : translateCoords.translate[0] - speed;
                translateY = translateCoords.translate[1];
            } else if (direction == 'up' || direction == 'down') {
                translateX = translateCoords.translate[0];
                translateY = direction == 'up' ? translateCoords.translate[1] + speed : translateCoords.translate[1] - speed;
            }
            scaleX = translateCoords.scale[0];
            scaleY = translateCoords.scale[1];
            scale = zoomListener.scale();
            svgGroup.transition().attr("transform", "translate(" + translateX + "," + translateY + ")scale(" + scale + ")");
            d3.select(domNode).select('g.node').attr("transform", "translate(" + translateX + "," + translateY + ")");
            zoomListener.scale(zoomListener.scale());
            zoomListener.translate([translateX, translateY]);
            panTimer = setTimeout(function() {
                pan(domNode, speed, direction);
            }, 50);
        }
    }

    // Define the zoom function for the zoomable tree

    function zoom() {
        svgGroup.attr("transform", "translate(" + d3.event.translate + ")scale(" + d3.event.scale + ")");
    }


    // define the zoomListener which calls the zoom function on the "zoom" event constrained within the scaleExtents
    var zoomListener = d3.behavior.zoom().scaleExtent([0.1, 3]).on("zoom", zoom);

    function initiateDrag(d, domNode) {
        draggingNode = d;
        // DRAG: d3.select(domNode).select('.ghostCircle').attr('pointer-events', 'none');
        // DRAG: d3.selectAll('.ghostCircle').attr('class', 'ghostCircle show');
        d3.select(domNode).attr('class', 'node activeDrag');

        svgGroup.selectAll("g.node").sort(function(a, b) { // select the parent and sort the path's
            if (a.id != draggingNode.id) return 1; // a is not the hovered element, send "a" to the back
            else return -1; // a is the hovered element, bring "a" to the front
        });
        // if nodes has children, remove the links and nodes
        if (nodes.length > 1) {
            // remove link paths
            links = tree.links(nodes);
            nodePaths = svgGroup.selectAll("path.link")
                .data(links, function(d) {
                    return d.target.id;
                }).remove();
            // remove child nodes
            nodesExit = svgGroup.selectAll("g.node")
                .data(nodes, function(d) {
                    return d.id;
                }).filter(function(d, i) {
                    if (d.id == draggingNode.id) {
                        return false;
                    }
                    return true;
                }).remove();
        }

        // remove parent link
        parentLink = tree.links(tree.nodes(draggingNode.parent));
        svgGroup.selectAll('path.link').filter(function(d, i) {
            if (d.target.id == draggingNode.id) {
                return true;
            }
            return false;
        }).remove();

        dragStarted = null;
    }

    // define the baseSvg, attaching a class for styling and the zoomListener
    var baseSvg = d3.select("#dndtree").append("svg")
        .attr("width", viewerWidth)
        .attr("height", viewerHeight)
        .attr("class", "overlay")
        .call(zoomListener);

    // Helper functions for collapsing and expanding nodes.

    function collapse(d) {
        if (d.children) {
            d._children = d.children;
            d._children.forEach(collapse);
            d.children = null;
        }
    }

    function expand(d) {
        if (d._children) {
            d.children = d._children;
            d.children.forEach(expand);
            d._children = null;
        }
    }

    var overCircle = function(d) {
        selectedNode = d;
        updateTempConnector();
    };
    var outCircle = function(d) {
        selectedNode = null;
        updateTempConnector();
    };

    // Function to update the temporary connector indicating dragging affiliation
    var updateTempConnector = function() {
        var data = [];
        if (draggingNode !== null && selectedNode !== null) {
            // have to flip the source coordinates since we did this for the existing connectors on the original tree
            data = [{
                source: {
                    x: selectedNode.y0,
                    y: selectedNode.x0
                },
                target: {
                    x: draggingNode.y0,
                    y: draggingNode.x0
                }
            }];
        }
        var link = svgGroup.selectAll(".templink").data(data);

        link.enter().append("path")
            .attr("class", "templink")
            .attr("d", d3.svg.diagonal())
            .attr('pointer-events', 'none');

        link.attr("d", d3.svg.diagonal());

        link.exit().remove();
    };

    // Function to center node when clicked/dropped so node doesn't get lost when collapsing/moving with large amount of children.

    function centerNode(source) {
        scale = zoomListener.scale();
        x = -source.y0;
        y = -source.x0;
        x = x * scale + viewerWidth / 2 - level_offset * level_scale;
        y = y * scale + viewerHeight / 2;
        d3.select('g').transition()
            .duration(duration)
            .attr("transform", "translate(" + x + "," + y + ")scale(" + scale + ")");
        zoomListener.scale(scale);
        zoomListener.translate([x, y]);
    }

    // Toggle children function

    function toggleChildren(d) {
        if (d.children) {
            d._children = d.children;
            d.children = null;
        } else if (d._children) {
            d.children = d._children;
            d._children = null;
        } else if (!d.leaf) {
            $.ajax({
              url: treeData.expand_url + d.path,
              success: function(response) {
                try {
                  children = JSON.parse(response);
                } catch(err) {
                  return;
                }
                if (children) {
                  d.children = children;
                  d._children = null;
                  var count = children.length;
                  for (var i = 0; i < count; i++) {
                    children[i].level = d.level + 1;
                    visit(children[i], buildLabelLengths, getChildren);
                  }
                  accumLabelLength = accumulateLabelLengths(maxLabelLength);
                  update(d);
                  centerNode(d);
                }
              }
            });
        } else {
            return null;
        }
        return d;
    }

    // Toggle children on click.

    function hammerClick(event) {
      // var datum = d3.select(this).datum();
      var datum = event.target.__data__;
      click(datum);
    }

    function click(d) {
        //if (d3.event.defaultPrevented)
        //    return; // click suppressed
        b = toggleChildren(d);
        if(b){
            update(b);
            centerNode(b);
        }else{
            clickLeaf(d);
        }
    }


    function fill_color(d) {
        if (d._children) {
          if (d.collapsed_color)
            return d.collapsed_color;
          else
            return "lightsteelblue";
        } else if (d.children) {
          if (d.in_color)
            return d.in_color;
          else
            return "#fff";
        } else {
          if (d.leaf_color)
            return d.leaf_color;
          else
            return "lightsteelblue";
        }
    }

    function stroke_color(d) {
      if (d.edge_color) {
        return d.edge_color;
      } else {
        var color = fill_color(d);
        if (color.indexOf("#") == 0)
          return ColorLuminance(color, -0.4);
        else
          return "darkgray";
      }
    }

    function update(source) {
        // Compute the new height, function counts total children of root node and sets tree height accordingly.
        // This prevents the layout looking squashed when new nodes are made visible or looking sparse when nodes are removed
        // This makes the layout more consistent.
        var levelWidth = [1];
        var childCount = function(level, n) {

            if (n.children && n.children.length > 0) {
                if (levelWidth.length <= level + 1) levelWidth.push(0);

                levelWidth[level + 1] += n.children.length;
                n.children.forEach(function(d) {
                    childCount(level + 1, d);
                });
            }
        };
        childCount(0, root);
        var newHeight = d3.max(levelWidth) * 25; // 25 pixels per line  
        tree = tree.size([newHeight, viewerWidth]);

        // Compute the new tree layout.
        var nodes = tree.nodes(root).reverse(),
            links = tree.links(nodes);

        // Set widths between levels based on maxLabelLength.
        nodes.forEach(function(d) {
            var maxLevel = Object.keys(maxLabelLength).length;
            if(d.level < maxLevel || d.depth == 0) {
                var label_len = accumLabelLength[d.level];
                d.y = (label_len + level_offset * d.level) * level_scale; //maxLabelLength * 10px
            } else {
                var label_len = accumLabelLength[d.level-1];
                d.y = (label_len + level_offset * d.level) * level_scale;
            }
            // alternatively to keep a fixed scale one can set a fixed depth per level
            // Normalize for fixed-depth by commenting out below line
            // d.y = (d.depth * 500); //500px per level.
        });

        // Update the nodes…
        node = svgGroup.selectAll("g.node")
            .data(nodes, function(d) {
                return d.id || (d.id = ++i);
            });

        // Enter any new nodes at the parent's previous position.
        var nodeEnter = node.enter().append("g")
            //DRAG: .call(dragListener)
            .attr("class", "node")
            .attr("transform", function(d) {
                return "translate(" + source.y0 + "," + source.x0 + ")";
            })
            .each(function(d, i){
                // install handlers with hammer
                Hammer(this, {
                  prevent_default: true,
                  no_mouseevents: true
                }).on('tap', hammerClick);
            });
            //.on('click', click);

        nodeEnter.append("circle")
            .attr('class', 'nodeCircle')
            .attr("r", 0)
            .style("stroke", stroke_color)
            .style("fill", fill_color);

        nodeEnter.append("text")
            .attr("x", function(d) {
                return d.children || d._children ? -10 : 10;
            })
            .attr("dy", ".35em")
            .attr('class', 'nodeText')
            .attr("text-anchor", function(d) {
                return d.children || d._children ? "end" : "start";
            })
            .text(function(d) {
                return d.name;
            })
            .style("fill-opacity", 0);

        // Update the text to reflect whether node has children or not.
        node.select('text')
            .attr("x", function(d) {
                return d.children || d._children ? -10 : 10;
            })
            .attr("text-anchor", function(d) {
                return d.children || d._children ? "end" : "start";
            })
            .attr("fill", function(d) {
                if (d.url) return "#05416c";
                else return "black";
            })
            .text(function(d) {
                return d.name;
            });

        // Change the circle fill depending on whether it has children and is collapsed
        node.select("circle.nodeCircle")
            //.attr("r", function(d){return 20.0*d.num_children/Math.max(1,d.max_peer_children);})
            .attr("r", function(d) {if (d.leaf) return 0; else return 6;})
            .style("stroke", stroke_color)
            .style("fill", fill_color);

        // Transition nodes to their new position.
        var nodeUpdate = node.transition()
            .duration(duration)
            .attr("transform", function(d) {
                return "translate(" + d.y + "," + d.x + ")";
            });

        // Fade the text in
        nodeUpdate.select("text")
            .style("fill-opacity", 1);

        // Transition exiting nodes to the parent's new position.
        var nodeExit = node.exit().transition()
            .duration(duration)
            .attr("transform", function(d) {
                return "translate(" + source.y + "," + source.x + ")";
            })
            .remove();

        nodeExit.select("circle")
            .attr("r", 0);

        nodeExit.select("text")
            .style("fill-opacity", 0);

        // Update the links…
        var link = svgGroup.selectAll("path.link")
            .data(links, function(d) {
                return d.target.id;
            });

        // Enter any new links at the parent's previous position.
        link.enter().insert("path", "g")
            .attr("class", "link")
            .attr("d", function(d) {
                var o = {
                    x: source.x0,
                    y: source.y0
                };
                return diagonal({
                    source: o,
                    target: o
                });
            });

        // Transition links to their new position.
        link.transition()
            .duration(duration)
            .attr("d", diagonal);

        // Transition exiting nodes to the parent's new position.
        link.exit().transition()
            .duration(duration)
            .attr("d", function(d) {
                var o = {
                    x: source.x,
                    y: source.y
                };
                return diagonal({
                    source: o,
                    target: o
                });
            })
            .remove();

        // Stash the old positions for transition.
        nodes.forEach(function(d) {
            d.x0 = d.x;
            d.y0 = d.y;
        });
    }

    // Append a group which holds all nodes and which the zoom Listener can act upon.
    var svgGroup = baseSvg.append("g");

    // Collapse all children
    var count = treeData.children.length;
    for (var i = 0; i < count; i++) {
        collapse(treeData.children[i]);
    }

    // Define the root
    root = treeData;
    root.x0 = viewerHeight / 2;
    root.y0 = 0;

    // Layout the tree initially and center on the root node.
    update(root);
    centerNode(root);
}

function ColorLuminance(hex, lum) {
    // validate hex string
    hex = String(hex).replace(/[^0-9a-f]/gi, '');
    if (hex.length < 6) {
            hex = hex[0]+hex[0]+hex[1]+hex[1]+hex[2]+hex[2];
    }
    lum = lum || 0;
    // convert to decimal and change luminosity
    var rgb = "#", c, i;
    for (i = 0; i < 3; i++) {
            c = parseInt(hex.substr(i*2,2), 16);
            c = Math.round(Math.min(Math.max(0, c + (c * lum)), 255)).toString(16);
            rgb += ("00"+c).substr(c.length);
    }
    return rgb;
}
