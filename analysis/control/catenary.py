# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Utility for analyzing catenary shapes attained by tethers.

The catenary shape is calculated in terms of non-dimensional length
scale (normalized by the tether length) and force (normalized by the
tether weight).  The following terminology is used:

                                   ^ vertical tension
                                   |
               y        end point  o---> horizontal tension
               ^                  '
               |                 '
 anchor point  o---> x         _'
   at (0, 0)    `_           _o  point of interest
                  `_       _'
                    `--o--'
                      bottom (lowest point)

The vertical and horizontal tensions can be positive or negative.
Positions along the tether are parameterized by the "tether fraction"
between [0.0, 1.0].  The point at 0.0 tether fraction is the "anchor
point" and is fixed at the origin (0, 0).  Horizontal and vertical
tensions correspond to the forces applied at the "end point" located
at a tether fraction of 1.0.  The "point of interest" may be the
bottom of the tether or a point parameterized by a tether fraction.
If the catenary shape does not include a vertex, then the bottom
will be at one of the end points.
"""

import numpy as np
from scipy import optimize


def DimensionlessCatenaryBottom(horizontal, vertical):
  """Calculates the position of the bottom of the tether.

  Args:
    horizontal: Horizontal tension / tether weight [#].
    vertical: Vertical tension / tether weight [#] (must be non-negative).

  Returns:
    (x_bot, y_bot) where x_bot and y_bot are the horizontal and vertical
    coordinates of the bottom of the tether ("down" is along the negative y
    direction) divided by the tether length.
  """
  assert vertical >= 0.0

  if horizontal < 0.0:
    # Reduce the negative horizontal tension problem to the positive
    # horizontal tension problem.
    (x_bot, y_bot) = DimensionlessCatenaryBottom(-horizontal, vertical)
    return (-x_bot, y_bot)
  elif vertical <= 1.0:
    if np.abs(horizontal) < np.finfo(np.float).eps:
      x_bot = 0.0
    else:
      x_bot = horizontal * np.arcsinh((1.0 - vertical) / horizontal)

    y_bot = -(np.hypot(horizontal, 1.0 - vertical) - horizontal)

    return (x_bot, y_bot)
  else:
    return (0.0, 0.0)


def DimensionlessCatenaryPoint(horizontal, vertical, tether_fraction=1.0):
  """Calculates the location of a point along a dimensionless catenary.

  Calculates the position of the point tether_fraction along the tether.

  Args:
    horizontal: Horizontal tension / tether weight [#].
    vertical: Vertical tension / tether weight [#].
    tether_fraction: Position along tether length / tether length [#]
        (between 0.0 and 1.0).

  Returns:
    (x, y) where x and y are the horizontal and vertical
    coordinates of the point ("down" is along the negative y
    direction) divided by the tether length.
  """
  assert 0.0 <= tether_fraction and tether_fraction <= 1.0

  if horizontal < 0.0:
    # Reduce the negative horizontal tension problem to the positive
    # horizontal tension problem.
    (x, y) = DimensionlessCatenaryPoint(-horizontal, vertical, tether_fraction)
    return (-x, y)
  elif vertical < 0.0:
    # If vertical < 0.0, then the end point forces are pulling down on
    # the tether in addition to the tether weight.  This is equivalent
    # to the case where the end-point pulls up with more force than
    # the weight of the tether, but with the role of the end-point and
    # anchor point exchanged.
    (x_end, y_end) = DimensionlessCatenaryPoint(horizontal, 1.0 - vertical, 1.0)
    (x, y) = DimensionlessCatenaryPoint(horizontal, 1.0 - vertical,
                                        1.0 - tether_fraction)
    return (x_end - x, y - y_end)
  elif vertical <= 1.0:
    # Handle the familiar case where both anchor points bear some weight.

    # Fraction of tether length between vertex and the requested point.
    # Since the end point supports vertical * weight of the tether weight,
    # the vertex is at (1.0 - vertical) * length along the tether.
    s_bot = np.abs(tether_fraction - (1.0 - vertical))
    if np.abs(horizontal) < np.finfo(np.float).eps:
      x_diff = 0.0
    else:
      x_diff = horizontal * np.arcsinh(s_bot / horizontal)

    y_diff = np.hypot(horizontal, s_bot) - horizontal

    (x_bot, y_bot) = DimensionlessCatenaryBottom(horizontal, vertical)

    return (x_bot + (x_diff if tether_fraction > 1.0 - vertical else -x_diff),
            y_bot + y_diff)
  else:
    # If vertical > 1.0, then the anchor point at 0.0 starts to pull down
    # on the catenary with a force vertical - weight.
    #
    # This is equivalent to examining a section of the given length
    # belonging to a subsection of a tether of length (length *
    # vertical / weight).
    (x_a, y_a) = DimensionlessCatenaryPoint(
        horizontal / vertical, 1.0, 1.0 - 1.0 / vertical)
    (x_b, y_b) = DimensionlessCatenaryPoint(
        horizontal / vertical, 1.0, 1.0 + (tether_fraction - 1.0) / vertical)

    return (vertical * (x_b - x_a), vertical * (y_b - y_a))


def DimensionlessTensionsFromPoint(x, y, tol=1e-3):
  """Find the tensions that would result in a given equilibrium position.

  Args:
    x: Horizontal coordinates of the desired point, normalized by the
        tether length.
    y: Vertical coordinates of the desired point, normalized by the
        tether length.
    tol: Tolerance in position difference (normalized by tether length).

  Returns:
    Tuple (horizontal, vertical) containing best estimates of the necessary
    horizontal and vertical tensions normalized by the tether weight.
  """
  # The tether is assumed to be inelastic.
  assert np.hypot(x, y) < 1.0
  def Cost((horizontal, vertical)):
    (x_est, y_est) = DimensionlessCatenaryPoint(horizontal, vertical, 1.0)
    return [x_est - x, y_est - y]
  tension = optimize.fsolve(Cost, [0.0, 0.5])
  assert np.linalg.norm(Cost(tension)) < tol
  return tuple(tension)
