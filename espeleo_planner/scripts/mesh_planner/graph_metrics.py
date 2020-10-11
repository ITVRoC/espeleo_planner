#!/usr/bin/env python

from enum import Enum


class GraphMetricType(Enum):
    SHORTEST = (0, "Shortest metric", (1.0, 0.0, 0.0))
    FLATTEST = (1, "Flattest metric", (1.0, 1.0, 1.0))
    ENERGY = (2, "Most energy efficient metric", (1.0, 1.0, 0.0))
    COMBINED = (3, "Combined metric", (0.0, 1.0, 0.0))
    STRAIGHTEST = (4, "Straightest metric", (0.5, 1.0, 0.5))
    FLATTEST_SIM = (5, "Flattest metric", (0.5, 0.5, 0.5))
