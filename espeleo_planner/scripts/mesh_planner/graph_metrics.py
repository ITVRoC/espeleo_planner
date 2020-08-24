#!/usr/bin/env python

from enum import Enum


class GraphMetricType(Enum):
    SHORTEST = (0, "Shortest metric", (1.0, 0.0, 0.0))
    FLATTEST = (1, "Flattest metric", (1.0, 1.0, 1.0))
    ENERGY = (2, "Most energy efficient metric", (1.0, 1.0, 0.0))
    COMBINED = (3, "Combined metric", (0.0, 1.0, 0.0))
    STRAIGHTEST = (4, "Straightest metric", (0.5, 1.0, 0.5))
    FLATTEST_SIM = (5, "Flattest metric", (0.5, 0.5, 0.5))


class GraphMetric(object):
    """
    Doc here #todo
    """

    def __init__(self, metric, source_face_index, target_face_index):
        self.metric = metric
        self.source_face_index = source_face_index
        self.target_face_index = target_face_index

    def get_metric(self):
        return self.metric

    def get_long_desc(self):
        return self.metric.value[1]

    def get_color(self):
        return self.metric.value[2]

    def get_source(self):
        return self.source_face_index

    def get_target(self):
        return self.target_face_index
