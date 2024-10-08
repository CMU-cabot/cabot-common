#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml
from pathlib import Path


class BagReader:
    def __init__(self, filename, serialization_format='cdr'):
        self.filepath = Path(filename)
        data = yaml.safe_load(open(self.filepath / "metadata.yaml"))

        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.filepath),
            storage_id=data['rosbag2_bagfile_information']['storage_identifier'])

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format)

        compression = data['rosbag2_bagfile_information']['compression_mode'] != ""

        if compression:
            self.reader = rosbag2_py.SequentialCompressionReader()
        else:
            self.reader = rosbag2_py.SequentialReader()
        self.info = rosbag2_py.Info().read_metadata(storage_options.uri, storage_options.storage_id)
        self.reader.open(storage_options, converter_options)
        self.topic_types = self.reader.get_all_topics_and_types()
        self.message_counts = {}
        for info in self.info.topics_with_message_count:
            self.message_counts[info.topic_metadata.name] = info.message_count
        self.type_map = {self.topic_types[i].name: self.topic_types[i].type for i in range(len(self.topic_types))}
        if hasattr(self.info.starting_time, "nanoseconds"):
            self.start_time = self.info.starting_time.nanoseconds/1e9
        else:   # back compatibility for galactic
            self.start_time = self.info.starting_time.timestamp()
        self.start = 0
        self.duration = 9999999999

    # function wrappers
    def read_next(self):
        return self.reader.read_next()

    def has_next(self):
        return self.reader.has_next()

    def get_all_topics_and_types(self):
        return self.reader.get_all_topics_and_types()

    def set_filter(self, filter):
        self.reader.set_filter(filter)

    def reset_filter(self):
        self.reader.reset_filter()

    def seek(self, timestamp):
        self.reader.seek(timestamp)

    # extended functions
    def set_filter_by_topics(self, topics):
        self.set_filter(rosbag2_py.StorageFilter(topics=topics))

    def set_filter_by_options(self, options):
        if hasattr(options, 'start'):
            self.start = options.start
        if hasattr(options, 'duration'):
            self.duration = options.duration

    def serialize_next(self):
        while self.reader.has_next():
            (topic, msg_data, nt) = self.reader.read_next()
            t = nt/1e9

            if (t - self.start_time) < self.start:
                continue
            if self.start + self.duration < (t - self.start_time):
                continue

            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(msg_data, msg_type)
            return (topic, msg, t, t - self.start_time)
        return None, None, None, None

    def bag_duration(self):
        return self.info.duration.total_seconds()
