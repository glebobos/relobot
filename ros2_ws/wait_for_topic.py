#!/usr/bin/env python3
import os
import time
import sys
import rclpy
from rclpy.node import Node

class TopicWaiter(Node):
    def __init__(self, topics):
        super().__init__('topic_waiter')
        self.target_topics = set(topics)
        self.found_topics = set()
        self.get_logger().info(f"Waiting for topics: {self.target_topics}")

    def scan_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        current_topics = {name for name, types in topic_names_and_types}

        for topic in self.target_topics:
            found = False
            if topic in current_topics:
                found = True
            elif not topic.startswith('/') and ('/' + topic) in current_topics:
                found = True

            if found:
                if topic not in self.found_topics:
                    self.get_logger().info(f"Found topic: {topic}")
                    self.found_topics.add(topic)

        return self.target_topics.issubset(self.found_topics)

def main(args=None):
    rclpy.init(args=args)

    topics_str = os.environ.get('WAIT_FOR_TOPIC')
    if not topics_str:
        print("WAIT_FOR_TOPIC environment variable not set. Skipping wait.")
        return

    topics = [t.strip() for t in topics_str.split(',') if t.strip()]
    if not topics:
        print("No topics specified in WAIT_FOR_TOPIC. Skipping wait.")
        return

    print(f"Waiting for topics: {topics}")
    waiter = TopicWaiter(topics)

    try:
        while rclpy.ok():
            if waiter.scan_topics():
                print("All topics found!")
                break
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("Wait interrupted by user.")
        sys.exit(1)
    finally:
        # Check if rclpy is still initialized before shutting down
        # context.ok() check is implicit in rclpy.ok()
        if rclpy.ok():
            waiter.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
