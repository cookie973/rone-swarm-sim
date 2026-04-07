import argparse
import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SearchMissionPublisher(Node):

    def __init__(self):
        super().__init__('search_mission_demo')
        self.pub = self.create_publisher(String, '/swarm/search_mission', 10)

    def publish_once(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.pub.publish(msg)


def _build_random_targets(count, x_min, x_max, y_min, y_max, min_weight, max_weight):
    targets = []
    for idx in range(count):
        targets.append({
            'id': f'target_{idx + 1}',
            'x': round(random.uniform(x_min, x_max), 2),
            'y': round(random.uniform(y_min, y_max), 2),
            'weight': round(random.uniform(min_weight, max_weight), 2),
        })
    return targets


def main(args=None):
    parser = argparse.ArgumentParser(description='发布随机加权多目标搜索任务')
    parser.add_argument('--count', type=int, default=3, help='目标点数量，必须小于 6')
    parser.add_argument('--seed', type=int, default=None, help='随机种子，便于复现实验')
    parser.add_argument('--x-min', type=float, default=80.0)
    parser.add_argument('--x-max', type=float, default=220.0)
    parser.add_argument('--y-min', type=float, default=-60.0)
    parser.add_argument('--y-max', type=float, default=60.0)
    parser.add_argument('--min-weight', type=float, default=1.0)
    parser.add_argument('--max-weight', type=float, default=5.0)
    parsed = parser.parse_args(args=args)

    if parsed.count <= 0 or parsed.count >= 6:
        raise SystemExit('count 必须在 1 到 5 之间')
    if parsed.seed is not None:
        random.seed(parsed.seed)

    targets = _build_random_targets(
        parsed.count,
        parsed.x_min,
        parsed.x_max,
        parsed.y_min,
        parsed.y_max,
        parsed.min_weight,
        parsed.max_weight,
    )
    payload = {
        'request_id': f'search_{int(time.time())}',
        'targets': targets,
    }

    rclpy.init(args=None)
    node = SearchMissionPublisher()
    try:
        for _ in range(5):
            node.publish_once(payload)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(payload, ensure_ascii=False))


if __name__ == '__main__':
    main()