#!/usr/bin/env python3
from typing import Optional, List
import rclpy
from rclpy.node import Node
from mrta_interfaces.srv import ReserveCharge, ReleaseCharge  # 인터페이스 빌드 후 사용 가능

class ChargeManager(Node):
    def __init__(self) -> None:
        super().__init__('charge_manager')
        self.declare_parameter('stations_config', '')
        self.create_service(ReserveCharge, 'reserve_charge', self.handle_reserve)
        self.create_service(ReleaseCharge, 'release_charge', self.handle_release)
        self.get_logger().info('ChargeManager up')

    def handle_reserve(self, req: ReserveCharge.Request, resp: ReserveCharge.Response) -> ReserveCharge.Response:
        resp.accepted = True
        resp.station_id = 'station_0'
        return resp

    def handle_release(self, req: ReleaseCharge.Request, resp: ReleaseCharge.Response) -> ReleaseCharge.Response:
        resp.success = True
        return resp

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ChargeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
