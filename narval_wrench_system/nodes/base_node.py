import rclpy
from narval_wrench_system.wrenchSystem import WrenchSystem

def main():
    rclpy.init()

    node = rclpy.create_node("n_wrench_system_base")

    wrenchSystem = WrenchSystem(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()