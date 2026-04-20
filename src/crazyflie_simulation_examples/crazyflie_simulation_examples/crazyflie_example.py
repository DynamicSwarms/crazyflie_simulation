import time
from crazyflie_simulation_examples.crazyflie_client import Crazyflie


def main():
    time.sleep(2)  # Wait for the gateway to start and the crazyflie to be added
    cf = Crazyflie(cf_id=0)

    cf.takeoff(height=1.0, secs=3)
    cf.get_logger().info("Taking off...")

    time.sleep(3)

    cf.goto(position=[0.0, 2.0, 2.0], secs=3)
    cf.get_logger().info("Going to position [0.0, 2.0, 2.0]...")

    time.sleep(3)

    cf.land(secs=3)
    cf.get_logger().info("Landing...")

    time.sleep(3)


if __name__ == "__main__":
    main()
