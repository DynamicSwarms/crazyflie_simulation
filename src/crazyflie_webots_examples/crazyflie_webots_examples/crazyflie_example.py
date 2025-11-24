import time
from crazyflie_webots_examples.crazyflie_client import Crazyflie


def main():

    cf = Crazyflie()

    cf.start()

    time.sleep(5)

    cf.goto([0, 2, 2])

    time.sleep(3)

    cf.stop()


if __name__ == "__main__":
    main()
