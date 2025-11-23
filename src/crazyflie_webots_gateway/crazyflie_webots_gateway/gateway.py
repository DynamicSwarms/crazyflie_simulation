import os
import asyncio
from asyncio.subprocess import Process
from signal import SIGINT
import time
from threading import Thread

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import EnvironmentVariable
from launch import LaunchContext
from ros2run.api import get_executable_path
import signal
import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import ChangeState, GetState


from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import threading
import sys

class GatewayError(Exception):
    pass


@dataclass
class CrazyflieInstance:
    get_state: Client
    change_state: Client
    process: Optional[Process] = None


class Gateway(Node):
    def __init__(self, event_loop):
        super().__init__(
            "crazyflie_webots_gateway",
        )
        self.declare_parameter("webots_port", 1234)
        self.declare_parameter("webots_use_tcp", False)
        self.declare_parameter("webots_tcp_ip", "127.0.0.1")

        self._event_loop = event_loop
        self._logger = self.get_logger()
        

        self._logger.info(
            "Started Webots-Gateway with "
            + "TCP?: {} IP: {} and PORT: {}. ".format(self._use_tcp, self._webots_ip, self._webots_port)
            + "Change this configuration by appropriate ROS2 parameters."
        )
        
        self.crazyflies: Dict[int, CrazyflieInstance] = {}
        self.crazyflies_lock = threading.Lock()
        self.crazyflie_callback_group = MutuallyExclusiveCallbackGroup()

        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            WebotsCrazyflie,
            "~/add_crazyflie",
            self._add_crazyflie_callback,
            callback_group=callback_group,
        )

        self.create_service(
            WebotsCrazyflie,
            "~/remove_crazyflie",
            self._remove_crazyflie_callback,
            callback_group=callback_group,
        )

    def remove_crazyflie(self, id: int) -> Tuple[bool, str]:
        self._logger.info("Removing crazyflie with ID: {}".format(id))
        with self.crazyflies_lock:
            if id in self.crazyflies.keys():
                self._transition_crazyflie(
                    id, LifecycleState.TRANSITION_STATE_SHUTTINGDOWN, "shutdown"
                )

                self.destroy_client(self.crazyflies[id].get_state)
                self.destroy_client(self.crazyflies[id].change_state)

                del self.crazyflies[id]
                return True, "Success"

        msg = "Couldn't remove crazyflie with ID: {}; was not active. ".format(id)
        self._logger.info(msg)
        return False, msg

    def remove_all_crazyflies(self) -> None:
        for cf_key in list(self.crazyflies.keys()):
            _ = self.remove_crazyflie(cf_key)

    def add_crazyflie(self, id: int) -> bool:
        with self.crazyflies_lock:
            self._logger.info("Adding crazyflie with ID: {}".format(id))
            if id in self.crazyflies.keys():           
                cf_state: Optional[LifecycleState] = self._get_state(id)
                if cf_state is not None:
                    if cf_state.id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
                        success = self._transition_crazyflie(
                            id,
                            LifecycleState.TRANSITION_STATE_CONFIGURING,
                            "configure",
                        )
                        return success
                    if cf_state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
                        # Called to add a crazyflie which is already properly initialized.
                        return True

                raise GatewayError("Cannot add Crazyflie, is already in Gateway!")
            else:
                wait_future = asyncio.run_coroutine_threadsafe(
                    self._create_cf(id), loop=self._event_loop
                )
                wait_future.add_done_callback(lambda fut: self._on_crazyflie_exit(fut, id))
                success = True

                if self._wait_for_change_state_service(id, timeout=3.0):
                    success = self._transition_crazyflie(
                        id,
                        LifecycleState.TRANSITION_STATE_CONFIGURING,
                        "configure",
                    )
                    return success
                else:
                    raise GatewayError(
                        f"Crazyflie {id} did not provide change_state service."
                    )

    def _on_crazyflie_exit(self, fut: asyncio.Future, id: int):
        """Callback invoked when a crazyflie subprocess exits."""
        self._logger.info(f"Crazyflie (id={id}) exited.")
        with self.crazyflies_lock:
            # Clean up the crazyflie entry from the dictionary
            if id in self.crazyflies.keys():
                self.destroy_client(self.crazyflies[id].get_state)
                self.destroy_client(self.crazyflies[id].change_state)
                del self.crazyflies[id]

    async def _create_cf(self, id: int):
        change_state_client = self.create_client(
            srv_type=ChangeState,
            srv_name=f"cf{id}/change_state",
            callback_group=self.crazyflie_callback_group,
        )
        get_state_client = self.create_client(
            srv_type=GetState,
            srv_name=f"cf{id}/get_state",
            callback_group=self.crazyflie_callback_group,
        )
        cmd = self._create_start_command(id)

        process = await asyncio.create_subprocess_exec(*cmd, preexec_fn=os.setsid)

        self.crazyflies[id] = CrazyflieInstance(change_state=change_state_client, get_state=get_state_client, process=process)
        await self.crazyflies[id].process.wait()

    def _create_start_command(self, id: int) -> str:
        crazyflie_path = get_executable_path(
            package_name="crazyflie_webots_cpp",
            executable_name="crazyflie",
        )

        cmd = [crazyflie_path, "--ros-args"]

        def add_parameter(name: str, value: str):
            cmd.append("-p")
            cmd.append(f"{name}:={value}")

        add_parameter("id", str(id))
        add_parameter("webots_port", str(self._webots_port))
        add_parameter("webots_use_tcp", str(self._use_tcp).lower())
        add_parameter("webots_tcp_ip", self._webots_ip)

        cmd += [
            "-r",
            "__node:=cf{}".format(id),
        ]

        return cmd

    @property
    def _webots_port(self) -> str:
        return self.get_parameter("webots_port").get_parameter_value().integer_value
    @property
    def _use_tcp(self) -> bool:
        return self.get_parameter("webots_use_tcp").get_parameter_value().bool_value
    @property
    def _webots_ip(self) -> str:
        return self.get_parameter("webots_tcp_ip").get_parameter_value().string_value


    def _add_crazyflie_callback(
        self, req: WebotsCrazyflie.Request, resp: WebotsCrazyflie.Response
    ) -> WebotsCrazyflie.Response:
        try:
            resp.success = self.add_crazyflie(req.id)
            if not resp.success:
                resp.msg = "See crazyflie log for details."
                self.remove_crazyflie(req.id)
        except (TimeoutError, Gateway) as ex:
            self.remove_crazyflie(req.id)
            resp.success = False
            resp.msg = str(ex)
        return resp

    def _remove_crazyflie_callback(
        self, req: WebotsCrazyflie.Request, resp: WebotsCrazyflie.Response
    ) -> WebotsCrazyflie.Response:
        resp.success, _msg = self.remove_crazyflie(req.id)
        return resp
    
    def _wait_for_change_state_service(
        self, key: int, timeout: float
    ) -> bool:
        while timeout > 0.0:
            if (
                key in self.crazyflies.keys()
                and self.crazyflies[key].change_state.service_is_ready()
            ):
                return True

            time.sleep(0.02)
            timeout -= 0.02
        return False

    def _wait_for_get_state_service(self, key: int, timeout: float) -> bool:
        while timeout > 0.0:
            if (
                key in self.crazyflies.keys()
                and self.crazyflies[key].get_state.service_is_ready()
            ):
                return True

            time.sleep(0.02)
            timeout -= 0.02
        return False

    def _transition_crazyflie(
        self, cf_id: int, state: LifecycleState, label: str
    ) -> bool:
        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        if cf_id in self.crazyflies.keys():
            self._wait_for_change_state_service(cf_id, 0.2)
            fut = self.crazyflies[cf_id].change_state.call_async(request)
            timeout = 0.0
            while not fut.done() and timeout < 10.0:
                rclpy.spin_until_future_complete(node=self, future=fut, timeout_sec=0.1)
                if not self._wait_for_change_state_service(cf_id, 0.1):
                    raise TimeoutError("Crazyflie died during transition.")
                timeout += 0.1

            if fut.done():
                response: Optional[ChangeState.Response] = fut.result()
                response: ChangeState.Response
                return response.success

            raise TimeoutError("Service call for transition timed out.")
        else:
            raise GatewayError(
                f"Crazyflie with id: {cf_id} not available."
            )

    def _get_state(self, cf_id: int) -> Optional[LifecycleState]:
        request = GetState.Request()
        if cf_id in self.crazyflies.keys():
            self._wait_for_get_state_service(cf_id, 0.2)
            fut: Future = self.crazyflies[cf_id].get_state.call_async(
                request
            )
            rclpy.spin_until_future_complete(node=self, future=fut, timeout_sec=1.0)
            response: Optional[GetState.Response] = fut.result()
            if fut.done():
                response: GetState.Response
                return response.current_state
            else:
                raise TimeoutError("Service call for getting the state timed out.")



async def run_node(cf_eventloop):
    rclpy.init()
    gateway = Gateway(cf_eventloop)
    try:
        while rclpy.ok():
            await asyncio.sleep(0.01)
            rclpy.spin_once(gateway, timeout_sec=0)
        print("Shutting down gateway...")
        rclpy.shutdown()
    except (asyncio.CancelledError, asyncio.exceptions.CancelledError, ExternalShutdownException):
        print("Shutting down gateway...this ways")   
        gateway.remove_all_crazyflies()
        print("All crazyflies removed.")
        rclpy.spin_once(gateway, timeout_sec=1.0)
        rclpy.shutdown()

def run_crazyflie_loop(event_loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(event_loop)

    async def keep_alive():
        try:
            while True:
                await asyncio.sleep(1)  # Keep the loop alive
        except asyncio.CancelledError:
            pass

    alive_task = asyncio.ensure_future(keep_alive())
    # If there is no future in the loop it cannot be stopped...

    event_loop.run_forever()
    alive_task.cancel()
    for fut in asyncio.all_tasks(event_loop):
        event_loop.run_until_complete(fut)
    event_loop.close()


def run_gateway_loop(
    event_loop: asyncio.AbstractEventLoop,
    crazyflie_event_loop: asyncio.AbstractEventLoop,
):
    asyncio.set_event_loop(event_loop)
    node = asyncio.ensure_future(run_node(crazyflie_event_loop), loop=event_loop)
    event_loop.run_forever()
    node.cancel()
    event_loop.run_until_complete(node)
    event_loop.close()


def main():
    gateway_event_loop = asyncio.new_event_loop()
    crazyflie_event_loop = asyncio.new_event_loop()

    crazyflie_thread = Thread(target=run_crazyflie_loop, args=(crazyflie_event_loop,))
    gateway_thread = Thread(
        target=run_gateway_loop, args=(gateway_event_loop, crazyflie_event_loop)
    )
    crazyflie_thread.start()
    gateway_thread.start()

    #is_running = True
    #def handle_sigint(signum, frame):
    #    nonlocal is_running
    #    is_running = False
    #    crazyflie_event_loop.call_soon_threadsafe(crazyflie_event_loop.stop)
    #    gateway_event_loop.call_soon_threadsafe(gateway_event_loop.stop)
#
    #signal.signal(signal.SIGINT, handle_sigint) 
    #signal.signal(signal.SIGTERM, exit())
    #while is_running:
    #    time.sleep(0.5)
    
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        crazyflie_event_loop.stop()
        gateway_event_loop.stop()

    crazyflie_thread.join()
    gateway_thread.join()
    exit()


if __name__ == "__main__":
    main()
