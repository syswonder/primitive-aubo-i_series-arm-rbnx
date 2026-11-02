# SPDX-License-Identifier: MulanPSL-2.0
import json
import os
import sys
import tempfile
import types
import unittest

from aubo_arm.backend import RpcError


class Enum:
    def __init__(self, name):
        self.name = name


class FakeRobot:
    def __init__(self, log):
        self.log = log

    def getRobotState(self):
        return types.SimpleNamespace(getRobotModeType=lambda: Enum("Running"),
                                     getJointPositions=lambda: (0.1,) * 6)

    def getIoControl(self):
        def set_do(index, value):
            self.log.append(("setStandardDigitalOutput", index, value))
            return 0
        return types.SimpleNamespace(setStandardDigitalOutput=set_do)


class FakeRpc:
    instances = []
    login_ok = True

    def __init__(self):
        self.log, self.connected, self.logged_in = [], False, False
        FakeRpc.instances.append(self)

    def setRequestTimeout(self, ms):
        self.log.append(("timeout", ms))

    def connect(self, host, port):
        self.connected = True
        self.log.append(("connect", host, port))

    def hasConnected(self):
        return self.connected

    def login(self, user, password):
        self.logged_in = FakeRpc.login_ok

    def hasLogined(self):
        return self.logged_in

    def logout(self):
        self.logged_in = False

    def disconnect(self):
        self.connected = False

    def getRobotNames(self):
        return ["rob1"]

    def getRobotInterface(self, name):
        return FakeRobot(self.log)

    def getRuntimeMachine(self):
        return types.SimpleNamespace(getStatus=lambda: Enum("Running"))


class SdkClientTest(unittest.TestCase):
    def setUp(self):
        sys.modules["pyaubo_sdk"] = types.SimpleNamespace(RpcClient=FakeRpc)
        FakeRpc.instances.clear()
        FakeRpc.login_ok = True
        fd, self.cred = tempfile.mkstemp()
        os.write(fd, json.dumps({"username": "u", "password": "p"}).encode())
        os.close(fd)
        os.chmod(self.cred, 0o600)

    def tearDown(self):
        os.unlink(self.cred)
        sys.modules.pop("pyaubo_sdk", None)

    def client(self, motion=False):
        from aubo_arm.sdk import SdkClient
        return SdkClient("10.0.0.2", 30004, "rob1", 1.0, motion=motion, credentials_path=self.cred)

    def test_reads_normalize_enums_and_tuples(self):
        c = self.client()
        self.assertEqual(c.call("RobotState.getRobotModeType"), "Running")
        self.assertEqual(c.call("RuntimeMachine.getStatus"), "Running")
        self.assertEqual(c.call("RobotState.getJointPositions"), [0.1] * 6)
        self.assertIn(("timeout", 1000), FakeRpc.instances[0].log)

    def test_writes_need_motion_and_the_allowlist(self):
        with self.assertRaises(RpcError):
            self.client().command("IoControl.setStandardDigitalOutput", [5, False])
        c = self.client(motion=True)
        c.command("IoControl.setStandardDigitalOutput", [5, False])
        with self.assertRaisesRegex(RpcError, "not permitted"):
            c.call("RobotManage.poweroff")

    def test_group_readable_credentials_are_refused(self):
        os.chmod(self.cred, 0o640)
        with self.assertRaisesRegex(RpcError, "readable by group"):
            self.client()

    def test_failed_login_closes_the_connection(self):
        FakeRpc.login_ok = False
        with self.assertRaisesRegex(RpcError, "authentication"):
            self.client()
        self.assertFalse(FakeRpc.instances[0].connected)


if __name__ == "__main__":
    unittest.main()
