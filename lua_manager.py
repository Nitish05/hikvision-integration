"""Upload, run, and stop Lua scripts on the Fairino FR5 controller."""
from fairino import Robot
import sys
import time
import os

IP = '192.168.57.2'
LUA_FILE = os.path.join(os.path.dirname(__file__), 'di_do_passthrough.lua')
PROGRAM_NAME = 'di_do_passthrough.lua'


def main():
    if len(sys.argv) < 2:
        print("Usage: python lua_manager.py [upload|start|stop|status|list]")
        return

    cmd = sys.argv[1]
    robot = Robot.RPC(IP)
    robot.connect_to_robot()

    if cmd == 'upload':
        err = robot.LuaUpload(LUA_FILE)
        if err == 0:
            print(f"Uploaded {LUA_FILE} to robot")
        else:
            print(f"Upload failed: {err}")

    elif cmd == 'start':
        err = robot.ProgramLoad(PROGRAM_NAME)
        if err != 0:
            print(f"ProgramLoad failed: {err}")
            return
        robot.Mode(0)  # auto mode required for program execution
        time.sleep(0.5)
        err = robot.ProgramRun()
        if err == 0:
            print("Lua script running - DI0->DO0 passthrough active")
        else:
            print(f"ProgramRun failed: {err}")

    elif cmd == 'stop':
        err = robot.ProgramStop()
        if err == 0:
            print("Program stopped")
        else:
            print(f"ProgramStop failed: {err}")

    elif cmd == 'status':
        err, state = robot.GetProgramState()
        states = {1: 'stopped', 2: 'running', 3: 'paused'}
        if err == 0:
            print(f"Program state: {states.get(state, state)}")
        else:
            print(f"GetProgramState failed: {err}")

    elif cmd == 'list':
        err, count, names = robot.GetLuaList()
        if err == 0:
            print(f"{count} Lua files on robot:")
            for name in names:
                if name:
                    print(f"  - {name}")
        else:
            print(f"GetLuaList failed: {err}")

    else:
        print(f"Unknown command: {cmd}")
        print("Usage: python lua_manager.py [upload|start|stop|status|list]")


if __name__ == '__main__':
    main()
