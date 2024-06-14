import os
import psutil


def kill_process_(program_name: str, debug: bool = False):
    process_list = psutil.process_iter()
    for process in process_list:
        if program_name in process.name():
            if debug:
                print(f"\t\t{process.name()} is running")
            os.kill(process.pid, 9)
            if debug:
                print(f"\t\t{process.name()} killed")
            os.system("sleep 0.1")
