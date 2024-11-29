import os
import psutil


def kill_process_(process_name: str, verbose: int = 0):
    target_found = True
    while target_found:
        target_found = False
        process_list = psutil.process_iter()
        if verbose >= 2:
            print(f'Killing process "{process_name}"...')
        for process in process_list:
            if process_name in process.name():
                target_found = True
                if verbose >= 1:
                    print(f'\t\t{process.name()} is running')
                os.kill(process.pid, 9)
                if verbose >= 2:
                    print(f'\t\t{process.name()} killed')
                os.system('sleep 0.1')
