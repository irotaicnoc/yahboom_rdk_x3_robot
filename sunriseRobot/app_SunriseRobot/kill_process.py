import os
import psutil


def kill_process_(program_name: str, verbose: bool = False):
    target_found = True
    while target_found:
        target_found = False
        process_list = psutil.process_iter()
        if verbose:
            print(f'Killing for {program_name}...')
        for process in process_list:
            if program_name in process.name():
                target_found = True
                if verbose:
                    print(f'\t\t{process.name()} is running')
                os.kill(process.pid, 9)
                if verbose:
                    print(f'\t\t{process.name()} killed')
                os.system('sleep 0.1')
