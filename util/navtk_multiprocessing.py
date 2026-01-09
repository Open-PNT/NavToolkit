#!/usr/bin/env python3

from subprocess import Popen
from multiprocessing import cpu_count
from os import environ


class MultiProcessManager:
    '''
    A class which can be used to start a limited number of jobs. If the user
    doesn't provide a job limit, then the value of MAX_CPU_COUNT will be
    checked. If that environment variable is not set, then it will default to
    the CPU count.
    '''

    def __init__(self, max_num_processes=None):
        # Use parameter if provided
        if max_num_processes is None:

            # Use environment variable if provided
            env_value = environ.get('MAX_CPU_COUNT')
            if env_value is not None:
                max_num_processes = int(env_value)
            else:
                # Fall back to CPU count
                max_num_processes = cpu_count()

        self.max_num_processes = max_num_processes
        self.processes = []
        self.exit_code = 0

    def enforce_process_limit(self, num_processes=None):
        '''
        Pauses execution until the number of active processes is equal to or
        less than num_processes.
        '''
        if num_processes is None:
            num_processes = self.max_num_processes
        while len(self.processes) >= num_processes:
            for process in self.processes:
                if process.poll() is not None:
                    self.exit_code = process.returncode or self.exit_code
                    self.processes.remove(process)
                    break

    def run_process(self, args):
        '''
        Checks the number of active processes and pauses execution until they
        are at or below the configured limit. Then starts a new process with
        the given arguments.
        '''
        self.enforce_process_limit()
        self.processes.append(Popen(args))

    def finish_processes(self):
        '''
        Pauses execution until all started processes have completed.
        '''
        self.enforce_process_limit(1)
