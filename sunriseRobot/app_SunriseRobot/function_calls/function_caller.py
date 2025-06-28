import warnings

from function_calls.available_functions import AvailableFunctions


class FunctionCaller:
    """
    A class to handle function calls with a specific function name.
    """

    def __init__(self, robot_head, robot_body, arm=None, light=None, verbose: int = 0):
        self.available_functions = AvailableFunctions(
            robot_head=robot_head,
            robot_body=robot_body,
            arm=arm,
            light=light,
        )
        self.verbose = verbose
        for attr in dir(self.available_functions):
            if not attr.startswith('_'):
                if callable(getattr(self.available_functions, attr)):
                    print(f'available_functions contains callable attr:\n\t"{attr}".')

        if hasattr(self.available_functions, 'change_light_effect'):
            print('available_functions contains "change_light_effect" method.')
        else:
            print('available_functions does not contain "change_light_effect" method.')
        if hasattr(self.available_functions, 'next_target'):
            print('available_functions contains "next_target" method.')
        else:
            print('available_functions does not contain "next_target" method.')
        if hasattr(self.available_functions, 'set_speed'):
            print('available_functions contains "set_speed" method.')
            set_speed = getattr(self.available_functions, 'set_speed')
            set_speed(speed_x=-0.1)
        else:
            print('available_functions does not contain "set_speed" method.')

    def call_function(self, function_name: str, kwargs) -> None:
        """
        Calls the function with the given arguments and keyword arguments.
        """
        method = getattr(self.available_functions, function_name, None)
        if method is None:
            warnings.warn(f'Method "{function_name}" not found in available functions.')
        print(f'Calling method: {method.__name__}')

        if kwargs is None:
            print('\twithout arguments.')
            method()
        else:
            print(f'\twith arguments {kwargs}.')
            method(**kwargs)
