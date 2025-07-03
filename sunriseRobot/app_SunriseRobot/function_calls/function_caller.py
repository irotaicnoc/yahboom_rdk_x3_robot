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

    def call_function(self, function_name: str, kwargs) -> None:
        """
        Calls the function with the given arguments and keyword arguments.
        """
        method = getattr(self.available_functions, function_name, None)
        if method is None:
            warnings.warn(f'Method "{function_name}" not found in available functions.')
        print(f'Calling method: {method.__name__}')

        if kwargs is None or len(kwargs) == 0:
            print('\twithout arguments.')
            method()
        else:
            print(f'\twith arguments {kwargs}.')
            method(**kwargs)
