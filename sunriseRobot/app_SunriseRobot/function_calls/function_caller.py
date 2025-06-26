import warnings

from function_calls import available_functions


class FunctionCaller:
    """
    A class to handle function calls with a specific function name.
    """

    def __init__(self, robot_head, robot_body, arm=None, light=None, verbose: int = 0):
        self.function_list = available_functions.function_list
        self.available_objects = available_functions.object_list
        self.robot_head = robot_head
        self.robot_body = robot_body
        self.arm = arm
        self.light = light
        self.verbose = verbose
        print(f'function list: {self.function_list}')
        print(f'available objects: {self.available_objects}')

    def find_function(self, function_name: str):
        """
        Finds the function in the available functions list and as attributes of object_list.
        :param function_name: The name of the function to find.
        :return: The function details if found, otherwise None.
        """
        for function in self.function_list:
            if function['name'] == function_name:
                return function
        for obj in self.available_objects:
            if function_name in obj['excluded_functions']:
                continue
            if hasattr(obj, function_name):
                return {
                    'name': function_name,
                    'containing_object': obj['name'],
                }
        return None

    def call_function(self, function_name: str, kwargs) -> None:
        """
        Calls the function with the given arguments and keyword arguments.
        """
        function = self.find_function(function_name=function_name)
        if function is None:
            warnings.warn(f'Function "{function_name}" not found in available functions.')
            return

        containing_object = function['containing_object']

        # Call the function on the containing object
        obj = getattr(self, containing_object, None)
        if obj is None:
            warnings.warn(f'Object "{containing_object}" for function calling not found.')

        method = getattr(obj, function_name, None)
        if method is None:
            warnings.warn(f'Method "{function_name}" not found in object "{containing_object}".')

        if kwargs is None:
            method()
        else:
            method(**kwargs)

        warnings.warn(f'Function "{function_name}" not found in available functions.')
