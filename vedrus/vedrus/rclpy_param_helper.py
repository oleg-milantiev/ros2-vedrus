from rclpy.node import Node
import rclpy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

# node: current node handle
# node_name: name of node for which you need to set params
# param_dict: dict of params that need to be set {'param_name': param_value}
def call_set_parameters(node:Node, node_name:str, param_dict:dict) -> bool:
  parameters=[]
  for key,value in param_dict.items():
    value_type = str(type(value)).lower()
    if 'None' in value_type:
      node.get_logger().error(f"{value_type} is not supported!")
    elif 'bool' in value_type:
      parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)))
    elif 'int' in value_type:
      parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)))
    elif 'float' in value_type:
      parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)))
    elif 'str' in value_type:
      parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)))
    elif 'list' in value_type:
      if all(isinstance(n, bool) for n in value):
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY, bool_array_value=value)))
      elif all(isinstance(n, int) for n in value):
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=value)))
      elif all(isinstance(n, float) for n in value):
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=value)))
      elif all(isinstance(n, str) for n in value):
        parameters.append(Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=value)))
    else:
      node.get_logger().error(f"{value_type} is not supported!")
      raise TypeError(f"{value_type} is not supported!")

  # create client
  client = node.create_client(SetParameters, f'{node_name}/set_parameters')

  # call as soon as ready
  ready = client.wait_for_service(timeout_sec=5.0)
  if not ready:
      raise RuntimeError('Wait for service timed out')

  request = SetParameters.Request()
  request.parameters = parameters
  future = client.call_async(request)
  '''
  TBD ниработает. И фиг с ним, с результатом
  
  rclpy.spin_until_future_complete(node, future)

  # handle response
  response = future.result()
  if response is None:
      e = future.exception()
      raise RuntimeError(
          f"Exception while calling service of node '{node_name}': {e}")
  if all(n.successful for n in response.results[:]): return True

  return False
  '''

# node: current node handle
# node_name: name of node for which you need to get params
# param_list: list of params that need to be get ['param_1','param_2']
def call_get_parameters(node:Node, node_name:str, parameter_names:list)->dict:
  param_dict = {}
  # create client
  client = node.create_client(GetParameters, f'{node_name}/get_parameters')

  # call as soon as ready
  ready = client.wait_for_service(timeout_sec=5.0)
  if not ready:
      raise RuntimeError('Wait for service timed out')

  request = GetParameters.Request()
  request.names = parameter_names
  future = client.call_async(request)
  rclpy.spin_until_future_complete(node, future)

  # handle response
  response = future.result()
  if response is None:
      e = future.exception()
      raise RuntimeError(
          f"Exception while calling service of node '{node_name}': {e}")
  for i in range(len(response.values)):#request.values[:]:
    if response.values[i].type==1:
      param_dict[parameter_names[i]]=response.values[i].bool_value
    elif response.values[i].type==2:
      param_dict[parameter_names[i]]=response.values[i].integer_value
    elif response.values[i].type==3:
      param_dict[parameter_names[i]]=response.values[i].double_value
    elif response.values[i].type==4:
      param_dict[parameter_names[i]]=response.values[i].string_value
    elif response.values[i].type==5:
      param_dict[parameter_names[i]]=response.values[i].byte_array_value
    elif response.values[i].type==6:
      param_dict[parameter_names[i]]=response.values[i].bool_array_value
    elif response.values[i].type==7:
      param_dict[parameter_names[i]]=response.values[i].integer_array_value
    elif response.values[i].type==8:
      param_dict[parameter_names[i]]=response.values[i].double_array_value
    elif response.values[i].type==9:
      param_dict[parameter_names[i]]=response.values[i].string_array_value
    else:
      node.get_logger().error(f"Type {response.values[i].type} is not supported!")
      raise TypeError(f"Type {response.values[i].type} is not supported!")

  return param_dict