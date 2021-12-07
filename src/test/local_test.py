import sys as _sys
_sys.path.append("../")

from local_navigation.py import *

from tdmclient import ClientAsync, aw
client = ClientAsync()
node = await client.wait_for_node()
await node.lock()
await node.wait_for_variables()

await obstacle_avoidance(node, client)
