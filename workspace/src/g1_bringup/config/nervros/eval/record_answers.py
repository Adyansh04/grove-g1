#!/usr/bin/env python3
"""Write what canopy answers for each kind of object as a `nervros-cli eval --robot` overlay.

Run in the container beside the stack, after `nervros-cli record`, and pass the output as a second
`--robot`. Python because it is a one-off helper around rclpy, as relocalize.py is.

    python3 record_answers.py /root/data/nervros-dev/eval-robot/answers.toml
"""

import json
import sys

import rclpy
from canopy_msgs.srv import FindObjects
from rosidl_runtime_py import message_to_ordereddict

SERVICE = "/canopy/find_objects"
# A model's own `max_results` is not replayed: a recording answers with this many.
LISTED = 20
NOTHING = "zzz-no-such-thing"


def toml_value(value):
    """Return a value as an inline TOML value."""
    if isinstance(value, dict):
        fields = ", ".join(f"{json.dumps(k)} = {toml_value(v)}" for k, v in value.items())
        return "{ " + fields + " }"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(toml_value(v) for v in value) + "]"
    return json.dumps(value)


def main():
    """Ask the world model for every label it knows and write the answers."""
    rclpy.init()
    node = rclpy.create_node("record_answers")
    client = node.create_client(FindObjects, SERVICE)
    if not client.wait_for_service(timeout_sec=30.0):
        sys.exit(f"{SERVICE} is not available")

    def ask(query, listed=LISTED):
        future = client.call_async(FindObjects.Request(query=query, max_results=listed))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        return future.result()

    def answer(query):
        return dict(message_to_ordereddict(ask(query)))

    known = ask("objects", 1000).objects
    words = sorted({o.label.lower() for o in known if o.label})
    with open(sys.argv[1], "w", encoding="utf-8") as out:
        out.write(f'[replies."{SERVICE}"]\nfield = "query"\n')
        out.write(f"otherwise = {toml_value(answer(NOTHING))}\n\n")
        out.write(f'[replies."{SERVICE}".by]\n')
        for word in words:
            out.write(f"{json.dumps(word)} = {toml_value(answer(word))}\n")
    print(f"{len(words)} kinds of object from {len(known)} objects")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
