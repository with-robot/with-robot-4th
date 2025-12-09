from pprint import pprint

import requests

from ..common.logger import get_logger

logger = get_logger(__name__)


class TaskExecutor:
    def __init__(self, url="http://127.0.0.1:8800"):
        response = requests.get(f"{url}/env")
        self.object_map = response.json().get("objects", [])
        self.url = url

    def _make_task_sequence(self, task_outputs):
        task_sequence = []

        for task_output in task_outputs:
            subgoal = task_output["subgoal"]
            tasks = task_output["tasks"]
            for task in tasks:
                task_sequence.append(task)

        return task_sequence

    def _go_to_object(self, target):
        pos = target["pos"]
        payload = {
            "action": {
                "type": "run_code",
                "payload": {
                    "code": f"""
path = plan_mobile_path({pos})
result = follow_mobile_path(path)
"""
                },
            }
        }
        response = requests.post(f"{self.url}/send_action", json=payload)
        objects = response.json()["result"]
        print(objects)

    def _pick_object(self, target):
        pos = target["pos"]
        payload = {
            "action": {
                "type": "run_code",
                "payload": {
                    "code": f"""
result = pick_object({pos}, 0.1, 0.2)
"""
                },
            }
        }
        response = requests.post(f"{self.url}/send_action", json=payload)
        objects = response.json()["result"]
        print(objects)

    def _place_object(self, target):
        pos = target["pos"]
        payload = {
            "action": {
                "type": "run_code",
                "payload": {
                    "code": f"""
result = place_object({pos}, 0.1, 0.2)
"""
                },
            }
        }
        response = requests.post(f"{self.url}/send_action", json=payload)
        objects = response.json()["result"]
        print(objects)

    def execute(self, task_outputs):
        task_sequence = self._make_task_sequence(task_outputs)
        logger.info("Executing task sequence:")
        pprint(task_sequence)

        results = []
        for task in task_sequence:
            logger.info(f"Executing task: {task}")
            if task["skill"] == "GoToObject":
                target = self.object_map[task["target"]]
                self._go_to_object(target)
            elif task["skill"] == "PickObject":
                target = self.object_map[task["target"]]
                self._pick_object(target)
            elif task["skill"] == "PlaceObject":
                target = self.object_map[task["target"]]
                self._place_object(target)
            else:
                raise ValueError(f"Unknown skill: {task['skill']}")
            task_result = task.copy()
            task_result["result"] = "Ok"
            results.append(task_result)

        logger.info("Task sequence execution completed.")
        return results
