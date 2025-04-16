# Terminal_UAV.py

import time
from config import Config

class TerminalUAV:
    def __init__(self):
        self.status = 'Idle'
        self.task = None
        print("Terminal UAV initialized. Status: Idle")

    def assign_task(self, task_info):
        if self.status == 'Idle':
            self.task = task_info
            self.status = 'Assigned'
            print(f"Task assigned: {self.task}")
        else:
            print("UAV currently busy with another task.")

    def execute_task(self):
        if self.status == 'Assigned':
            self.status = 'Executing'
            print(f"Executing task: {self.task}")
            # 模拟任务执行过程
            execution_time = self.task.get('duration', 5)
            time.sleep(execution_time)
            self.status = 'Completed'
            print(f"Task completed: {self.task}")
        elif self.status == 'Idle':
            print("No task assigned.")
        else:
            print(f"Cannot execute task. Current status: {self.status}")

    def report_status(self):
        print(f"Current UAV status: {self.status}")
        return self.status


# 测试TerminalUAV
if __name__ == "__main__":
    uav_terminal = TerminalUAV()

    task_example = {
        'task_id': 'Delivery001',
        'description': 'Deliver package to location X.',
        'duration': 3
    }

    uav_terminal.assign_task(task_example)
    uav_terminal.report_status()

    uav_terminal.execute_task()
    uav_terminal.report_status()
