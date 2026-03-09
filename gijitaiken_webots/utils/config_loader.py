import json
import os
# import time

class ConfigLoader:
    def __init__(self, node, file_path):
        self.node = node
        self.file_path = file_path
        self.last_mtime = 0
        self.data = {}
        
        self.check_reload()

    def check_reload(self):
        try:
            if not os.path.exists(self.file_path):
                print(f"[CONFIG] Not Found: {self.file_path}")
                return False

            current_mtime = os.path.getmtime(self.file_path)
            if current_mtime > self.last_mtime:
                print("[CONFIG] Reloaded: joint_directions.json")
                with open(self.file_path, 'r') as f:
                    self.data = json.load(f)
                self.last_mtime = current_mtime
                return True
        except Exception as e:
            print(f"[CONFIG] Error: {e}")
        return False

    def get_data(self):
        return self.data