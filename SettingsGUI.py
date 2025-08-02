import tkinter as tk
from tkinter import messagebox


class SettingsGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("הגדרות מצלמה")

        # ערכי ברירת מחדל
        self.settings = {
            "FRAME_WIDTH": 640,
            "FRAME_HEIGHT": 480,
            "MAX_SERVO_ANGLE": 5,
            "TOP_STOP": 70,
            "BOTTOM_STOP": 70,
            "LEFT_STOP": 100,
            "RIGHT_STOP": 100,
            "BOTTOM_DISTANCE_STOP": 0.2,
            "TOP_DISTANCE_STOP": 0.2,
            "FRONT_DISTANCE_STOP": 0.2
        }

        self.entries = {}

        row = 0
        for key, value in self.settings.items():
            tk.Label(self.root, text=key).grid(row=row, column=0, padx=10, pady=5, sticky="e")
            entry = tk.Entry(self.root)
            entry.insert(0, str(value))
            entry.grid(row=row, column=1, padx=10, pady=5)
            self.entries[key] = entry
            row += 1

        tk.Button(self.root, text="אישור", command=self.submit).grid(row=row, column=0, columnspan=2, pady=10)

        self.result = None

    def submit(self):
        try:
            self.result = {
                key: self._convert_value(key, entry.get())
                for key, entry in self.entries.items()
            }
            self.root.destroy()
        except ValueError as e:
            messagebox.showerror("שגיאה", f"שגיאה בהזנה: {e}")

    def _convert_value(self, key, value):
        if "DISTANCE" in key:
            return float(value)
        else:
            return int(value)

    def run(self):
        self.root.mainloop()
        return self.result
