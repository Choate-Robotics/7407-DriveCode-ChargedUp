import datetime


class Logger:
    def __init__(
        self,
        debug: bool = False,
        filename: str = f"custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
    ):
        self.filename: str = filename
        self.logfile = None
        self.debug_on = debug

    def log(self, system: str, message: str):
        self.logfile = open(self.filename, "w")
        self.logfile.write(f"[{system}] ~ {message}\n")
        self.logfile.close()

    def debug(self, system: str, message: str):
        if self.debug_on:
            self.logfile = open(self.filename, "a")
            self.logfile.write(f"[{system}] ~ {message}\n")
            self.logfile.close()

    def close(self):
        self.logfile.close()


logger = Logger(debug=True)
