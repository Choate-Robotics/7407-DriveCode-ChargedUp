import datetime
import inspect
import os

import robotpy_toolkit_7407.utils.logger as lg


class Logger:
    def __init__(
        self,
        debug: bool = False,
        filename: str = f"custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
    ):
        self.filename: str = filename
        self.logfile = None
        self.debug_on = debug

        self.start_time = datetime.datetime.now()

    def log(self, system: str, message: str):
        self.logfile = open(self.filename, "a")

        frame = inspect.stack()[1].frame
        file_name = os.path.basename(frame.f_code.co_filename)
        line_no = str(frame.f_lineno)

        self.logfile.write(
            f"[{str(datetime.datetime.now() - self.start_time) + ']'} [{file_name + ':' + line_no + ']' : <19} [{system + ']'  : <15} ~ {message  : <20}\n"
        )
        self.logfile.close()

        lg.info(message, system, frame)

    def debug(self, system: str, message: str):
        if self.debug_on:
            self.logfile = open(self.filename, "a")

            frame = inspect.stack()[1].frame
            file_name = os.path.basename(frame.f_code.co_filename)
            line_no = str(frame.f_lineno)

            self.logfile.write(
                f"[{str(datetime.datetime.now() - self.start_time) + ']'} [{file_name + ':' + line_no + ']' : <19} [{system + ']'  : <15} ~ {message  : <20}\n"
            )
            self.logfile.close()

            lg.info(message, system, frame)

    def close(self):
        self.logfile.close()


logger = Logger(debug=True)
