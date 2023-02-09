import datetime

from robotpy_toolkit_7407.utils.data_log import Logger

logger = Logger(
    debug=True,
    filename=f"/home/crispy/Programming/choate_robotics/7407-DriveCode-ChargedUp/custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
)
