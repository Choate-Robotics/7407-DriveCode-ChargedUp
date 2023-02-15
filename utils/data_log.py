import datetime

from robotpy_toolkit_7407.utils.data_log import Logger

logger = Logger(
    debug=False,
    filename=f"custom_logs/custom_logging_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
)
