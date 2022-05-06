__version__ = '0.1.0'

import rich.traceback

from loguru import logger as log

rich.traceback.install(show_locals=False)
