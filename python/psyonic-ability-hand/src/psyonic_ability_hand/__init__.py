__version__ = '0.1.0'

from rich.logging import RichHandler

from loguru import logger as log
log.configure(handlers=[{'sink':RichHandler(rich_tracebacks=True)}])
log.add('hand-serial-demo.log')
