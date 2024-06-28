import logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logger.setLevel(level="INFO")

__version__ = "0.1.0"
