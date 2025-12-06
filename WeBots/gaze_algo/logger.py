# logger.py
import logging
def get_logger(name=__name__):
    logger = logging.getLogger(name)
    if not logger.handlers:
        ch = logging.StreamHandler()
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
        ch.setFormatter(fmt)
        logger.addHandler(ch)
        logger.setLevel(logging.INFO)
    return logger
