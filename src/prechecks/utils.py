import sys
import time


def print_progress(iteration, total, prefix='', suffix='', decimals=1, bar_length=100):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        bar_length  - Optional  : character length of bar (Int)
    """
    str_format = "{0:." + str(decimals) + "f}"
    percents = str_format.format(100 * (iteration / float(total)))
    filled_length = int(round(bar_length * iteration / float(total)))
    bar = '█' * filled_length + '-' * (bar_length - filled_length)

    if len(prefix) < 65:
        prefix += ' ' * (65 - len(prefix))

    sys.stdout.write(f'\r{prefix} |{bar}| {percents}% {suffix}')

    if iteration == total:
        sys.stdout.write('\n')
    sys.stdout.flush()


def time_func_call(func):
    """
    Prints the processing time of the wrapped function after its context is left.
    :param func: Function to be wrapped.

    Usage:
    @time_func_call
    def func():
        pass
    """

    def wrapped_func(*args, **kwargs):
        start = time.process_time()
        try:
            result = func(*args, **kwargs)
        finally:
            print(f'Finished in {time.process_time() - start} seconds.')
        return result

    return wrapped_func
