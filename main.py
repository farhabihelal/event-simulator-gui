import logging
import os
import sys

logging.basicConfig()

def is_venv():
    return sys.exec_prefix != sys.base_prefix

def get_python_exec(venv = None):
    return sys.executable if is_venv() or not venv else f"{venv}/bin/python"

def get_args(args):
    argv = []
    if args.daemon:
        argv.append("--daemon")
    return argv

if __name__ == "__main__":

    import argparse
    import subprocess

    parser = argparse.ArgumentParser()

    parser.add_argument("--daemon", action="store_true", dest="daemon", default=False, help="Daemon mode suppress extra console outputs.")
    parser.add_argument("--venv", type=str, dest="venv", help="Virtual environment to use.")
    parser.add_argument("--script", type=str, dest="script", help="Script to run.")

    args, unknown_args = parser.parse_known_args()

    script = os.path.join(os.path.dirname(__file__), args.script)
    python_exec = get_python_exec(args.venv)

    process_info = [python_exec, script] + get_args(args)
    try:
        process = subprocess.Popen(process_info)
        return_code = process.wait()
        exit(return_code)

    except KeyboardInterrupt as e:
        logging.info("Preparing to shutdown...")
    except Exception as e:
        logging.exception(e)

