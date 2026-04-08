# #!/usr/bin/python3
# # encoding:utf-8
import sys
import importlib.util
from pathlib import Path


# Load the compiled extension explicitly to avoid importing the Python fallback file.
_hbsdk_so = Path(__file__).resolve().parent / 'HBSDK' / 'HBSDK.so'
_hbsdk_spec = importlib.util.spec_from_file_location('hawkbot.HBSDK.HBSDK', _hbsdk_so)
if _hbsdk_spec is None or _hbsdk_spec.loader is None:
    raise ImportError(f'Cannot load HBSDK extension module from {_hbsdk_so}')

_hbsdk_module = importlib.util.module_from_spec(_hbsdk_spec)
_hbsdk_spec.loader.exec_module(_hbsdk_module)
run = _hbsdk_module.run


def main():
    run(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

if __name__ == '__main__':
    main()
