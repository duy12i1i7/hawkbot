# #!/usr/bin/python3
# # encoding:utf-8
import sys

from .HBSDK.HBSDK import run


def main():
    run(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

if __name__ == '__main__':
    main()
