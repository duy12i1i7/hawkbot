# #!/usr/bin/python3
# # encoding:utf-8
import sys
import  os
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
from HBSDK_decompiled.HBSDK import run
def main():
    run(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

if __name__ == '__main__':
    main()