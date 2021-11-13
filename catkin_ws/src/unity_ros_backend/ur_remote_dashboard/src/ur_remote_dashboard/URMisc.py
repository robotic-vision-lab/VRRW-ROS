#!/usr/bin/env python3

class PrintColor():
    HEADER = '\033[95m'
    
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
    # dracula color palette (ANSI RGB)
    # https://draculatheme.com/contribute
    DCL_CYAN = '\033[38;2;139;233;253m'
    DCL_GREEN = '\033[38;2;80;250;123m'
    DCL_ORANGE = '\033[38;2;255;184;108m'
    DCL_PINK = '\033[38;2;255;121;198m'
    DCL_PURPLE = '\033[38;2;189;147;249m'
    DCL_RED = '\033[38;2;255;85;85m'
    DCL_YELLOW = '\033[38;2;241;250;140m'
    
    # tokyo night
    # https://github.com/enkia/tokyo-night-vscode-theme