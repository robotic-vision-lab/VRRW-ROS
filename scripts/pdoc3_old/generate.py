#!/usr/bin/env python3

import roslib
import os, sys
import pdoc
import re
from pprint import pprint
from contextlib import contextmanager

__pdoc__ = {
    'rvl_robotiq_controller.msg' : False,
    'rvl_ur_robotiq.rvl_robotiq_controller.setup' : False,
    'rvl_ur_robotiq.rvl_robotiq_controller.scripts.RobotiqControllerTest' : False,
}

sys.path.append('/root/catkin_ws/devel/lib/python3/dist-packages/rvl_robotiq_controller/msg/')
sys.path.append('/root/catkin_ws/src/rvl_ur_robotiq/rvl_robotiq_controller/src/rvl_robotiq_controller')

modules = ['rvl_ur_remote_dashboard', 'rvl_robotiq_controller', 'rvl_utilities']  # Public submodules are auto-imported
context = pdoc.Context()

modules = [pdoc.Module(mod, context=context)
           for mod in modules]
pdoc.link_inheritance(context)

@contextmanager
def _open_write_file(filename):
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            yield f
            print(filename)  # print created file path to stdout
    except Exception:
        try:
            os.unlink(filename)
        except Exception:
            pass
        raise

def module_path(m: pdoc.Module, ext: str):
    return os.path.join(os.path.abspath('.'), *re.sub(r'\.html$', ext, m.url()).split('/'))

def recursive_write_files(m: pdoc.Module, ext: str, **kwargs):
    assert ext in ('.html', '.md')
    filepath = module_path(m, ext=ext)

    dirpath = os.path.dirname(filepath)
    if not os.access(dirpath, os.R_OK):
        os.makedirs(dirpath)

    with _open_write_file(filepath) as f:
        if ext == '.html':
            f.write(m.html(**kwargs))
        elif ext == '.md':
            f.write(m.text(**kwargs))

    for submodule in m.submodules():
        recursive_write_files(submodule, ext=ext, **kwargs)
        
for mod in modules:
    recursive_write_files(mod, '.html')