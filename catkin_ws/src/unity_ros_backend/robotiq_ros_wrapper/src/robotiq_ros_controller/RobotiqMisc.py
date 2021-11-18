#!/usr/bin/env python3

from ur_remote_dashboard.URMisc import PrintColor

rbt_pcolor = PrintColor()
    
def rbt_log_error(msg):
    print(f'{rbt_pcolor.flags["ERROR"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_log_success(msg):
    print(f'{rbt_pcolor.flags["SUCCESS"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_log(msg):
    print(f'{rbt_pcolor.flags["LOG"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')

def rbt_log_warn(msg):
    print(f'{rbt_pcolor.flags["WARNING"]}[ROBOTIQ] {msg}{rbt_pcolor.effects["RESET"]}')