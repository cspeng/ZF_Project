#!/usr/bin/env python 
# -*- coding: utf-8 -*- 

import curses 
import curses.textpad
import cmd


def maketextbox(h, w, y, x, title="", style="frame", stylepair=1):

    stdscr = curses.initscr()
    curses.noecho()
    stdscr.clear()

    nwin = curses.newwin(h, w, y, x)

    txtbox = curses.textpad.Textbox(nwin, insert_mode=True)

    if style == "frame":
        curses.textpad.rectangle(stdscr, y-1, x-1, y+h, x+w)
    elif style == "underline":
        stdscr.hline(y+1, x, '_', w, stylepair)

    nwin.attron(stylepair)
    stdscr.addstr(y-1, x+w/5, title, stylepair)
    stdscr.refresh()

    return stdscr, nwin, txtbox


class Commands(cmd.Cmd):
    """Simple command processor example."""
        
    def __init__(self, intro=""):
        cmd.Cmd.__init__(self)
        self.intro = intro

    def do_greet(self, line):
        self.write("hello "+line)

    def do_hello(self, line):
        self.write("hello "+line)

    def do_help(self, line):
        cmdlist = ('greet', 'help', 'quit')
        self.write("command: {}".format(cmdlist))

    def do_quit(self, line):
        curses.endwin()
        return True

    def default(self, line):
        self.write("Don't understand '" + line + "'")

    def write(self, text):
        stdscr.addstr(y+2, x, " " * 40)
        stdscr.addstr(y+2, x, text)
        stdscr.refresh()


if __name__ == '__main__':
    intro = "TRIO-AGV on Cloud-112"
    is_quit = False
    x, y = (20, 2)

    stdscr, nwin, textbox = maketextbox(1, 40, y, x, intro)

    while not is_quit:
        try:

            nwin.clear()
            text = textbox.edit()
            curses.beep()
            is_quit = Commands(intro).onecmd(text)

        except KeyboardInterrupt:
            print "stop on ctrl-C"
            is_quit = True

        finally:
            curses.endwin()
