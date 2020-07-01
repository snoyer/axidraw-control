from functools import partial
import logging
import argparse
import json
import os
import threading

import npyscreen, curses

from axidrawcontrol import  SerialEbb
from axidrawcontrol.axidraw import CommandsBuilder, parse_settings
from axidrawcontrol.plotter import do_plot

sample = '''
M 6.72,6.53 C 7.15,5.88 7.59,5.23 7.91,4.51 C 8.02,4.21 8.14,3.91 8.09,3.59 C 7.96,3.35 7.78,3.41 7.59,3.55 C 7.42,3.65 7.35,3.90 7.28,4.08 C 6.97,5.20 6.86,6.37 6.72,7.52 C 6.86,7.00 7.20,6.39 7.42,6.08 C 7.63,5.77 7.78,5.72 8.00,5.69 C 8.22,5.67 8.36,5.75 8.45,6.03 C 8.54,6.32 8.35,6.66 8.29,6.93 C 8.24,7.20 8.24,7.37 8.46,7.49 C 8.90,7.60 9.34,7.31 9.69,7.11 C 9.96,6.96 10.29,6.78 10.39,6.50 C 10.49,6.22 10.44,5.88 10.22,5.77 C 10.01,5.66 9.72,5.82 9.56,6.05 C 9.40,6.28 9.35,6.66 9.40,6.94 C 9.44,7.23 9.71,7.41 9.97,7.49 C 10.73,7.60 11.27,6.93 11.55,6.52 C 11.88,6.05 12.33,5.13 12.62,4.40 C 12.72,4.14 12.84,3.87 12.76,3.59 C 12.68,3.31 12.32,3.45 12.17,3.62 C 11.93,3.96 11.87,4.38 11.79,4.77 C 11.70,5.27 11.60,6.03 11.58,6.28 C 11.30,8.24 12.54,7.35 13.12,6.52 C 13.46,6.02 13.90,5.12 14.20,4.40 C 14.29,4.14 14.41,3.87 14.33,3.59 C 14.26,3.31 13.89,3.45 13.74,3.62 C 13.50,3.96 13.44,4.38 13.36,4.77 C 13.27,5.27 13.17,6.03 13.15,6.28 C 13.13,6.54 13.00,7.12 13.37,7.40 C 13.73,7.68 14.17,7.02 14.53,6.53
M 15.70,5.75 C 15.47,5.73 15.24,5.74 15.04,5.88 C 14.76,6.01 14.59,6.32 14.53,6.60 C 14.49,6.84 14.57,7.06 14.70,7.27 C 14.83,7.40 15.05,7.52 15.25,7.52 C 15.53,7.56 15.80,7.41 16.00,7.23 C 16.15,7.08 16.25,6.86 16.29,6.66 C 16.34,6.41 16.24,6.19 16.12,5.97 C 15.97,5.91 15.73,5.64 15.61,5.85 C 15.44,5.97 15.51,6.23 15.54,6.40 C 15.62,6.61 15.72,6.80 15.95,6.85 C 16.17,7.00 16.44,6.90 16.69,6.93 C 16.91,6.82 17.13,6.74 17.27,6.53
M 5.70,10.58 C 5.36,10.93 5.08,11.40 5.12,11.91 C 5.14,12.13 5.31,12.36 5.55,12.33 C 5.92,12.31 6.23,12.06 6.46,11.78 C 6.56,11.60 6.60,11.39 6.67,11.20 C 6.74,10.99 6.81,10.78 6.88,10.58 C 6.75,11.05 6.51,11.50 6.47,12.00 C 6.47,12.19 6.73,12.37 7.04,12.34 C 7.35,12.31 7.60,12.07 7.83,11.76 C 8.05,11.38 8.07,10.98 8.06,10.59 C 8.16,10.88 8.13,11.22 8.41,11.44 C 8.68,11.66 8.96,11.50 9.24,11.36
M 10.41,10.58 C 9.95,10.50 9.47,10.78 9.31,11.22 C 9.16,11.53 9.24,11.94 9.49,12.17 C 9.83,12.45 10.36,12.39 10.67,12.09 C 10.99,11.81 11.13,11.32 10.91,10.95 C 10.81,10.75 10.48,10.47 10.28,10.70 C 10.15,10.90 10.22,11.18 10.32,11.38 C 10.43,11.65 10.74,11.76 11.00,11.76 C 11.30,11.79 11.59,11.72 11.83,11.52 C 12.06,11.31 12.21,11.02 12.38,10.76 C 12.47,10.66 12.62,10.20 12.57,10.55 C 12.52,10.94 12.95,10.70 13.16,10.80 C 13.53,10.95 13.37,11.46 13.25,11.71 C 13.12,11.97 13.14,12.34 13.46,12.35 C 13.87,12.29 14.21,12.21 14.56,11.65 C 14.90,11.09 15.53,9.95 15.75,9.38 C 15.97,8.80 16.08,8.37 15.75,8.26 C 15.43,8.15 15.24,8.50 15.16,8.76 C 14.91,9.50 14.79,10.67 14.74,11.06 C 14.70,11.45 14.74,12.15 14.74,12.15 C 14.91,12.81 15.83,11.75 16.12,11.36
M 17.88,11.17 C 17.66,10.72 17.35,10.47 16.84,10.61 C 16.54,10.72 16.28,10.94 16.17,11.25 C 16.04,11.66 16.31,12.18 16.59,12.29 C 17.00,12.43 17.51,12.26 17.63,11.88 C 17.95,11.06 18.19,10.22 18.47,9.39 C 18.60,9.00 18.73,8.61 18.86,8.22 L 18.08,10.58 C 17.98,11.08 17.70,11.65 17.88,12.11 C 18.07,12.56 18.65,12.17 18.87,11.78
M 4.09,2.10 C 2.99,2.10 2.10,2.99 2.10,4.09 L 2.10,11.99 C 2.10,13.10 2.99,13.98 4.09,13.98 L 10.92,13.98 C 10.67,15.14 9.59,16.17 8.62,17.21 C 10.82,16.29 12.74,15.29 13.44,13.98 L 19.90,13.98 C 21.00,13.98 21.89,13.10 21.89,11.99 L 21.89,4.09 C 21.89,2.99 21.00,2.10 19.90,2.10 L 4.09,2.10
'''.strip().splitlines()

class MyTestApp(npyscreen.NPSAppManaged):
    def __init__(self, ebb=None, filename=None, stop_event=None, *args, **kwargs):
        self.ebb = ebb
        self.filename = filename
        self.stop_event = stop_event
        self.form = None
        self.log = ''
        super().__init__(*args, **kwargs)

    def onStart(self):
        npyscreen.setTheme(npyscreen.Themes.TransparentThemeDarkText)
        self.form = MainForm(ebb=self.ebb, filename=self.filename, stop_event=self.stop_event)
        self.registerForm("MAIN", self.form)


    def write(self, xxx):
        self.log += xxx
        ml = self.form.multiline
        ml.value = '\n'.join(self.log.splitlines()[-ml.height:])
        ml.update()

    def flush(self):
        self.form.multiline.display()

class MainForm(npyscreen.FormWithMenus):
    def __init__(self, ebb=None, filename=None, stop_event=None, *args, **kwargs):
        self.ebb = ebb
        self.stop_event = stop_event
        self.filename = filename
        self.axidraw = CommandsBuilder()
        
        self.settings_values = dict(
            pen_up='80%',
            pen_down='20%',
            max_velocity='50%',
            acceleration='50%',
            cornering='50%',
        )
        if filename and os.path.isfile(filename):
            try:
                self.settings_values = json.load(open(filename))
            except json.JSONDecodeError:
                self.filename = None

        super().__init__(*args, **kwargs)

    def create(self):
        self.how_exited_handers[npyscreen.wgwidget.EXITED_ESCAPE] = self.exit_application


        live_adjust = self.add(npyscreen.Checkbox, name="live pen adjust.")


        def set_val_from_slider(name):
            def f(slider):
                val = slider.value
                self.settings_values[name] = str(val) + '%'
                if live_adjust.value:
                    if name == 'pen_up':
                        self.ebb.run(self.axidraw.raise_pen(val/100, speed=5))
                    if name == 'pen_down':
                        self.ebb.run(self.axidraw.lower_pen(val/100, speed=5))
            return f
        for key in [
            'pen_up', 'pen_down', 'max_velocity', 'acceleration', 'cornering',
        ]:
            name = key.replace('_', ' ').title()
            s  = self.add(npyscreen.TitleSliderPercent, name=name, accuracy=0)
            s.entry_widget.value = float(str(self.settings_values.get(key, 50)).rstrip('%'))
            set_callback_on_titled(s, set_val_from_slider(key))


        menu = self.add_menu(name="Actions", shortcut='^M')
        menu.do_pre_display_function = self.stop_plot
        menu.addItemsFromList([
            ("Plot Sample", self.do_sample_plot, 'p'),
            ("Save Settings", self.save_settings, 's'),
            ("Exit", self.exit_application, 'e'),
        ])


        self.multiline = self.add(npyscreen.MultiLineEdit,
           value = '',  editable=False,
        )



    def do_sample_plot(self):
        def task():
            try:
                def commands():
                    settings = parse_settings(self.settings_values)
                    for path in sample:
                        yield from self.axidraw.draw_path(path, settings)
                do_plot(self.ebb, commands(), file=self.parentApp, stop_event=self.stop_event)
            finally:
                self.ebb.run(('EM',0,0))

        self.stop_event.clear()
        threading.Thread(target=task).start()


    def stop_plot(self):
        self.stop_event.set()


    def save_settings(self):
        if self.filename:
            fn = self.filename
        else:
            fn = '/tmp/settings.json'
        json.dump(self.settings_values, open(fn, 'w'))
        npyscreen.notify_confirm("Saved settings as `%s`" % fn)



    def exit_application(self):
        # self.parentApp.setNextForm(None)
        # self.editing = False
        # self.parentApp.switchFormNow()
        self.parentApp.switchForm(None)


def set_callback_on_titled(widget, cb):
    if isinstance(widget, npyscreen.TitleText):
        widget = widget.entry_widget
    def make_cb(w):
        return cb.__get__(w, type(w))
    widget.when_value_edited = make_cb(widget)




if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('settings', nargs='?')
    argparser.add_argument('--port')
    argparser.add_argument('--logging', action='store_true')
    
    args = argparser.parse_args()

    
    try:
        stop_event = threading.Event()
        with SerialEbb(args.port) as ebb:
            app = MyTestApp(ebb=ebb, filename=args.settings, stop_event=stop_event)
            if args.logging:
                logging.basicConfig(stream=app, level=logging.DEBUG)
            app.run()
    except KeyboardInterrupt as e:
        pass
