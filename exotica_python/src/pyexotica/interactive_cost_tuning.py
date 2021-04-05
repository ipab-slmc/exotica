from __future__ import print_function
import os, sys
from time import time

if sys.version_info >= (3, 0):
    import tkinter as tk
else:
    import Tkinter as tk

# import rospkg as rp

__all__ = ['InteractiveCostTuning']

class InteractiveCostTuning(object):
    
    def __init__(self, problem):

        # Initial setup
        self.master = tk.Tk()
        self.problem = problem

        # Set title
        self.master.winfo_toplevel().title("Interactive Cost Tuning")

        # Set icon
        # icon = rp.RosStack().get_path('exotica') + '/doc/images/EXOTica_icon.png'
        # img = tk.PhotoImage(file=icon)
        # self.master.tk.call('wm', 'iconphoto', self.master._w, img)

        # Grab current rhos and cost task map names
        self.rho = {}
        self.original_rho = {}
        self.cost_task_map_names = []
        for k in list(problem.get_task_maps().keys()):
            try:
                r = problem.get_rho(k)
                self.rho[k] = r
                self.original_rho[k] = r
                self.cost_task_map_names.append(k)
            except:
                continue

        # Setup labels and entries
        self.entries = {}
        for i, k in enumerate(self.cost_task_map_names):
            tk.Label(self.master, text=k).grid(row=i, column=0)
            self.entries[k] = tk.Entry(self.master)
            self.entries[k].grid(row=i, column=1, pady=4)
            self.entries[k].insert(0, self.rho[k])

        n_cost_task_maps = len(self.cost_task_map_names)
        tk.Label(self.master, text='Filename').grid(row=n_cost_task_maps, column=0, pady=4)
        self.entries['filename'] = tk.Entry(self.master)
        self.entries['filename'].grid(row=n_cost_task_maps, column=1, pady=4)
        self.entries['filename'].insert(0, 'FilenameHere')

        # Setup buttons
        tk.Button(self.master, text="Set", command=self.set_button).grid(row=n_cost_task_maps+1, column=0, pady=4)
        tk.Button(self.master, text="Save", command=self.save_button).grid(row=n_cost_task_maps+1, column=1, pady=4)
        tk.Button(self.master, text="Reset", command=self.reset_button).grid(row=n_cost_task_maps+2, column=0, pady=4)
        tk.Button(self.master, text="Quit", command=self.quit_button).grid(row=n_cost_task_maps+2, column=1, pady=4)

    def set_button(self):
        """Sets rho parameters in entries into Exotica problem."""
        print("Setting cost parameters:")
        for k in self.cost_task_map_names:
            userin = self.entries[k].get() # is a str
            rho = float(eval(userin))
            self.entries[k].delete(0, 'end')
            self.entries[k].insert(0, rho)
            self.problem.set_rho(k, rho)
            print("  {}\t{}".format(k, rho))

    def save_button(self):
        """Saves current rho parameters in entries to file in home dir."""

        # Generate filename, filename structure is FILENAMEINENTRY_TIMEINMS.costparams
        # Use time to avoid overwriting errors
        t = int(round(time() * 1000)) # time in ms (int)
        filename = "%s/%s_%d.costparams" % (os.environ['HOME'], self.entries['filename'].get(), t)

        # Save parameters
        with open(filename, 'w') as fout:
            fout.write('<Cost>\n')
            for k in self.cost_task_map_names:
                fout.write('  <Task Task="{}" Rho="{}"/>\n'.format(k, float(eval(self.entries[k].get()))))
            fout.write('</Cost>\n')
        print("Saved cost parameters to %s" % filename)

    def reset_button(self):
        """Resets entries/exotica to original cost terms as specified in xml."""
        print("Resetting cost parameters:")
        for k in self.cost_task_map_names:
            rho = self.original_rho[k]

            # Reset entries
            self.entries[k].delete(0, 'end')
            self.entries[k].insert(0, rho)

            # Reset exotica problem
            self.problem.set_rho(k, rho)

            print("  {}\t{}".format(k, rho))

    def quit_button(self):
        """Quits interactive cost tuning."""
        print("Quitting interactive cost tuning...")
        self.master.quit()

    def mainloop(self):
        """Starts tk mainloop."""
        tk.mainloop()
