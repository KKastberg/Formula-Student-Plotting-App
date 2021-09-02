#!/usr/bin/env python3

# MISC
import numpy as np
from PIL import ImageTk, Image
import csv
from math import *  # required for easy parsing of new function inputs

# Tkinter
import tkinter as tk
import tkinter.ttk as ttk
import tkinter.font as tfont
from tkinter.filedialog import asksaveasfilename

# Matplotlib
import tikzplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import pyplot as plt
import matplotlib.animation as animation
plt.style.use("bmh")

# Global app variables
APP_TITLE = "Formula Student Graph Visualizer"
DEFAULT_PROJECT_NAME = "Demo Project"
APP_WINDOW_SIZE = (1000, 700)
KTHFS_IMAGE_PATH = "assets/kthfs.png"
SETTINGS_IMAGE_PATH = "assets/settings.png"

# Global Graph variables
PLOT_TITLE = "Plotting h(\u03BB(t))"
DEFAULT_INNER_FUNC = "5 * sin(2 * pi * 1 * t)"
DEFAULT_OUTER_FUNC = "3 * pi * exp(t)"
DEFAULT_FREQUENCY = 10
DEFAULT_GRID_STATE = True
DEFAULT_ZOOM = 15
DEFAULT_MAX_ZOOM = 30
DEFAULT_MIN_ZOOM = 0.1
DEFAULT_PLOT_ACCURACY = 0.01
PLOT_REFRESH_FREQUENCY = 10  # ms
STATIC_LINE_COLOR = "blue"
LIVE_LINE_COLOR = "red"

# Global Color variables
BACKGROUND_COLOR = "#ffffff"
MENU_COLOR = "#ececec"
TITLE_COLOR = "#003164"
ERROR_COLOR = "#ec0000"


class App(tk.Tk):
    """
    Root App class
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Configure app window
        self.title(APP_TITLE)

        # Set window size
        self.geometry(f"{APP_WINDOW_SIZE[0]}x{APP_WINDOW_SIZE[1]}")
        self.resizable(False, False)

        # Init grid layout to cover the whole window
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1, minsize=500)

        # Init the graph
        self.m_graph = Graph(parent=self)

        # Create the Graph Page
        self.graph_page = GraphPage(m_graph=self.m_graph, parent=self,
                                    master=self, bg=BACKGROUND_COLOR)
        self.graph_page.grid(row=0, column=0, stick="nsew")

        # Create the settings Page
        self.setting_page = SettingsPage(m_graph=self.m_graph, parent=self,
                                         master=self, bg=BACKGROUND_COLOR)
        self.setting_page.grid(row=0, column=0, stick="nsew")

        # Start app at graph page
        self.graph_page.tkraise()


class SettingsPage(tk.Frame):
    """
    The Settings Page displays a GUI for changing plotting parameters
    """

    def __init__(self, m_graph, parent, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent = parent
        self.m_graph = m_graph

        # Init GUI Entry variables
        self.setting_project_name_var = tk.StringVar()
        self.setting_project_name_var.set(DEFAULT_PROJECT_NAME)
        self.inner_func = tk.StringVar()
        self.inner_func.set(DEFAULT_INNER_FUNC)
        self.last_inner_func = self.inner_func.get()
        self.outer_func = tk.StringVar()
        self.outer_func.set(DEFAULT_OUTER_FUNC)
        self.last_outer_func = self.outer_func.get()
        self.plot_accuracy = tk.StringVar()
        self.plot_accuracy.set(str(DEFAULT_PLOT_ACCURACY))

        # Available settings
        self.parameters = ["Project Name", "Inner Function lambda(t)",
                           "Outer Function h(t)", "Plotting Accuracy"]
        self.parameter_vars = [self.setting_project_name_var, self.inner_func,
                               self.outer_func, self.plot_accuracy]
        self.entry_fields = []

        # Create layout
        title_frame = self.create_title_frame(self)
        title_frame.pack(side=tk.TOP, fill=tk.BOTH,
                         pady=(0, 0), padx=(0, 0))
        settings_frame = self.create_settings_frame(self)
        settings_frame.pack(side=tk.TOP, fill=tk.BOTH,
                            pady=(0, 0), padx=(0, 0))
        image_frame = self.create_image_frame(self)
        image_frame.pack(side=tk.BOTTOM, fill=tk.BOTH,
                         pady=(0, 0), padx=(0, 0))
        save_cancel_frame = self.create_save_cancel_frame(self)
        save_cancel_frame.pack(side=tk.BOTTOM, fill=tk.BOTH,
                               pady=(0, 0), padx=(0, 0))

    def create_title_frame(self, container):
        frame = tk.Frame(container, bg=MENU_COLOR)

        # Create header label
        title_font = tfont.Font(family="Helvetica", size=28, weight="bold")
        settings_label = tk.Label(frame, text="Settings", bg=MENU_COLOR,
                                  fg=TITLE_COLOR, font=title_font)
        settings_label.pack(side=tk.LEFT, fill=tk.BOTH,
                            pady=(20, 20), padx=(20, 0))

        return frame

    # Create the settings frame
    def create_settings_frame(self, container):
        frame = tk.Frame(container, bg=BACKGROUND_COLOR)

        # Specify layout
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=3)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=4)

        # Create parameter labels and entry fields
        param_font = tfont.Font(family="Helvetica", size=16)
        for idx, name in enumerate(self.parameters):
            param_label = tk.Label(frame, text=name, bg=BACKGROUND_COLOR,
                                   fg=TITLE_COLOR, font=param_font)
            param_label.grid(row=idx, column=0, stick="w",
                             padx=(20, 0), pady=(20, 0))

            param_entry = tk.Entry(frame, width=30,
                                   textvariable=self.parameter_vars[idx])
            param_entry.grid(row=idx, column=1, stick="we",
                             padx=(10, 0), pady=(18, 0))
            self.entry_fields.append(param_entry)

        # Add a validation check for plot accuracy entries.
        # Only allow float input
        def float_validation(s):
            return s.isnumeric() or s in ["."]
        vcmd = (self.register(float_validation), "%S")
        self.entry_fields[3]["validate"] = "key"
        self.entry_fields[3]["validatecommand"] = vcmd

        return frame

    # Create save and cancel buttons on settings page
    def create_save_cancel_frame(self, container):
        frame = tk.Frame(container, bg=BACKGROUND_COLOR)

        # Specify layout
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)

        # Create cancel button
        cancel_button = tk.Button(frame, text="cancel",
                                  command=self.click_cancel,
                                  width=6, bg=MENU_COLOR)
        cancel_button.grid(row=0, column=0, sticky="E",
                           padx=(0, 20), pady=(0, 20))

        # Create save button
        start_button = tk.Button(frame, text="save", command=self.click_save,
                                 width=6, bg=MENU_COLOR)
        start_button.grid(row=0, column=1, stick="W",
                          padx=(20, 0), pady=(0, 20))

        return frame

    @staticmethod
    # Create the image frame at the bottom of the settings page
    def create_image_frame(container):
        frame = tk.Frame(container, bg=MENU_COLOR)

        # Create formula student logo
        kth_image = Image.open(KTHFS_IMAGE_PATH)
        kth_image = kth_image.resize((252, 120), Image.ANTIALIAS)

        logo = ImageTk.PhotoImage(kth_image)
        logo_label = tk.Label(frame, image=logo, bg=MENU_COLOR)
        logo_label.image = logo

        logo_label.pack(side=tk.BOTTOM, fill=tk.BOTH,
                        pady=(0, 10), padx=(0, 0))

        return frame

    # Button functions:
    # Executed when cancel is pressed
    def click_cancel(self):
        # Update graph settings
        self.setting_project_name_var.set(
            self.m_graph.project_name_var.get())
        self.plot_accuracy.set(self.m_graph.plot_accuracy)
        self.inner_func.set(self.last_inner_func)
        self.outer_func.set(self.last_outer_func)

        # Reset entry field colors
        self.color_entry_fields([True, True, True, True])

        # Change back to graph page
        self.parent.graph_page.tkraise()

    # Executed when save is pressed
    def click_save(self):
        # Test if the values in the func entry fields are valid
        parsed_inner_func = self.m_graph.string_to_lambda(
                                        self.inner_func.get())
        parsed_outer_func = self.m_graph.string_to_lambda(
                                        self.outer_func.get())

        if parsed_inner_func and parsed_outer_func:
            # Update graph settings
            self.m_graph.project_name_var.set(
                self.setting_project_name_var.get())
            self.m_graph.inner_func = parsed_inner_func
            self.m_graph.outer_func = parsed_outer_func
            self.last_inner_func = self.inner_func.get()
            self.last_outer_func = self.outer_func.get()
            self.m_graph.vectorize_functions()
            self.m_graph.plot_accuracy = \
                float(self.plot_accuracy.get())

            # Reset entry field colors
            self.color_entry_fields([True, True, True, True])

            # Change to graph page
            self.parent.graph_page.tkraise()

        else:
            # If invalid func mark the field red
            self.color_entry_fields([True, parsed_inner_func,
                                     parsed_outer_func, True])

    # Change color of entry fields between background color and error color
    def color_entry_fields(self, states):
        for (field, state) in zip(self.entry_fields, states):
            field["bg"] = BACKGROUND_COLOR if state else ERROR_COLOR


class GraphPage(tk.Frame):
    """
    The Main Page displaying the graph and toolbar
    """

    def __init__(self, m_graph, parent, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent = parent
        self.m_graph = m_graph

        # Init variables
        self.freq_var = tk.StringVar()
        self.freq_var.set(DEFAULT_FREQUENCY)

        # Create container containing the graph view
        graph_view = self.create_graph_view(self)
        graph_view.pack(side=tk.TOP, fill=tk.BOTH,
                        pady=(0, 0), padx=(0, 0), expand=True)

        # Create container containing the tool bar
        tool_bar = self.create_tool_bar(self)
        tool_bar.pack(side=tk.BOTTOM, fill=tk.BOTH, pady=(0, 0), padx=(0, 0))

    # Create the graph frame
    def create_graph_view(self, container):
        # Create container for the graph
        frame = tk.Frame(container, bg=BACKGROUND_COLOR)
        frame.grid_rowconfigure(1, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        # Create and place the title view
        title_view = self.create_title_view(frame)
        title_view.grid(column=0, row=0, sticky="NSWE", padx=(0, 0))

        # Create, place and animate graph
        canvas = self.m_graph.get_tk_canvas(frame)
        graph_widget = canvas.get_tk_widget()
        graph_widget.grid(column=0, row=1, sticky="NSWE",
                          padx=(0, 0), pady=(2, 0))
        self.m_graph.animate()

        return frame

    # Create title view
    def create_title_view(self, container):
        # Create Container for title view
        title_frame = tk.Frame(container, bg=MENU_COLOR)
        title_frame.grid_rowconfigure(0, weight=1)
        title_frame.grid_columnconfigure(0, weight=1)
        title_frame.grid_columnconfigure(1, weight=1)

        # Create project title
        title_font = tfont.Font(family="Helvetica", size=24, weight="bold")
        title = tk.Label(title_frame,
                         textvariable=self.m_graph.project_name_var,
                         bg=MENU_COLOR, fg=TITLE_COLOR, font=title_font)
        title.grid(column=0, row=0, sticky="",
                   padx=(20, 0), pady=(8, 0), stick="W")

        # Create formula student logo
        kth_image = Image.open(KTHFS_IMAGE_PATH)
        kth_image = kth_image.resize((147, 70), Image.ANTIALIAS)

        logo = ImageTk.PhotoImage(kth_image)
        logo_label = tk.Label(title_frame, image=logo, bg=MENU_COLOR)
        logo_label.image = logo

        logo_label.grid(column=1, row=0, sticky="E",
                        padx=(0, 15), pady=(0, 0))

        return title_frame

    # Create tool bar at the bottom of graph page
    def create_tool_bar(self, container):
        frame = tk.Frame(container, bg=MENU_COLOR, height=200)

        # Init grid layout to cover the whole frame
        frame.grid_columnconfigure(6, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        n = 0  # For easier column placment of components

        # Create Settings button
        settings_image = Image.open(SETTINGS_IMAGE_PATH)
        settings_image = settings_image.resize((12, 12), Image.ANTIALIAS)
        settings_logo = ImageTk.PhotoImage(settings_image)

        settings_button = ttk.Button(frame, command=self.click_settings,
                                     width=.2, image=settings_logo)
        settings_button.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        settings_button.image = settings_logo
        n += 1

        # Create frequency entry:
        # Validtation check for entry
        # Return true if the input is an integer shorter than three digits
        def freq_validation(s, p):
            return s.isnumeric() and len(p) < 3

        freq_vcmd = (self.parent.register(freq_validation), "%S", "%P")

        # Tkinter Entry component
        freq_entry = ttk.Entry(frame, width=2, validate="key",
                               textvariable=self.freq_var,
                               validatecommand=freq_vcmd)
        freq_entry.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        n += 1

        # Create frequency label
        freq_label = tk.Label(frame, text="Hz", bg=MENU_COLOR)
        freq_label.grid(column=n, row=0, sticky="NSWE",
                        padx=(0, 8), pady=(1, 1))
        n += 1

        # Create Start button
        start_button = ttk.Button(frame, text="start",
                                  command=self.click_start, width=4)
        start_button.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        n += 1

        # Create Stop button
        stop_button = ttk.Button(frame, text="stop",
                                 command=self.click_stop, width=4)
        stop_button.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        n += 1

        # Create Reset button
        reset_button = ttk.Button(frame, text="reset",
                                  command=self.click_reset, width=4)
        reset_button.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        n += 1

        # Create zoom scale
        zoom_scale1 = ttk.Scale(
            frame, from_=self.m_graph.zoom_min,
            to=self.m_graph.zoom_max, variable=self.m_graph.zoom,
            cursor="sb_h_double_arrow")
        zoom_scale1.grid(column=n, row=0, sticky="NSWE", padx=(5, 5))
        n += 1

        # Create checkbutton for grid
        grid_check_button = ttk.Checkbutton(frame, text="grid",
                                            var=self.m_graph.grid)
        grid_check_button.grid(column=n, row=0, sticky="NSWE", padx=(0, 0))
        n += 1

        # Create csv save button
        csv_save_button = ttk.Button(frame, text="Save CSV", width=8,
                                     command=self.m_graph.save_csv)
        csv_save_button.grid(column=n, row=0, sticky="NSWE", padx=(3, 0))
        n += 1

        # Create latex save button
        latex_save_button = ttk.Button(frame, text="Save LATEX", width=10,
                                       command=self.m_graph.save_latex)
        latex_save_button.grid(column=n, row=0, sticky="NSWE", padx=(3, 0))
        n += 1

        return frame

    # Button functions:
    # Executed when start is clicked
    def click_start(self):
        self.m_graph.live = True
        self.m_graph.playing = True
        self.m_graph.frequency = float(self.freq_var.get())

    # Executed when settings is clicked
    def click_settings(self):
        self.parent.setting_page.tkraise()

    # Executed when stop is clicked
    def click_stop(self):
        self.m_graph.playing = False

    # Executed when reset is clicked
    def click_reset(self):
        self.m_graph.live = False
        self.m_graph.timestep = 1
        self.m_graph.zoom.set(DEFAULT_ZOOM)


class Graph:
    """
    The Graph class makes upp the graph and owns all graph related variables
    """

    def __init__(self, parent):
        self.parent = parent

        # Default params
        self.inner_func = self.string_to_lambda(DEFAULT_INNER_FUNC)
        self.outer_func = self.string_to_lambda(DEFAULT_OUTER_FUNC)
        self.v_inner_func = np.vectorize(self.inner_func)
        self.v_outer_func = np.vectorize(self.outer_func)
        self.frequency = DEFAULT_FREQUENCY
        self.grid = tk.BooleanVar()
        self.grid.set(DEFAULT_GRID_STATE)
        self.zoom = tk.DoubleVar()
        self.zoom.set(DEFAULT_ZOOM)
        self.zoom_max = DEFAULT_MAX_ZOOM
        self.zoom_min = DEFAULT_MIN_ZOOM
        self.project_name_var = tk.StringVar()
        self.project_name_var.set(DEFAULT_PROJECT_NAME)
        self.plot_accuracy = DEFAULT_PLOT_ACCURACY
        self.plot_update_frequency = PLOT_REFRESH_FREQUENCY
        self.static_line_color = STATIC_LINE_COLOR
        self.live_line_color = LIVE_LINE_COLOR
        self.title = PLOT_TITLE

        # Init variables
        self.subtitles = ["Static", "Live"]
        self.live = False
        self.playing = False
        self.timestep = 1

        # Create plot
        self.anim = None
        self.fig, self.ax, self.line = self.create_plot()

    # Init plot
    def create_plot(self):
        # Create figure
        fig = plt.figure()
        ax = fig.add_subplot(111)

        # Define x range
        x_values = np.arange(0, self.zoom.get(),
                             self.plot_accuracy)

        # Create plot
        line1 = self.plot(fig=fig, ax=ax, x_values=x_values)

        return fig, ax, line1

    # Create plot
    def plot(self, fig, ax, x_values):
        # Plot graph
        color = self.live_line_color if self.live else self.static_line_color
        line1, = ax.plot(x_values,
                         self.v_outer_func(self.v_inner_func(x_values)),
                         color=color)

        # Set title
        fig.suptitle(self.title, fontsize=10)
        ax.set_title(self.subtitles[1 if self.live else 0], fontsize=8)

        # Toggle grid
        ax.grid(self.grid.get())

        return line1

    # The animation function which is executed every plot refresh cycle
    def anim_update(self, *args):
        if self.live:
            # Define x range
            x_values = np.arange(0, self.timestep, self.plot_accuracy)
            if self.playing:
                self.timestep += self.frequency / self.plot_update_frequency
        else:
            # Define x range
            x_values = np.arange(0, self.zoom.get(), self.plot_accuracy)

        # Plot
        self.ax.clear()
        self.line = self.plot(fig=self.fig, ax=self.ax, x_values=x_values)

    # Start the animation
    def animate(self):
        self.anim = animation.FuncAnimation(
            fig=self.fig, func=self.anim_update,
            interval=self.plot_update_frequency)

    # End the animation
    def stop_anim(self):
        self.anim = None

    # Return tkinter type canvas
    def get_tk_canvas(self, container):
        return FigureCanvasTkAgg(self.fig, container)

    # Save the current plot as a tex file for usage in LATEX
    def save_latex(self):
        # Get file save path
        file_path = asksaveasfilename(
            initialfile=self.project_name_var.get() + ".tex",
            filetypes=[("Tex Files", ".tex")])

        if file_path:
            # Remove titles to make latex graph cleaner
            self.ax.set_title("")
            self.fig.suptitle("")

            # Save
            tikzplotlib.clean_figure()
            tikzplotlib.save(file_path)

    # Generate a CSV file from graph data
    def save_csv(self):
        # Get file save path
        file_path = asksaveasfilename(
            initialfile=self.project_name_var.get() + ".csv",
            filetypes=[("CSV Files", "*.csv")])

        if file_path:
            # Define x range
            x_values = np.arange(0, self.zoom.get(),
                                 self.plot_accuracy)

            # Create file
            with open(file_path, 'w', newline='') as file:
                writer = csv.writer(file)

                # Write column titles
                writer.writerow(["t", "h(t)", "lambda(t)", "h(lambda(t))"])

                # Write data
                for v in x_values:
                    writer.writerow([v, self.outer_func(v), self.inner_func(v),
                                     self.outer_func(self.inner_func(v))])

    # Vectorice the lambda functions to be able to create x ranges
    def vectorize_functions(self):
        self.v_inner_func = np.vectorize(self.inner_func)
        self.v_outer_func = np.vectorize(self.outer_func)

    @staticmethod
    # Parses a string to lambda function
    def string_to_lambda(input_str):
        try:
            l_func = lambda t: eval(input_str)
            l_func(1)   # Test if func works
            return l_func

        # If func is invalid return false
        except SyntaxError:
            return False


if __name__ == "__main__":
    app = App()
    app.mainloop()
