"""
    TODO
"""

import dearpygui.dearpygui as dpg

from osgar.logger import lookup_stream_names, LogReader
from osgar.lib.serialize import deserialize


class OsgarView:
    def __init__(self):
        self.log_path = None
        self.log_data = None

        # tag ids
        self.main_win_id = "main_win_id"
        self.main_setting_id = "main_setting_id"
        self.load_log_id = "load_log_id"
        self.streams_tab_id = "streams_tab_id"
        self.map_win_id = "map_win_id"
        self.main_plot_id = "main_plot_id"
        self.map_x_axis_id = "map_x_axis_id"
        self.map_y_axis_id = "map_y_axis_id"

    @staticmethod
    def clean_item_children(parent_id, key = 1):
        children = dpg.get_item_children(parent_id, key)
        for child in children:
            dpg.delete_item(child)

    def fill_stream_table(self, streams, counts, sizes, last_timestamp):  # TODO metadata for some streams
        self.clean_item_children(self.streams_tab_id)  # delete table first
        for stream, size, count in zip(streams, sizes, counts):
            with dpg.table_row(parent=self.streams_tab_id):
                dpg.add_text(stream)
                dpg.add_text(f"{count}")
                dpg.add_text(f"{size}")
                dpg.add_text(f"{count/last_timestamp.total_seconds():.1f} Hz")
                dpg.add_button(label="Setting", callback=None)

    def load_log_callback(self, sender, app_data):
        log_path = app_data["file_path_name"]
        assert log_path.endswith(".log"), log_path
        self.log_path = log_path
        self.load_data()

    def set_streams_callback(self, sender, app_data):
        print(sender)

    def pay_callback(self):
        pass

    def back_callback(self):
        pass

    def pause_callback(self):
        pass

    def stop_callback(self):
        pass

    def load_data(self):
        if self.log_path is None:
            return
        self.log_data = []
        names = ['sys'] + lookup_stream_names(self.log_path)
        sizes = [0] * len(names)
        counts = [0] * len(names)
        with LogReader(self.log_path) as log:
            for timestamp, stream_id, data in log:
                sizes[stream_id] += len(data)
                counts[stream_id] += 1
                if stream_id != 0:
                    data = deserialize(data)
                self.log_data.append([timestamp.total_seconds(), names[stream_id], data])
            last_timestamp = timestamp
        self.fill_stream_table(names, counts, sizes, last_timestamp)

    def main(self):
        dpg.create_context()
        dpg.create_viewport(title='Osgar viewer', width=1600, height=1000)
        dpg.add_window(tag=self.main_win_id)
        # Main bar
        with dpg.viewport_menu_bar():
            with dpg.menu(label="File"):
                pass

            with dpg.menu(label="Tools"):
                pass

            dpg.add_menu_item(label="Help", callback=None)

        with dpg.child_window(parent=self.main_win_id, label="Main setting", tag=self.main_setting_id, pos=(0, 20), width=380, height=900):
            with dpg.file_dialog(
                    directory_selector=False, show=False, callback=self.load_log_callback, tag=self.load_log_id,
                    width=700, height=400):
                dpg.add_file_extension("Log files (*.log){.log}")

            with dpg.group(horizontal=True):
                dpg.add_button(label="Load log", callback=lambda: dpg.show_item(self.load_log_id))
            with dpg.group(horizontal=True):
                dpg.add_button(label="Play", callback=self.pay_callback)
                dpg.add_button(label="Back", callback=self.back_callback)
                dpg.add_button(label="Pause", callback=self.pause_callback)
                dpg.add_button(label="Stop", callback=self.stop_callback)

            with dpg.collapsing_header(label="Available streams"):
                with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp, tag=self.streams_tab_id):
                    dpg.add_table_column(label="Stream name")
                    dpg.add_table_column(label="Count")
                    dpg.add_table_column(label="Size        ")
                    dpg.add_table_column(label="Freq.")
                    dpg.add_table_column(label="      ")

        with dpg.child_window(parent=self.main_win_id, tag=self.map_win_id, pos=(380, 20), width=-1, height=-1):
            dpg.add_button(label="Set streams", callback=self.set_streams_callback, tag="map_win")
            with dpg.plot(pos=(0, 30), height=-1, width=-1, tag=self.main_plot_id, equal_aspects=True):
                dpg.add_plot_axis(dpg.mvXAxis, label="x", tag=self.map_x_axis_id)
                dpg.add_plot_axis(dpg.mvYAxis, label="y", tag=self.map_y_axis_id)

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window(self.main_win_id, True)
        # while dpg.is_dearpygui_running():
            # insert here any code you would like to run in the render loop
            # you can manually stop by using stop_dearpygui()
            # print("this will run every frame")
        #    dpg.render_dearpygui_frame()
        dpg.start_dearpygui()
        dpg.destroy_context()

    # context manager functions
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, traceback):
        pass

if __name__ == "__main__":
    with OsgarView() as viewer:
        viewer.main()
