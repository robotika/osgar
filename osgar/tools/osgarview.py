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

        self.channel_types_dic = {
            "image": ["image", "color"],
            "depth": ["depth"],
            "scan": ["scan"],
            "scan3d": [],
            "pose2d": ["pose2d"],
            "pose3d": ["pose3d"],
            "gps": ["position"],
            "gps_utm": [],
            "numeric": ["desired_speed", "encoders"]
        }

        # tag ids
        self.main_setting_id = "main_setting_id"
        self.load_log_id = "load_log_id"
        self.streams_tab_id = "streams_tab_id"

    @staticmethod
    def clean_item_children(parent_id, key = 1):
        children = dpg.get_item_children(parent_id, key)
        for child in children:
            dpg.delete_item(child)

    def get_stream_type(self, streams):
        ret = []
        for stream in streams:
            channel = stream.split(".")[1]
            for channel_type, channel_names in self.channel_types_dic.items():
                if channel in channel_names:
                    ret.append([stream, channel_type])
                    break
            else:
                ret.append([stream, "NaN"])
        return ret

    def fill_stream_table(self, streams):  # TODO metadata for some streams
        self.clean_item_children(self.streams_tab_id)  # delete table first
        streams_and_types = self.get_stream_type(streams)
        for stream, channel_type in streams_and_types:
            with dpg.table_row(parent=self.streams_tab_id):
                for item in [stream, channel_type]:
                    dpg.add_text(item)
        # Add setting button
        for row_id in dpg.get_item_children(self.streams_tab_id, 1):
            dpg.add_button(parent=row_id, label="Setting", callback=None)

    def load_log_callback(self, sender, app_data):
        log_path = app_data["file_path_name"]
        assert log_path.endswith(".log"), log_path
        streams = lookup_stream_names(log_path)
        self.fill_stream_table(streams)
        self.log_path = log_path

    def load_data_callback(self, sender, app_data):
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

    def main(self):
        dpg.create_context()
        dpg.create_viewport(title='Osgar viewer', width=1600, height=1000)
        # Main bar
        with dpg.viewport_menu_bar():
            with dpg.menu(label="File"):
                pass

            with dpg.menu(label="Tools"):
                pass

            dpg.add_menu_item(label="Help", callback=None)

        with dpg.window(label="Main setting", tag=self.main_setting_id, pos=(0, 20), width=380, height=900):
            with dpg.file_dialog(
                    directory_selector=False, show=False, callback=self.load_log_callback, tag=self.load_log_id,
                    width=700, height=400):
                dpg.add_file_extension("Log files (*.log){.log}")

            with dpg.group(horizontal=True):
                dpg.add_button(label="Load log", callback=lambda: dpg.show_item(self.load_log_id))
                dpg.add_button(label="Load data", callback=self.load_data_callback)

            with dpg.collapsing_header(label="Available streams"):
                with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp, tag=self.streams_tab_id):
                    dpg.add_table_column(label="Stream name")
                    dpg.add_table_column(label="Type")
                    dpg.add_table_column(label="Count")
                    dpg.add_table_column(label="Size")
                    dpg.add_table_column(label="Freq.")
                    dpg.add_table_column(label="      ")

        dpg.setup_dearpygui()
        dpg.show_viewport()
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
