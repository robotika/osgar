"""
    TODO
"""

import dearpygui.dearpygui as dpg

from osgar.logger import lookup_stream_names


class OsgarView:
    def __init__(self):
        self.log_path = None
        self.log_data = None

        # tag ids
        self.main_setting_id = "main_setting_id"
        self.load_log_id = "load_log_id"

    def load_log_callback(self, sender, app_data):
        log_path = app_data["file_path_name"]
        assert log_path.endswith(".log"), log_path
        names = lookup_stream_names(log_path)
        print(names)

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
