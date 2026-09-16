"""Exercise window shutdown without connecting to a Crazyflie."""
from contextlib import ExitStack
import os
import tempfile
import unittest
from unittest.mock import Mock, patch


class ShutdownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import cfclient
        from cfclient.utils.config import Config

        resources = ExitStack()
        cls.addClassCleanup(resources.close)
        config_dir = resources.enter_context(tempfile.TemporaryDirectory())
        resources.enter_context(patch.object(cfclient, 'config_path', config_dir))
        resources.enter_context(patch.object(Config, '_instances', {}))
        resources.enter_context(patch.dict(os.environ, JOYSTREAM='0',
                                           QT_QPA_PLATFORM='offscreen'))

        from PyQt6.QtWidgets import QApplication, QMainWindow
        from cfclient.ui.main import MainUI
        from cfclient.ui.tab_toolbox import TabToolbox

        cls.app = QApplication.instance() or QApplication([])
        cls.MainUI = MainUI
        cls.QMainWindow = QMainWindow
        cls.Config = Config
        cls.TabToolbox = TabToolbox

    def check_shutdown(self, with_vicon):
        self.Config().set('open_tabs', 'Flight Control')
        self.Config().set('open_toolboxes', '')
        self.Config().save_file()

        # Use Qt's real close-event dispatch, without starting hardware workers.
        window = self.MainUI.__new__(self.MainUI)
        self.QMainWindow.__init__(window)
        self.addCleanup(window.deleteLater)
        window.cf = Mock(spec=['close_link'])
        if with_vicon:
            window.vicon_worker = Mock(spec=['stop'])

        tab = self.TabToolbox(None, 'Parameters')
        self.addCleanup(tab.deleteLater)
        tab.set_display_state(self.TabToolbox.DS_TAB)
        toolbox = self.TabToolbox(None, 'Console')
        self.addCleanup(toolbox.deleteLater)
        toolbox.set_display_state(self.TabToolbox.DS_TOOLBOX)

        self.assertTrue(window.close())
        window.cf.close_link.assert_called_once_with()
        if with_vicon:
            window.vicon_worker.stop.assert_called_once_with()

        # Reload from disk to verify the next client launch restores the layout.
        self.Config._instances.pop(self.Config)
        self.assertEqual(self.TabToolbox.read_open_tab_config(),
                         ['Flight Control', 'Parameters'])
        self.assertEqual(self.TabToolbox.read_open_toolbox_config(), ['Console'])

    def test_close_saves_layout_and_closes_radio(self):
        self.check_shutdown(with_vicon=False)

    def test_close_also_stops_optional_vicon_worker(self):
        self.check_shutdown(with_vicon=True)
