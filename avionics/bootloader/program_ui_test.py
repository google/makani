# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import unittest

from makani.avionics.bootloader import program_ui
import mock


class ProgramUiTest(unittest.TestCase):
  dialog_ok = program_ui.ui_helper.dialog.Dialog.OK
  dialog_cancel = program_ui.ui_helper.dialog.Dialog.CANCEL

  def setUp(self):
    # Patch AioClient class and its Recv method.
    self._patch_aio = mock.patch(target='program_ui.aio.AioClient', spec=True)
    self._patch_aio.start()
    self._patch_aio_recv = program_ui.aio.AioClient.return_value.Recv
    self._patch_aio_recv.side_effect = socket.timeout('No socket allowed.')

    # Patch UserInterface class and its Menu method.
    self._patch_ui = mock.patch(target='program_ui.ui_helper.UserInterface',
                                spec=True)
    self._patch_ui.start()
    self._patch_ui_menu = program_ui.ui_helper.UserInterface.return_value.Menu
    self._patch_ui_menu.side_effect = [(self.dialog_ok, '')]
    self._patch_ui_input = program_ui.ui_helper.UserInterface.return_value.Input
    self._patch_ui_input.side_effect = [(self.dialog_ok, '')]

    # Patch RunProcess method.
    self._patch_proc_runner = mock.patch(
        target='program_ui.process_helper.RunProcess',
        spec=True)
    self._patch_proc_runner.start()

    # Replace ProgramWrapper's GetFirmwareFiles method. This removes tms570-bin
    # dependency.
    program_ui.ProgramWrapper.GetFirmwareFiles = mock.Mock(
        return_value=['calib_file1', 'calib_file2'])

    # Replace DetectAio with network failure response.
    program_ui.DetectAio = mock.Mock(return_value=(tuple(), tuple()))

  def ProgramArgumentChecker(self, mocked_call, exact_arguments):
    self.assertEqual(mocked_call.call_count, 1)
    self.assertTrue(mocked_call.call_args[0][0][1].endswith('program.py'))
    self.assertEqual(mocked_call.call_args[0][0][2:], exact_arguments)

  def PrepareForProgram(self):
    # Set return of calls to RunProcess (likely calling program.py).
    program_ui.process_helper.RunProcess.return_value = (0, '', '')
    program_ui.process_helper.RunProcess.reset_mock()

    # Reset UserInterface mock.
    self._patch_ui_menu.reset_mock()

  def GetNodeFromSnake(self, node_snake):
    assert isinstance(node_snake, str)
    for node in program_ui._NETWORK_CONFIG.aio_nodes:
      if getattr(node, 'snake_name', '') == node_snake:
        return node
    raise RuntimeError('Invalid node name: %s.', node_snake)

  def PreloadInput(self, answers):
    self._patch_ui_input.side_effect = [(self.dialog_ok, answer) for answer in
                                        answers]

  def PreloadMenu(self, answers):
    self._patch_ui_menu.side_effect = [(self.dialog_ok, answer) for answer in
                                       answers]

  def PreloadMenuNodeSelect(self, node):
    self.PreloadMenu([node.label_name, str(node.enum_value)])

  def CreateProgramWrapper(self, node_snake):
    node = self.GetNodeFromSnake(node_snake)
    category = node.label_name

    # DetectAio will fail, user will select node_snake.
    self.PreloadMenuNodeSelect(node)

    p1 = program_ui.ProgramWrapper()

    # Verify that category was offered.
    menu_1_options = p1.dialog_ui.Menu.call_args_list[0][0]
    codenames, desc = zip(*menu_1_options[1])
    self.assertIn(category, codenames)

    # Verify that node_snake was offered.
    menu_2_options = p1.dialog_ui.Menu.call_args_list[1][0]
    codenames, desc = zip(*menu_2_options[1])
    self.assertIn(str(node.enum_value), codenames)
    self.assertIn(node_snake, desc)
    self.assertEqual(codenames.index(str(node.enum_value)),
                     desc.index(node.snake_name))
    return p1

  def SetIsSubset(self, subset, superset):
    self.assertTrue(subset.issubset(superset),
                    '{} not found in {}.'.format(subset, superset))

  def testGlobals(self):
    self.assertTrue(hasattr(program_ui.program, 'SERIAL_PARAM_FILE_TEMPLATE'),
                    'Missing global variable: SERIAL_PARAM_FILE_TEMPLATE.')

  def testProgramApplication(self):
    self.PrepareForProgram()
    node = self.GetNodeFromSnake('servo_a1')

    # Create a ProgramWrapper instance.
    p1 = self.CreateProgramWrapper(node.snake_name)

    # Test ProgramApplication.
    p1.ProgramApplication()
    self.ProgramArgumentChecker(program_ui.process_helper.RunProcess,
                                exact_arguments=[node.snake_name])

  def testProgramBootloader(self):
    self.PrepareForProgram()
    node = self.GetNodeFromSnake('servo_a1')

    # Create a ProgramWrapper instance.
    p1 = self.CreateProgramWrapper(node.snake_name)

    # Test ProgramBootloader.
    p1.ProgramBootloader()
    self.ProgramArgumentChecker(program_ui.process_helper.RunProcess,
                                exact_arguments=[node.snake_name,
                                                 '--bootloader'])

  def testProgramRename(self):
    self.PrepareForProgram()
    node = self.GetNodeFromSnake('servo_a1')
    node_rename_to = self.GetNodeFromSnake('servo_a2')

    # Create a ProgramWrapper instance.
    p1 = self.CreateProgramWrapper(node.snake_name)

    # RenameNode: Select rename_to Node.
    self.PreloadMenuNodeSelect(node_rename_to)

    # RenameNode: Setup Recv mock to return version data.
    mock_header = mock.Mock()
    mock_header.version = 0xFFFF
    self._patch_aio_recv.side_effect = None
    self._patch_aio_recv.return_value = (node_rename_to.ip, mock_header, None)

    # Test RenameNode.
    p1.RenameNode()
    self.ProgramArgumentChecker(program_ui.process_helper.RunProcess,
                                exact_arguments=[node.snake_name, '--rename_to',
                                                 node_rename_to.snake_name])

  def testProgramSerial(self):
    self.PrepareForProgram()
    node = self.GetNodeFromSnake('servo_a1')

    # Create a ProgramWrapper instance.
    p1 = self.CreateProgramWrapper(node.snake_name)

    # ProgramSerial: Select hardware type and type in serial.
    hardware_type_rev = ['aio', 'rev_ab']
    serial_number = 'my_test_serial'
    self.PreloadMenu(hardware_type_rev)
    self.PreloadInput([serial_number])

    # Test ProgramBootloader.
    p1.ProgramSerial()
    self.ProgramArgumentChecker(
        program_ui.process_helper.RunProcess,
        exact_arguments=([node.snake_name, '--serial'] + hardware_type_rev +
                         [serial_number]))

  def testProgramCalib(self):
    self.PrepareForProgram()
    node = self.GetNodeFromSnake('motor_sbo')

    # Create a ProgramWrapper instance.
    p1 = self.CreateProgramWrapper(node.snake_name)

    # ProgramCalib: Select calibration.
    calib = p1.GetCalibNames()[0]
    self.PreloadMenu([calib])

    # Test ProgramBootloader.
    p1.ProgramCalib()
    self.ProgramArgumentChecker(
        program_ui.process_helper.RunProcess,
        exact_arguments=([node.snake_name, '--calib', calib]))

  def testNodeSelectMenu(self):
    self._patch_ui_menu.reset_mock()
    self.CreateProgramWrapper('servo_a1')
    self._patch_ui_menu.reset_mock()
    self.CreateProgramWrapper('cs_a')

  def testGetSerialRevisions(self):
    self.SetIsSubset({'rev_ab', 'rev_ac', 'rev_ad', 'rev_ba'},
                     program_ui.GetSerialRevisions())
    self.SetIsSubset({'gin_a2', 'gin_a3'},
                     program_ui.GetSerialRevisions(serial_type='motor'))
    self.SetIsSubset({'rev_ac', 'rev_ad_clk8'},
                     program_ui.GetSerialRevisions(serial_type='cs'))

  def testGetSerialTypes(self):
    self.SetIsSubset({'aio', 'motor', 'cs'}, program_ui.GetSerialTypes())

  def tearDown(self):
    self._patch_proc_runner.start()
    self._patch_ui.stop()
    self._patch_aio.stop()


if __name__ == '__main__':
  unittest.main()

